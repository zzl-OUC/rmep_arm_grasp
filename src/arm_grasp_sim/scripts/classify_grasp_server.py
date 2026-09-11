#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验三 扩展抓取控制器: ClassifyGrasp action server。

与已验收的 grasp_controller.py (固定 A->B) 解耦:
  - 复用同一运动层(速度跟踪 + IK + 括架补偿 + 平行爪)
  - 网格 -> (yaw, d) 参数化: 由 grid_id 查网格世界坐标, yaw=atan2(y,x), d=|p|+TCP_LEAD
  - 料盒 -> (yaw, d): bin_0/bin_1 同理
  - 放置: IK 把 TCP 对准盒心上方 Z_DROP(块底高于盒壁), 开爪投放
  - 抓取失败/不可达 -> 返回 success=False + message, 由任务节点决定重试/跳过

ClassifyGrasp.action (新增):
  string grid_id     # cell_1..cell_6
  string class_id    # green_block / yellow_block
  string bin_id      # bin_0 / bin_1
  ---
  bool success
  string bin_id
  string message
  ---
  string current_state
  float32 progress
  bool gripper_closed
"""
import math
import time as _time
import numpy as np
from scipy.optimize import least_squares

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from arm_grasp_interfaces.action import ClassifyGrasp
from gazebo_msgs.srv import GetEntityState, SetModelConfiguration
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState

# ---- 官方几何常数(与 grasp_controller.py 完全一致) ----
TABLE_TOP = 0.025
ARM1_X = 0.0103961
ARM1_Z = 0.03465 + 0.0906477 + 0.030741
A1X, A1Z = 0.0018704, 0.1210238
A2X, A2Z = 0.1058557, -0.0561093
TCPX, TCPZ = 0.124 + 0.0002793, -0.039 + 0.0001815
TCP_LEAD = 0.066
Q1_LIM = (-0.274, 1.384)
Q2_LIM = (-1.25, 0.40)

GRIP_OPEN = (-5.0, -5.0)
GRIP_SEAT = (5.0, 5.0)
GRIP_CLOSE = (15.0, 15.0)

Q_GRASP = (1.384, -0.4714)
Q_LIFT_Q1 = 1.20
Q_CARRY = (0.90, -0.30)   # 搬运姿态: TCP z=0.110, 方块底悬空~6.5cm
Q_RAISED = (0.9, -0.3)
Z_DROP = 0.115            # 投放高度: 块底 ~0.095 高于盒壁顶 0.085

MOVE_DUR = 3.0
LIFT_DUR = 3.0
YAW_DUR = 6.0
GRIP_SEAT_DUR = 3.0
VC = 0.9
KV = 4.0

# ---- 网格/料盒世界坐标(与 table_grid_4c2b.world desk_layout 一致) ----
GRIDS = {
    'cell_1': (0.190, 0.000),
    'cell_2': (0.095, 0.1645),
    'cell_3': (-0.095, 0.1645),
    'cell_4': (-0.190, 0.000),
    'cell_5': (0.1645, 0.0950),
    'cell_6': (-0.1645, 0.0950),
}
BINS = {
    'bin_0': (-0.095, -0.1645),   # 绿盒
    'bin_1': (0.095, -0.1645),    # 黄盒
}


def fk_dz(q1, q2):
    th1, th2 = q1, q1 + q2
    d = ARM1_X + A1X * math.cos(th1) + A1Z * math.sin(th1) \
        + A2X * math.cos(th2) + A2Z * math.sin(th2) + TCPX
    z = TABLE_TOP + ARM1_Z - A1X * math.sin(th1) + A1Z * math.cos(th1) \
        - A2X * math.sin(th2) + A2Z * math.cos(th2) + TCPZ
    return d, z


def ik_dz(d, z, seed=(1.0, 0.0)):
    def res(x):
        fd, fz = fk_dz(x[0], x[1])
        return [fd - d, fz - z]
    lo = [Q1_LIM[0], Q2_LIM[0]]
    hi = [Q1_LIM[1], Q2_LIM[1]]
    best = None
    for s0 in [seed, (1.1, 0.0), (1.2, -0.1), (0.9, 0.2), (1.3, -0.2)]:
        r = least_squares(res, x0=np.array(s0, dtype=float), bounds=(lo, hi))
        if best is None or r.cost < best.cost:
            best = r
        if r.cost < 1e-10:
            break
    q1, q2 = float(best.x[0]), float(best.x[1])
    fd, fz = fk_dz(q1, q2)
    err = math.hypot(fd - d, fz - z)
    if err > 2e-3:
        return q1, q2, False, 'IK 不可达 d=%.3f z=%.3f (最优误差 %.1fmm)' % (d, z, err * 1e3)
    return q1, q2, True, 'ok'


class ClassifyGraspServer(Node):
    def __init__(self):
        super().__init__('classify_grasp_server')
        self.declare_parameter('z_grasp', 0.0757)
        self.declare_parameter('block_name', 'block')
        self.declare_parameter('block_side', 0.04)
        self.z_grasp = float(self.get_parameter('z_grasp').value)
        self.block_name = self.get_parameter('block_name').value
        self.block_side = float(self.get_parameter('block_side').value)

        self._cb = ReentrantCallbackGroup()
        self.state_pub = self.create_publisher(String, '/grasp_state', 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/arm_vel_controller/commands', 10)
        self.brk_pub = self.create_publisher(Float64MultiArray, '/bracket_controller/commands', 10)
        self.grip_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.js = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 50)
        self.get_entity = self.create_client(GetEntityState, '/gazebo/get_entity_state',
                                             callback_group=self._cb)
        self._as = ActionServer(
            self, ClassifyGrasp, 'classify_grasp',
            execute_callback=self._execute, goal_callback=self._goal,
            cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=self._cb)
        self.get_logger().info('classify_grasp_server 就绪 (网格=%s 料盒=%s)' % (list(GRIDS), list(BINS)))

    # ---------- 基础(同 grasp_controller) ----------
    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))

    def _arm_q(self):
        try:
            return np.array([float(self.js[n]) for n in
                             ['chassis_yaw_joint', 'arm_1_joint', 'arm_2_joint',
                              'endpoint_bracket_joint']])
        except Exception:
            return np.zeros(4)

    def _pub_vel(self, v3):
        m = Float64MultiArray(); m.data = [float(x) for x in v3]
        self.vel_pub.publish(m)

    def _pub_bracket(self, q):
        m = Float64MultiArray(); m.data = [float(q)]
        self.brk_pub.publish(m)

    def _pub_grip(self, l, r):
        m = Float64MultiArray(); m.data = [float(l), float(r)]
        self.grip_pub.publish(m)

    def _pub_state(self, state):
        m = String(); m.data = state
        self.state_pub.publish(m)
        self.get_logger().info('[state] %s' % state)

    def _wait_fut(self, fut, timeout=2.0):
        t0 = _time.time()
        while rclpy.ok() and not fut.done() and _time.time() - t0 < timeout:
            _time.sleep(0.02)
        return fut.result() if fut.done() else None

    def _block_xyz(self):
        if not self.get_entity.service_is_ready():
            return None
        req = GetEntityState.Request()
        req.name = self.block_name
        s = self._wait_fut(self.get_entity.call_async(req))
        if s is None or not s.success:
            return None
        p = s.state.pose.position
        return p.x, p.y, p.z

    def _bin_min_dist(self, bx, by):
        """查询 block_0..block_5, 返回 (距盒心最近距离, 方块名)。"""
        if not self.get_entity.service_is_ready():
            return -1.0, ''
        best_d, best_n = -1.0, ''
        for i in range(6):
            req = GetEntityState.Request()
            req.name = 'block_%d' % i
            s = self._wait_fut(self.get_entity.call_async(req))
            if s is None or not s.success:
                continue
            p = s.state.pose.position
            d = math.hypot(p.x - bx, p.y - by)
            if best_d < 0 or d < best_d:
                best_d, best_n = d, req.name
        return best_d, best_n

    async def _send_arm(self, yaw, q1, q2, duration):
        steps = max(1, int(float(duration) / 0.01))
        settled = 0
        for _ in range(steps):
            qn = self._arm_q()
            tgt = np.array([yaw, q1, q2])
            err = tgt - qn[:3]
            err[0] = (err[0] + math.pi) % (2 * math.pi) - math.pi
            v = np.clip(KV * err, -VC, VC)
            self._pub_bracket(-(qn[1] + qn[2]))
            if np.max(np.abs(err)) < 0.01:
                settled += 1
                v = np.zeros(3)
            else:
                settled = 0
            self._pub_vel(v)
            _time.sleep(0.01)
            if settled >= 50:
                break
        self._pub_vel(np.zeros(3))
        qn = self._arm_q()
        derr = np.array([yaw, q1, q2]) - qn[:3]
        derr[0] = (derr[0] + math.pi) % (2 * math.pi) - math.pi
        ok = np.max(np.abs(derr)) < 0.06
        return ok, 'yaw=%.3f q1=%.3f q2=%.3f' % tuple(qn[:3])

    async def _goto_tcp(self, yaw, d, z, dur, seed):
        q1, q2, ok, msg = ik_dz(d, z, seed)
        if not ok:
            return False, msg
        ok2, msg2 = await self._send_arm(yaw, q1, q2, dur)
        return ok2, msg2

    def _hold(self, grip_eff, dur):
        q0 = self._arm_q()[:3].copy()
        steps = max(1, int(float(dur) / 0.05))
        for _ in range(steps):
            qn = self._arm_q()
            self._pub_vel(np.clip(KV * (q0 - qn[:3]), -VC, VC))
            self._pub_bracket(-(qn[1] + qn[2]))
            self._pub_grip(*grip_eff)
            _time.sleep(0.05)
        self._pub_vel(np.zeros(3))

    def _safe_stop(self):
        self._pub_grip(*GRIP_OPEN)
        self._pub_vel(np.zeros(3))

    # ---------- Action ----------
    def _goal(self, goal):
        return rclpy.action.GoalResponse.ACCEPT

    async def _execute(self, goal_handle):
        goal = goal_handle.request
        grid = goal.grid_id
        cls = goal.class_id
        bin_id = goal.bin_id
        fb = ClassifyGrasp.Feedback()
        fb.gripper_closed = False

        if grid not in GRIDS or bin_id not in BINS:
            goal_handle.succeed()
            return ClassifyGrasp.Result(success=False, bin_id=bin_id,
                                        message='未知网格/料盒: %s / %s' % (grid, bin_id))
        gx, gy = GRIDS[grid]
        bx, by = BINS[bin_id]
        yawG = math.atan2(gy, gx)
        yawB = math.atan2(by, bx)
        dG = math.hypot(gx, gy) + TCP_LEAD
        dB = math.hypot(bx, by) + TCP_LEAD
        seed = (1.05, 0.0)
        # 与已验收 grasp_controller 一致的关节角(不再自算 IK):
        # fk(Q_GRASP) = (0.2743, 0.0448) 方块中部; Q_RAISED 抬升过渡; Q_LIFT 半抬
        self._pub_state('SCAN_OK')
        self.get_logger().info('任务: %s(%s) -> %s | 抓 d=%.3f yaw=%.2f 放 d=%.3f yaw=%.2f'
                               % (grid, cls, bin_id, dG, yawG, dB, yawB))
        try:
            # HOME 抬臂(yaw 从当前位置转回 0, 按角度差自适应时长)
            self._pub_state('HOME')
            qn0 = self._arm_q()
            home_dur = max(8.0, abs(qn0[0]) / 0.30 + 2.0)
            ok, msg = await self._send_arm(0.0, 1.30, 0.35, home_dur)
            self._pub_grip(*GRIP_OPEN)

            # 接近网格(抬升过渡位, 同 controller Q_RAISED; yaw 时长按角度差自适应)
            self._pub_state('APPROACH_GRID')
            dyaw = abs((yawG - self._arm_q()[0] + math.pi) % (2 * math.pi) - math.pi)
            app_dur = max(MOVE_DUR, dyaw / 0.50 + 3.0)
            ok, msg = await self._send_arm(yawG, Q_RAISED[0], Q_RAISED[1], app_dur)
            if not ok:
                goal_handle.succeed()
                return ClassifyGrasp.Result(success=False, bin_id=bin_id, message=msg)
            # 下降到抓取位(同 controller Q_GRASP, TCP 在方块中部)
            self._pub_state('DESCEND_GRID')
            ok, msg = await self._send_arm(yawG, Q_GRASP[0], Q_GRASP[1], MOVE_DUR)
            if not ok:
                goal_handle.succeed()
                return ClassifyGrasp.Result(success=False, bin_id=bin_id, message=msg)
            # 夹取
            self._pub_state('GRASP')
            self._hold(GRIP_SEAT, GRIP_SEAT_DUR)
            self._hold(GRIP_CLOSE, 1.5)
            fb.gripper_closed = True
            b = self._block_xyz()
            if b is None:
                goal_handle.succeed()
                return ClassifyGrasp.Result(success=False, bin_id=bin_id,
                                            message='抓取后无法查询方块状态')
            # 抬升(半抬, 同 controller Q_LIFT_Q1)
            self._pub_state('LIFT')
            ok, msg = await self._send_arm(yawG, Q_CARRY[0], Q_CARRY[1], LIFT_DUR)
            # 搬运到料盒上方(抬臂姿态 Q_CARRY, 方块悬空; yaw 时长按实测 0.5 rad/s)
            self._pub_state('TRANSPORT')
            yaw_dur = max(YAW_DUR, abs((yawB - yawG + math.pi) % (2 * math.pi) - math.pi) / 0.50 + 3.0)
            ok2, msg2 = await self._send_arm(yawB, Q_CARRY[0], Q_CARRY[1], yaw_dur)
            # 降到料盒上方(IK 对准盒心, 块底保持高于盒壁后投放)
            self._pub_state('DESCEND_BIN')
            ok3, msg3 = await self._goto_tcp(yawB, dB, Z_DROP, LIFT_DUR + 2.0, seed)
            self._pub_state('RELEASE')
            self._hold(GRIP_OPEN, 2.0)
            fb.gripper_closed = False
            self._pub_state('LIFT_BIN')
            await self._send_arm(yawB, Q_CARRY[0], Q_CARRY[1], 2.0)

            if not (ok and ok2 and ok3):
                goal_handle.succeed()
                return ClassifyGrasp.Result(success=False, bin_id=bin_id,
                                            message='运动失败: %s | %s | %s' % (msg, msg2, msg3))
            # 判定: 应有一方块落在盒心 7cm 内
            dist, bn = self._bin_min_dist(bx, by)
            okc = 0 <= dist < 0.07
            self.get_logger().info('[%s->%s] 最近方块=%s 距盒心=%.3f 成功=%s'
                                   % (grid, bin_id, bn, dist, okc))
            self._pub_state('CYCLE_DONE')
            goal_handle.succeed()
            return ClassifyGrasp.Result(success=bool(okc), bin_id=bin_id,
                                        message='dist=%.3f' % dist)
        except Exception as e:
            self.get_logger().error('执行异常: %s' % e)
            self._safe_stop()
            goal_handle.succeed()
            return ClassifyGrasp.Result(success=False, bin_id=bin_id, message='异常: %s' % e)


def main():
    rclpy.init()
    node = ClassifyGraspServer()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
