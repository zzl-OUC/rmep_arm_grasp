#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EP 机械臂抓取控制器（官方 jeguzzi 模型移植版）。
臂: chassis_yaw + arm_1 + arm_2 速度接口(100Hz 速度跟踪), 括架 = -(arm_1+arm_2) 软件补偿。
爪: 平行直进双指夹爪(effort 接口, N), 官方EP规格(开合~10cm/夹力5N), 双指对称平移非旋转; 抓取保持由 gazebo_grasp_fix 插件兜底。
IK: 官方尺寸解析 FK + 带界数值反解(2 自由度), 不可达直接报错(验收四.4)。
航点约定: (yaw, d, z) —— d 为 TCP 相对底盘中心的水平距离。
注意: 官方 TCP(gripper_t) 比指尖超前 TCP_LEAD=0.053m, 抓取 d = 方块距离 + TCP_LEAD。"""
import math
import time as _time
import numpy as np
from scipy.optimize import least_squares

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from arm_grasp_interfaces.action import GraspCycle
from gazebo_msgs.srv import GetEntityState, SetEntityState
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState

ARM_JOINTS = ['chassis_yaw_joint', 'arm_1_joint', 'arm_2_joint', 'endpoint_bracket_joint']
GRIP_JOINTS = ['gripper_left_joint', 'gripper_right_joint']

# ---- 官方几何常数（x-z 平面, 括架保持水平） ----
TABLE_TOP = 0.025
ARM1_X = 0.0103961
ARM1_Z = 0.03465 + 0.0906477 + 0.030741          # base_link -> arm_1 关节
A1X, A1Z = 0.0018704, 0.1210238                   # arm_1 -> arm_2
A2X, A2Z = 0.1058557, -0.0561093                  # arm_2 -> 括架
TCPX, TCPZ = 0.124 + 0.0002793, -0.039 + 0.0001815  # 括架 -> gripper_t
TCP_LEAD = 0.066                                   # TCP 相对指尖的超前量(实测标定项)
Q1_LIM = (-0.274, 1.384)
Q2_LIM = (-1.25, 0.40)

# ---- 爪(平行直进, effort 接口, 单位 N, 正=闭合/负=张开) ----
GRIP_OPEN = (-5.0, -5.0)
GRIP_SEAT = (5.0, 5.0)
GRIP_CLOSE = (15.0, 15.0)

# ---- 关节空间抓取姿态(probe 实测标定, 对应 pad 罩住 4cm 方块) ----
Q_GRASP = (1.384, -0.4714)   # (q1, q2), q1 贴上限
Q_LIFT_Q1 = 1.20             # 抬升: 降 q1 实现近垂直抬升
Q_RAISED = (0.9, -0.3)       # 高位过渡姿态

MOVE_DUR = 3.0
LIFT_DUR = 3.0
YAW_DUR = 6.0
GRIP_SEAT_DUR = 3.0
VC = 0.9            # 关节速度限幅 rad/s
KV = 4.0            # 速度跟踪增益

S_HOME = 'HOME'
S_APPROACH_A = 'APPROACH_A'
S_DESCEND_A = 'DESCEND_A'
S_SETTLE = 'SETTLE'
S_GRASP_SEAT = 'GRASP_SEAT'
S_CLAMP = 'CLAMP'
S_LIFT = 'LIFT'
S_TRANSPORT = 'TRANSPORT'
S_DESCEND_B = 'DESCEND_B'
S_RELEASE = 'RELEASE'
S_LIFT_B = 'LIFT_B'


def fk_dz(q1, q2):
    """官方尺寸 FK: (q1, q2) -> (d, z), d 为 TCP 相对底盘中心的水平距离。"""
    th1, th2 = q1, q1 + q2
    d = ARM1_X + A1X * math.cos(th1) + A1Z * math.sin(th1) \
        + A2X * math.cos(th2) + A2Z * math.sin(th2) + TCPX
    z = TABLE_TOP + ARM1_Z - A1X * math.sin(th1) + A1Z * math.cos(th1) \
        - A2X * math.sin(th2) + A2Z * math.cos(th2) + TCPZ
    return d, z


def ik_dz(d, z, seed=(1.0, 0.0)):
    """带界数值反解。返回 (q1, q2, ok, msg)。"""
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


class GraspController(Node):

    def __init__(self):
        super().__init__('grasp_controller')
        self.declare_parameter('A_xy', [0.20, 0.0])
        self.declare_parameter('B_xy', [0.20, 0.0])
        self.declare_parameter('z_approach', 0.13)
        self.declare_parameter('z_grasp', 0.0757)
        self.declare_parameter('z_lift', 0.095)
        self.declare_parameter('block_name', 'block')
        p = self.get_parameters_by_prefix
        self.A = tuple(self.get_parameter('A_xy').value)
        self.B = tuple(self.get_parameter('B_xy').value)
        self.z_app = float(self.get_parameter('z_approach').value)
        self.z_grasp = float(self.get_parameter('z_grasp').value)
        self.z_lift = float(self.get_parameter('z_lift').value)
        self.block_name = self.get_parameter('block_name').value
        self.block_z0 = TABLE_TOP + 0.020

        self._cb = ReentrantCallbackGroup()
        self.state_pub = self.create_publisher(String, '/grasp_state', 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/arm_vel_controller/commands', 10)
        self.brk_pub = self.create_publisher(Float64MultiArray, '/bracket_controller/commands', 10)
        self.grip_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.js = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 50)
        self.get_entity = self.create_client(GetEntityState, '/gazebo/get_entity_state', callback_group=self._cb)
        self.set_entity = self.create_client(SetEntityState, '/gazebo/set_entity_state', callback_group=self._cb)

        self._as = ActionServer(
            self, GraspCycle, 'grasp_cycle',
            execute_callback=self._execute, goal_callback=self._goal,
            cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=self._cb)
        self.get_logger().info('EP grasp_controller 就绪 (A=%s B=%s)' % (self.A, self.B))

    # ---------- 基础 ----------
    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))

    def _arm_q(self):
        try:
            return np.array([float(self.js[n]) for n in ARM_JOINTS])
        except Exception:
            return np.zeros(4)

    def _pub_vel(self, v3):
        m = Float64MultiArray(); m.data = [float(x) for x in v3]
        self.vel_pub.publish(m)

    def _pub_bracket(self, q):
        # 括架位置指令(并联机构等效): 始终 -(arm_1+arm_2), 使爪保持水平
        m = Float64MultiArray(); m.data = [float(q)]
        self.brk_pub.publish(m)

    def _pub_grip(self, l, r):
        m = Float64MultiArray(); m.data = [float(l), float(r)]
        self.grip_pub.publish(m)
        self.get_logger().info('[gripper] L=%+.2f R=%+.2f rad' % (l, r))

    def _pub_state(self, state):
        m = String(); m.data = state
        self.state_pub.publish(m)
        self.get_logger().info('[state] %s' % state)

    def _wait_fut(self, fut, timeout=2.0):
        # 不用 spin_until_future_complete: 它会把节点从执行器上拽下来, 导致订阅冻结
        t0 = _time.time()
        while rclpy.ok() and not fut.done() and _time.time() - t0 < timeout:
            _time.sleep(0.02)
        return fut.result() if fut.done() else None

    def _block_state(self):
        if not self.get_entity.service_is_ready():
            return None
        req = GetEntityState.Request()
        req.name = self.block_name
        return self._wait_fut(self.get_entity.call_async(req))

    def _block_xyz(self):
        s = self._block_state()
        if s is None or not s.success:
            return None
        p = s.state.pose.position
        return p.x, p.y, p.z

    def _reset_block(self):
        """每轮开始把方块放回 A 点（模拟人工摆放）。"""
        for _ in range(10):
            if not self.set_entity.service_is_ready():
                _time.sleep(0.5)
                continue
            req = SetEntityState.Request()
            req.state.name = self.block_name
            req.state.pose.position.x = float(self.A[0])
            req.state.pose.position.y = float(self.A[1])
            req.state.pose.position.z = float(self.block_z0)
            if self._wait_fut(self.set_entity.call_async(req)) is not None:
                return True
        return False

    # ---------- 运动层: 100Hz 速度跟踪 ----------
    async def _send_arm(self, yaw, q1, q2, duration):
        """速度跟踪到 (yaw, q1, q2); 括架每拍瞬时补偿 -(q1n+q2n)。"""
        steps = max(1, int(float(duration) / 0.01))
        settled = 0
        for _ in range(steps):
            qn = self._arm_q()
            tgt = np.array([yaw, q1, q2])
            err = tgt - qn[:3]
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
        ok = np.max(np.abs(np.array([yaw, q1, q2]) - qn[:3])) < 0.03
        return ok, 'yaw=%.3f q1=%.3f q2=%.3f' % tuple(qn[:3])

    async def _goto_tcp(self, yaw, d, z, dur, seed):
        """IK 反解 + 速度跟踪到位。返回 (ok, msg, 新seed)。"""
        q1, q2, ok, msg = ik_dz(d, z, seed)
        if not ok:
            return False, msg, seed
        ok2, msg2 = await self._send_arm(yaw, q1, q2, dur)
        return ok2, msg2, (q1, q2)

    def _hold(self, grip_eff, dur, tag=''):
        """保持当前臂姿(位置锁定), 只施加爪力矩。"""
        q0 = self._arm_q()[:3].copy()
        steps = max(1, int(float(dur) / 0.05))
        for _ in range(steps):
            qn = self._arm_q()
            self._pub_vel(np.clip(KV * (q0 - qn[:3]), -VC, VC))
            self._pub_bracket(-(qn[1] + qn[2]))
            self._pub_grip(*grip_eff)
            _time.sleep(0.05)
        self._pub_vel(np.zeros(3))

    # ---------- Action ----------
    def _goal(self, goal):
        return rclpy.action.GoalResponse.ACCEPT

    async def _execute(self, goal_handle):
        goal = goal_handle.request
        cycles = max(1, int(goal.cycles))
        success_count = 0
        total_count = 0
        seed = (1.05, 0.0)
        dA = math.hypot(self.A[0], self.A[1])
        dB = math.hypot(self.B[0], self.B[1])
        yawA = math.atan2(self.A[1], self.A[0])
        yawB = math.atan2(self.B[1], self.B[0])
        dG = dA + TCP_LEAD
        self.get_logger().info('抓取 d=%.3f (方块 d=%.3f + TCP_LEAD), z_grasp=%.3f' % (dG, dA, self.z_grasp))
        try:
            for c in range(cycles):
                fb = GraspCycle.Feedback()
                fb.current_state = 'CYCLE_%d' % (c + 1)
                fb.progress = float(c) / float(cycles)
                fb.gripper_closed = False
                goal_handle.publish_feedback(fb)
                self._pub_state(S_HOME)
                self._reset_block()
                _time.sleep(0.5)
                b0 = self._block_xyz()
                self.get_logger().info('方块复位 A: %s' % (b0,))
                self._pub_grip(*GRIP_OPEN)
                # 启动停顿期手臂可能瘫在桌面上, 先大幅抬臂使指尖脱桌, 再进入循环
                ok, msg = await self._send_arm(0.0, 1.30, 0.35, 10.0)
                if not ok:
                    self.get_logger().warn("HOME 抬臂未到位(继续): %s" % msg)

                # 接近 + 下降 + 抓取
                self._pub_state(S_APPROACH_A)
                ok, msg = await self._send_arm(yawA, Q_RAISED[0], Q_RAISED[1], MOVE_DUR)
                if not ok:
                    self.get_logger().warn("move FAIL: %s" % msg)
                    break
                b = self._block_xyz(); self.get_logger().info('APPROACH 方块: %s' % (b,))
                self._pub_state(S_DESCEND_A)
                ok, msg = await self._send_arm(yawA, Q_GRASP[0], Q_GRASP[1], MOVE_DUR)
                if not ok:
                    self.get_logger().warn("move FAIL: %s" % msg)
                    break
                self._pub_state(S_SETTLE)
                self._hold(GRIP_OPEN, 1.5)
                b = self._block_xyz(); self.get_logger().info('DESCEND 方块: %s' % (b,))
                self._pub_state(S_GRASP_SEAT)
                self._hold(GRIP_SEAT, GRIP_SEAT_DUR)
                self._pub_state(S_CLAMP)
                self._hold(GRIP_CLOSE, 1.5)
                b = self._block_xyz(); self.get_logger().info('CLAMP 方块: %s' % (b,))

                # 抬升 + 搬运
                self._pub_state(S_LIFT)
                ok, msg = await self._send_arm(yawA, Q_LIFT_Q1, Q_GRASP[1], LIFT_DUR)
                if not ok:
                    self.get_logger().warn("move FAIL: %s" % msg)
                    break
                b = self._block_xyz(); self.get_logger().info('LIFT 方块: %s' % (b,))
                self._pub_state(S_TRANSPORT)
                ok, msg = await self._send_arm(yawB, Q_LIFT_Q1, Q_GRASP[1], YAW_DUR)
                if not ok:
                    self.get_logger().warn("move FAIL: %s" % msg)
                    break
                b = self._block_xyz(); self.get_logger().info('TRANSPORT 方块: %s' % (b,))
                self._pub_state(S_DESCEND_B)
                ok, msg = await self._send_arm(yawB, Q_GRASP[0], Q_GRASP[1], LIFT_DUR)
                if not ok:
                    self.get_logger().warn("move FAIL: %s" % msg)
                    break
                self._pub_state(S_SETTLE)
                self._hold(GRIP_CLOSE, 1.0)
                self._pub_state(S_RELEASE)
                self._hold(GRIP_OPEN, 2.0)
                self._pub_state(S_LIFT_B)
                ok, msg = await self._send_arm(yawB, Q_RAISED[0], Q_RAISED[1], 2.5)
                if not ok:
                    self.get_logger().warn("move FAIL: %s" % msg)
                    break

                # 判定
                total_count += 1
                b = self._block_xyz()
                if b is not None:
                    dist = math.hypot(b[0] - self.B[0], b[1] - self.B[1])
                    okc = dist < 0.06 and abs(b[2] - self.block_z0) < 0.02
                else:
                    dist = -1.0
                    okc = False
                if okc:
                    success_count += 1
                self.get_logger().info('[CYCLE %d] 方块=(%.4f,%.4f,%.4f) 距B=%.3f 成功=%s'
                                       % (c + 1, b[0], b[1], b[2], dist, okc))
                self._pub_state('CYCLE_%d_%s' % (c + 1, 'OK' if okc else 'FAIL'))
                # 回零
                self._pub_grip(*GRIP_OPEN)
                await self._send_arm(0.0, 1.30, 0.35, 8.0)
        except Exception as e:
            self.get_logger().error('执行异常: %s' % e)
            self._safe_stop()
        self._pub_state('DONE')
        goal_handle.succeed()
        return GraspCycle.Result(
            success=success_count >= 4,
            success_count=success_count,
            total_count=total_count,
            message='%d/%d 成功' % (success_count, total_count))

    def _safe_stop(self):
        """异常时安全停止: 张爪 + 速度清零。"""
        self._pub_grip(*GRIP_OPEN)
        self._pub_vel(np.zeros(3))


def main():
    rclpy.init()
    node = GraspController()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
