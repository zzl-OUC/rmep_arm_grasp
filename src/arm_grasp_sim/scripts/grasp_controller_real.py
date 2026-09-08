#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EP 机械臂抓取控制器（真机版, jeguzzi robomaster_ros 驱动）。
臂: move_arm action(笛卡尔 x-z 绝对航点, arm_base_link 系, x 向前 z 向上, 无 yaw);
    驱动同时只允许一个 goal(发新 goal 前旧 goal 必须已结束或 cancel), 硬超时 5s, 速度不可配置。
爪: gripper action(PAUSE/OPEN/CLOSE + power), 无开度反馈, 超时 7s。
反馈: arm_position topic(10Hz) 滞后数秒, 只用于监视/日志和 LIFT 后轮询确认;
    插值起点、恢复抬升等即时判断一律用 _last_cmd 指令值。
限速: 航点间按 max_step_m 插值子航点(每段 <= max_step_m)。
航点约定: (x, z) 绝对坐标(米)。真机无方块真值: 成功 = 所有航点 move_arm/gripper result 成功 + LIFT 抬升确认。
日志: 每个 goal 一个 CSV(航点序列 + 10Hz TCP 轨迹 + 每轮结果/错误), 目录 log_dir, 文件名带时间戳。"""
import csv
import math
import os
import time as _time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus

from arm_grasp_interfaces.action import GraspCycle
from robomaster_msgs.action import MoveArm, GripperControl
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

MOVE_TIMEOUT = 5.0     # move_arm 驱动侧硬超时(s)
GRIP_TIMEOUT = 7.0     # gripper 驱动侧超时(s)
SEND_TIMEOUT = 3.0     # goal 发送/接受等待(s)
HOME_X = 0.15          # HOME 安全位姿 x(回收位, 运行时夹在 x_range 内)

S_HOME = 'HOME'
S_APPROACH_A = 'APPROACH_A'
S_DESCEND_A = 'DESCEND_A'
S_CLAMP = 'CLAMP'
S_LIFT = 'LIFT'
S_TRANSPORT = 'TRANSPORT'
S_DESCEND_B = 'DESCEND_B'
S_RELEASE = 'RELEASE'
S_LIFT_B = 'LIFT_B'


class GraspControllerReal(Node):

    def __init__(self):
        super().__init__('grasp_controller_real')
        self.declare_parameter('A_x', 0.20)
        self.declare_parameter('B_x', 0.27)
        self.declare_parameter('z_grasp', 0.02)
        self.declare_parameter('z_safe', 0.10)
        self.declare_parameter('gripper_power', 0.5)
        self.declare_parameter('max_step_m', 0.03)
        self.declare_parameter('x_range', [0.05, 0.32])
        self.declare_parameter('z_range', [0.0, 0.20])
        self.declare_parameter('rm_ns', 'robomaster')
        self.declare_parameter('log_dir', '~/grasp_logs')
        self.A_x = float(self.get_parameter('A_x').value)
        self.B_x = float(self.get_parameter('B_x').value)
        self.z_grasp = float(self.get_parameter('z_grasp').value)
        self.z_safe = float(self.get_parameter('z_safe').value)
        self.gripper_power = float(self.get_parameter('gripper_power').value)
        self.max_step = float(self.get_parameter('max_step_m').value)
        self.x_range = list(self.get_parameter('x_range').value)
        self.z_range = list(self.get_parameter('z_range').value)
        self.rm_ns = str(self.get_parameter('rm_ns').value).strip('/')
        self.log_dir = os.path.expanduser(str(self.get_parameter('log_dir').value))
        self.home = (min(max(HOME_X, self.x_range[0]), self.x_range[1]), self.z_safe)

        self._cb = ReentrantCallbackGroup()
        self.state_pub = self.create_publisher(String, '/grasp_state', 10)
        self.tcp = None            # 最新 arm_position (Point)
        self.create_subscription(
            PointStamped, '/%s/arm_position' % self.rm_ns, self._tcp_cb, 10)
        self.move_client = ActionClient(
            self, MoveArm, '/%s/move_arm' % self.rm_ns, callback_group=self._cb)
        self.grip_client = ActionClient(
            self, GripperControl, '/%s/gripper' % self.rm_ns, callback_group=self._cb)
        self._move_gh = None       # 当前活动的 move_arm goal handle(驱动单 goal 限制)
        self._last_cmd = self.home  # 最后一条已完成的指令航点(插值起点, 替代滞后反馈)
        self.gripper_closed = False

        # CSV 日志状态
        self._csv = None
        self._csvw = None
        self._t0 = 0.0
        self._traj_t = 0.0
        self._state = S_HOME

        self._as = ActionServer(
            self, GraspCycle, 'grasp_cycle',
            execute_callback=self._execute, goal_callback=self._goal,
            cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=self._cb)
        self.get_logger().info('EP grasp_controller_real 就绪 (A_x=%.3f B_x=%.3f ns=/%s)'
                               % (self.A_x, self.B_x, self.rm_ns))

    # ---------- 基础 ----------
    def _tcp_cb(self, m):
        self.tcp = m.point

    def _tcp_xz(self):
        if self.tcp is None:
            return None
        return float(self.tcp.x), float(self.tcp.z)

    def _in_workspace(self, x, z):
        return self.x_range[0] <= x <= self.x_range[1] and self.z_range[0] <= z <= self.z_range[1]

    def _pub_state(self, state):
        self._state = state
        m = String(); m.data = state
        self.state_pub.publish(m)
        self.get_logger().info('[state] %s' % state)
        self._log_row('state', state, math.nan, math.nan, '')

    def _wait_fut(self, fut, timeout=2.0):
        # 不用 spin_until_future_complete: 它会把节点从执行器上拽下来, 导致订阅冻结
        t0 = _time.time()
        while rclpy.ok() and not fut.done() and _time.time() - t0 < timeout:
            self._sample_traj()
            _time.sleep(0.02)
        return fut.result() if fut.done() else None

    # ---------- CSV 日志 ----------
    def _open_csv(self):
        os.makedirs(self.log_dir, exist_ok=True)
        fn = os.path.join(self.log_dir, 'grasp_real_%s.csv' % _time.strftime('%Y%m%d_%H%M%S'))
        self._csv = open(fn, 'w', newline='')
        self._csvw = csv.writer(self._csv)
        self._csvw.writerow(['t', 'type', 'state', 'x', 'z', 'detail'])
        self._csv.flush()
        self._t0 = _time.time()
        self._traj_t = 0.0
        self.get_logger().info('日志: %s' % fn)

    def _close_csv(self):
        if self._csv is not None:
            self._csv.close()
        self._csv = None
        self._csvw = None

    def _log_row(self, typ, state, x, z, detail):
        if self._csvw is None:
            return
        self._csvw.writerow(['%.3f' % (_time.time() - self._t0), typ, state,
                             '%.4f' % x if x == x else '',
                             '%.4f' % z if z == z else '', detail])
        self._csv.flush()

    def _sample_traj(self, force=False):
        """10Hz TCP 轨迹采样(在 _wait_fut 轮询中调用)。"""
        if self._csvw is None or self.tcp is None:
            return
        now = _time.time()
        if not force and now - self._traj_t < 0.1:
            return
        self._traj_t = now
        self._log_row('traj', self._state, float(self.tcp.x), float(self.tcp.z), '')

    def _log_wp(self, tag, x, z, result, detail=''):
        self._log_row('wp', tag, x, z, '%s %s' % (result, detail) if detail else result)

    # ---------- 运动层: move_arm(单 goal 限制) ----------
    def _ensure_no_active_move(self):
        """发新 goal 前确保旧 goal 已结束, 否则先 cancel(驱动同时只允许一个 goal)。"""
        gh = self._move_gh
        if gh is not None:
            self.get_logger().warn('上一 move_arm goal 未结束, 先 cancel 再发新 goal')
            self._wait_fut(gh.cancel_goal_async(), 2.0)
            self._move_gh = None

    def _move_once(self, x, z, tag):
        """单个 move_arm 绝对航点 goal, 等 result/超时。返回 (ok, msg)。"""
        self._ensure_no_active_move()
        g = MoveArm.Goal()
        g.x = float(x)
        g.z = float(z)
        g.relative = False
        gh = self._wait_fut(self.move_client.send_goal_async(g), SEND_TIMEOUT)
        if gh is None:
            msg = 'move_arm goal 发送超时: %s(%.3f,%.3f)' % (tag, x, z)
            self._log_wp(tag, x, z, 'FAIL', msg)
            return False, msg
        if not gh.accepted:
            msg = 'move_arm goal 被拒: %s(%.3f,%.3f)' % (tag, x, z)
            self._log_wp(tag, x, z, 'FAIL', msg)
            return False, msg
        self._move_gh = gh
        res = self._wait_fut(gh.get_result_async(), MOVE_TIMEOUT + 1.0)
        self._move_gh = None
        if res is None:
            self._wait_fut(gh.cancel_goal_async(), 2.0)
            msg = 'move_arm 超时(>%.1fs): %s(%.3f,%.3f)' % (MOVE_TIMEOUT, tag, x, z)
            self._log_wp(tag, x, z, 'FAIL', msg)
            return False, msg
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            msg = 'move_arm 失败(status=%d): %s(%.3f,%.3f)' % (res.status, tag, x, z)
            self._log_wp(tag, x, z, 'FAIL', msg)
            return False, msg
        self._log_wp(tag, x, z, 'OK')
        return True, 'ok'

    def _goto(self, tag, x, z):
        """工作空间校验 + 按 max_step_m 插值子航点 + 逐段 move_arm。返回 (ok, msg)。
        插值起点永远用 _last_cmd(上一指令航点): arm_position 反馈滞后数秒,
        拿它当起点会把臂往回拽再爬回来(实测抽动), 且路径不再确定。"""
        if not self._in_workspace(x, z):
            msg = '目标超出工作空间: %s(%.3f,%.3f) x_range=%s z_range=%s' \
                  % (tag, x, z, self.x_range, self.z_range)
            self.get_logger().error(msg)
            self._log_wp(tag, x, z, 'FAIL', msg)
            return False, msg
        cur = self._last_cmd
        dist = math.hypot(x - cur[0], z - cur[1])
        n = max(1, int(math.ceil(dist / self.max_step)))
        if n > 1:
            self.get_logger().info('[move] %s(%.3f,%.3f) 距离 %.3fm -> %d 段子航点'
                                   % (tag, x, z, dist, n))
        for i in range(n):
            px = cur[0] + (x - cur[0]) * (i + 1) / n
            pz = cur[1] + (z - cur[1]) * (i + 1) / n
            sub = tag if n == 1 else '%s_%d/%d' % (tag, i + 1, n)
            ok, msg = self._move_once(px, pz, sub)
            if not ok:
                return False, msg
        self._last_cmd = (x, z)
        return True, 'ok'

    # ---------- 爪: gripper ----------
    def _gripper(self, target, tag):
        """gripper action goal, 等 result/超时。返回 (ok, msg)。"""
        g = GripperControl.Goal()
        g.target_state = target
        g.power = float(self.gripper_power)
        gh = self._wait_fut(self.grip_client.send_goal_async(g), SEND_TIMEOUT)
        if gh is None:
            msg = 'gripper goal 发送超时: %s' % tag
            self._log_wp(tag, math.nan, math.nan, 'FAIL', msg)
            return False, msg
        if not gh.accepted:
            msg = 'gripper goal 被拒: %s' % tag
            self._log_wp(tag, math.nan, math.nan, 'FAIL', msg)
            return False, msg
        res = self._wait_fut(gh.get_result_async(), GRIP_TIMEOUT + 1.0)
        if res is None:
            self._wait_fut(gh.cancel_goal_async(), 2.0)
            msg = 'gripper 超时(>%.1fs): %s' % (GRIP_TIMEOUT, tag)
            self._log_wp(tag, math.nan, math.nan, 'FAIL', msg)
            return False, msg
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            msg = 'gripper 失败(status=%d): %s' % (res.status, tag)
            self._log_wp(tag, math.nan, math.nan, 'FAIL', msg)
            return False, msg
        self.gripper_closed = (target == GripperControl.Goal.CLOSE)
        d = res.result.duration
        self.get_logger().info('[gripper] %s OK duration=%d.%03ds'
                               % (tag, d.sec, int(d.nanosec / 1e6)))
        self._log_wp(tag, math.nan, math.nan, 'OK', 'duration=%.2fs' % (d.sec + d.nanosec * 1e-9))
        return True, 'ok'

    def _confirm_lift(self, timeout=10.0):
        """LIFT 确认: 轮询等 arm_position 追上来(反馈滞后数秒, 单点读必误判),
        每 0.2s 读一次, z 升到确认阈以上算抬起; 超时仍低于阈才判 FAIL(真没夹住)。"""
        th = 0.5 * (self.z_grasp + self.z_safe)
        t0 = _time.time()
        p = self._tcp_xz()
        while rclpy.ok() and _time.time() - t0 < timeout:
            p = self._tcp_xz()
            if p is not None and p[1] >= th:
                self.get_logger().info('LIFT 确认: z=%.3f >= %.3f (等 %.1fs)'
                                       % (p[1], th, _time.time() - t0))
                return True, 'ok'
            self._sample_traj()
            _time.sleep(0.2)
        if p is None:
            return False, '无 arm_position 反馈, 无法确认末端抬起'
        return False, '末端未抬起: 等 %.1fs 后 z=%.3f < 确认阈 %.3f' % (timeout, p[1], th)

    def _lift_with_confirm(self):
        ok, msg = self._goto(S_LIFT, self.A_x, self.z_safe)
        if not ok:
            return ok, msg
        return self._confirm_lift()

    def _cycle_waypoints(self):
        """单轮全部笛卡尔航点 [(tag, x, z), ...](爪动作不含), 用于执行前统一校验。"""
        return [
            (S_HOME, self.home[0], self.home[1]),
            (S_APPROACH_A, self.A_x, self.z_safe),
            (S_DESCEND_A, self.A_x, self.z_grasp),
            (S_LIFT, self.A_x, self.z_safe),
            (S_TRANSPORT, self.B_x, self.z_safe),
            (S_DESCEND_B, self.B_x, self.z_grasp),
            (S_LIFT_B, self.B_x, self.z_safe),
        ]

    def _recover_home(self):
        """失败后的安全回收: 尽力先抬到 z_safe 再回 HOME, 夹着东西则回位后张爪。
        抬升判断用 _last_cmd(指令值), 不用滞后的 arm_position 反馈。不抛异常。"""
        try:
            cur = self._last_cmd
            if cur[1] < self.z_safe and self._in_workspace(cur[0], self.z_safe):
                self._goto('RECOVER_LIFT', cur[0], self.z_safe)
            self._goto(S_HOME, self.home[0], self.home[1])
            if self.gripper_closed:
                self.get_logger().info('回 HOME 后张爪(避免夹着物体回位)')
                self._gripper(GripperControl.Goal.OPEN, 'RECOVER_OPEN')
        except Exception as e:
            self.get_logger().error('回收 HOME 失败: %s' % e)

    # ---------- Action ----------
    def _goal(self, goal):
        return GoalResponse.ACCEPT

    async def _execute(self, goal_handle):
        goal = goal_handle.request
        cycles = max(1, int(goal.cycles))
        success_count = 0
        total_count = 0
        fail_reason = ''
        OPEN, CLOSE = GripperControl.Goal.OPEN, GripperControl.Goal.CLOSE

        # 目标点先校验: 任一航点超出工作空间则不发任何运动指令
        self._open_csv()
        bad = ['%s(%.3f,%.3f)' % (t, x, z) for t, x, z in self._cycle_waypoints()
               if not self._in_workspace(x, z)]
        if bad:
            msg = '目标超出工作空间, 拒绝执行: %s (x_range=%s z_range=%s)' \
                  % (', '.join(bad), self.x_range, self.z_range)
            self.get_logger().error(msg)
            self._log_row('error', 'SAFETY', math.nan, math.nan, msg)
            self._close_csv()
            self._recover_home()
            goal_handle.abort()
            return GraspCycle.Result(
                success=False, success_count=0, total_count=0, message=msg)

        # 驱动 action server 可用性检查
        for cli, name in ((self.move_client, 'move_arm'), (self.grip_client, 'gripper')):
            t0 = _time.time()
            while rclpy.ok() and not cli.server_is_ready() and _time.time() - t0 < 5.0:
                _time.sleep(0.1)
            if not cli.server_is_ready():
                msg = '驱动 action server 不可用: /%s/%s' % (self.rm_ns, name)
                self.get_logger().error(msg)
                self._log_row('error', 'DRIVER', math.nan, math.nan, msg)
                self._close_csv()
                goal_handle.abort()
                return GraspCycle.Result(
                    success=False, success_count=0, total_count=0, message=msg)

        try:
            for c in range(cycles):
                if goal_handle.is_cancel_requested:
                    fail_reason = 'goal 被客户端取消'
                    self.get_logger().warn(fail_reason)
                    self._log_row('error', 'CANCEL', math.nan, math.nan, fail_reason)
                    self._recover_home()
                    goal_handle.canceled()
                    return GraspCycle.Result(
                        success=False, success_count=success_count,
                        total_count=total_count, message=fail_reason)
                fb = GraspCycle.Feedback()
                fb.current_state = 'CYCLE_%d' % (c + 1)
                fb.progress = float(c) / float(cycles)
                fb.gripper_closed = self.gripper_closed
                goal_handle.publish_feedback(fb)

                # 单轮状态机(与仿真版同骨架): HOME -> APPROACH_A -> DESCEND_A ->
                # CLAMP -> LIFT(确认) -> TRANSPORT -> DESCEND_B -> RELEASE -> LIFT_B -> HOME
                steps = [
                    (S_HOME, lambda: self._gripper(OPEN, 'OPEN')),
                    (S_HOME, lambda: self._goto(S_HOME, *self.home)),
                    (S_APPROACH_A, lambda: self._goto(S_APPROACH_A, self.A_x, self.z_safe)),
                    (S_DESCEND_A, lambda: self._goto(S_DESCEND_A, self.A_x, self.z_grasp)),
                    (S_CLAMP, lambda: self._gripper(CLOSE, 'CLOSE')),
                    (S_LIFT, self._lift_with_confirm),
                    (S_TRANSPORT, lambda: self._goto(S_TRANSPORT, self.B_x, self.z_safe)),
                    (S_DESCEND_B, lambda: self._goto(S_DESCEND_B, self.B_x, self.z_grasp)),
                    (S_RELEASE, lambda: self._gripper(OPEN, 'OPEN')),
                    (S_LIFT_B, lambda: self._goto(S_LIFT_B, self.B_x, self.z_safe)),
                    (S_HOME, lambda: self._goto(S_HOME, *self.home)),
                ]
                cycle_fail = None
                for st, fn in steps:
                    self._pub_state(st)
                    ok, msg = fn()
                    if not ok:
                        cycle_fail = '%s: %s' % (st, msg)
                        break

                # 判定: 真机无方块真值, 一轮成功 = 所有航点/爪动作 result 成功 + LIFT 已确认
                total_count += 1
                if cycle_fail is None:
                    success_count += 1
                    self.get_logger().info('[CYCLE %d] 成功(所有航点完成)' % (c + 1))
                    self._log_row('cycle', 'CYCLE_%d' % (c + 1), math.nan, math.nan, 'SUCCESS')
                    self._pub_state('CYCLE_%d_OK' % (c + 1))
                else:
                    self.get_logger().error('[CYCLE %d] 失败: %s' % (c + 1, cycle_fail))
                    self._log_row('cycle', 'CYCLE_%d' % (c + 1), math.nan, math.nan,
                                  'FAIL: %s' % cycle_fail)
                    self._pub_state('CYCLE_%d_FAIL' % (c + 1))
                    fail_reason = cycle_fail
                    self._recover_home()
                    break
        except Exception as e:
            self.get_logger().error('执行异常: %s' % e)
            fail_reason = '执行异常: %s' % e
            self._log_row('error', 'EXCEPTION', math.nan, math.nan, fail_reason)
            self._recover_home()
        finally:
            self._close_csv()
        self._pub_state('DONE')
        if fail_reason:
            goal_handle.abort()
            return GraspCycle.Result(
                success=False, success_count=success_count,
                total_count=total_count, message=fail_reason)
        goal_handle.succeed()
        return GraspCycle.Result(
            success=success_count >= cycles,
            success_count=success_count,
            total_count=total_count,
            message='%d/%d 成功' % (success_count, total_count))


def main():
    rclpy.init()
    node = GraspControllerReal()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
