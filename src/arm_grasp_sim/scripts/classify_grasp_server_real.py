#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验三 真机版分类抓取 server（ClassifyGrasp action, 与仿真版同契约）。

与仿真版 classify_grasp_server.py 的差别
----------------------------------------
仿真: (yaw, d, z) 关节空间 + IK + Gazebo 真值判定 + 速度接口。
真机: 用 jeguzzi robomaster_ros 驱动
  - 臂  : move_arm action（笛卡尔 x-z 绝对航点, arm_base_link 系, x 向前 z 向上;
          单 goal 限制, 硬超时 5s, 速度不可配）→ 航点间按 max_step_m 插值限速。
  - 爪  : gripper action（OPEN/CLOSE + power, 无开度反馈）。
  - 底盘: move action（相对 theta 旋转）→ 把目标网格/料盒转到臂正前方。
          仿真里靠 chassis_yaw_joint 转 yaw；真机臂没有 yaw, 只能转底盘对位。

对位: 视觉闭环（align_mode=closed, 默认）
----------------------------------------
旧实现是**开环**: 用静态 grid_azimuth 转一次, 然后假设"转完目标就在臂正前方
(x_grasp, z_grasp)", 且 `heading += 指令值` —— 把"我发了多少"当成"我转了多少"。
麦轮打滑/惯性/旋转中心≠臂基座 会让实际转角与位置都偏离, 误差还逐轮累积。

现在: 转 → 用 /grid_detections 里实测的物体坐标 (x, y) 算横向残差 → 再转, 迭代到
|y| ≤ align_lateral_tol 才下降; 抓取 x 也用实测值(钳进工作空间)。
要点: 闭环只要求"照相机看得见目标", 不要求"底盘转得准" —— 打滑被观测吸收,
不需要建模补偿。`heading` 则由 /<ns>/odom 的实测 yaw 推进(打滑量写进日志可查)。
前提: 相机随车转（故"像素→臂基座系"的映射与底盘位姿无关）。

上机必须标定（见 config/classify_real.yaml）
  1. grid_azimuth / bin_azimuth：各网格/料盒的方位角（世界系, 相对机器人初始朝向）。
  2. x_grasp/z_grasp/x_drop/z_drop：固定抓取/投放点位。目标必须落在臂可达范围内
     （EP 实测 x∈[0.05,0.24], >0.20 饱和）→ 网格环半径要 ≤~0.20m, 否则靠底盘前移补偿。
  3. 相机：grid_mapper 的 hfov / cam_height, 或替换为完整内外参标定。

成功判定: 真机无方块真值, 用两路证据:
  a) 所有 move_arm / gripper result 成功 + LIFT 后 arm_position 确认末端抬起（必选）;
  b) confirm_via_detect: 抓取前该网格在检测中、抓取后消失 → 强确认（可选, 注意遮挡）。
"""
import csv
import json
import math
import os
import time as _time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus

from arm_grasp_interfaces.action import ClassifyGrasp
from robomaster_msgs.action import MoveArm, GripperControl, Move
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

MOVE_TIMEOUT = 5.0        # move_arm 驱动侧硬超时(s)
GRIP_TIMEOUT = 7.0        # gripper 驱动侧超时(s)
SEND_TIMEOUT = 3.0        # goal 发送/接受等待(s)
HEADING_DEADBAND = 0.03   # rad, 小于此不转底盘

# ---- 失败分类码(与 ClassifyGrasp.action 的 error_code 字段 / 仿真版一致) ----
EC_NONE = 0
EC_UNREACHABLE = 1     # 超出工作空间/不可达(上机为安全闸门拒绝)
EC_GRASP_MISSED = 2    # 夹空
EC_MOTION_FAILED = 3   # 航点/爪动作失败
EC_UNKNOWN_TARGET = 4  # 非法网格/料盒
EC_INTERNAL = 5        # 驱动不可用 / 内部异常


def _err_code_of(msg):
    """失败信息 -> 分类码(真机无 IK, 夹空靠检测确认 + 抬升确认)。"""
    if '夹空' in str(msg):
        return EC_GRASP_MISSED
    return EC_MOTION_FAILED


def _wrap(a):
    """把角度归一到 [-pi, pi]。"""
    return (a + math.pi) % (2 * math.pi) - math.pi


def vloop_correction(x_o, y_o, tol, min_step, base_offset=0.0):
    """纯函数: 由"实测物体在臂基座系的坐标"算出下一轮底盘该转多少。

    几何: 设 ρ = 物体到**旋转中心**的距离, Δ = 当前朝向残差, r_b = **臂基座到旋转
    中心的距离**(base_offset), 则物体在臂基座系里是
        y = ρ·sinΔ            x = ρ·cosΔ − r_b
    反解得
        Δ = atan2(y, x + r_b)
    ⚠ 分母必须带 r_b。若当 0 处理(把 atan2(y, x) 当答案), 算出的转角会**偏大**,
      闭环于是过冲、来回振荡 —— 实测轨迹(ρ=0.2, r_b=0.10, 滑差10%):
      173mm → -74mm → +49mm → -37mm, 3 轮远不收敛。带上 r_b 后 1-2 轮即收。

    返回 (dtheta, reason):
      dtheta = None 表示本轮不转 —— reason:
        'converged'  : |y| 已在 tol 内**且物体在正前方**(横向对齐不等于能抓;
                       物体在正后方时 y 也≈0, 那时必须转 π 掉头)
        'degenerate' : 物体与臂基座几乎重合, 方位角病态
        'too_small'  : 残差还在但转角小于 min_step, 再转也不管用(避免空转)
      dtheta = float 表示相对转这么多(rad)。

    单独抽成纯函数, 是为了能离线验证"纠正数学"本身 —— 不必起 ROS 或真机。
    """
    if abs(x_o) < 1e-6 and abs(y_o) < 1e-6:
        return None, 'degenerate'
    d = _wrap(math.atan2(y_o, x_o + base_offset))
    if abs(y_o) <= tol and (x_o + base_offset) > 0.0:
        return None, 'converged'
    if abs(d) < min_step:
        return None, 'too_small'
    return d, 'rotate'


def estimate_base_offset(x1, y1, x2, y2, psi):
    """由"转前/转后各一次实测 + 实测转角"在线估出臂基座偏置 r_b。

    只需横向分量即可解。设 s = 转前朝向残差, ψ = 实际转角, ρ = 物体到旋转中心距离:
        y1 = ρ·sin(s)            y2 = ρ·sin(s−ψ)
    消去 ρ:
        tan(s) = y1·sinψ / (y1·cosψ − y2)
    于是
        s   = atan2(y1·sinψ, y1·cosψ − y2),  ρ = y1 / sin(s)
        r_b = ρ·cos(s) − x1                    (由 x1 = ρ·cos s − r_b)
    返回 r_b; sin(s)≈0(物体几乎在正前/正后, ρ 不可解) 时返回 None。

    为什么要有它: 纠正量 atan2(y, x+r_b) 的分母含 r_b, 而 r_b 是"臂基座到旋转中心"
    的距离 —— 藏在车体里、拿尺子量不准、还随云台/臂姿态变。能在线估出来最省事。
    """
    sp, cp = math.sin(psi), math.cos(psi)
    s = math.atan2(y1 * sp, y1 * cp - y2)
    sn = math.sin(s)
    if abs(sn) < 0.2:            # sin s 太小 -> ρ 估计不稳, 放弃本次
        return None
    rho = y1 / sn
    if rho <= 0 or rho > 5.0:    # 明显不合理的距离
        return None
    return rho * math.cos(s) - x1


class ClassifyGraspServerReal(Node):

    def __init__(self):
        super().__init__('classify_grasp_server_real')
        self.declare_parameter('rm_ns', 'robomaster')
        self.declare_parameter('x_grasp', 0.20)
        self.declare_parameter('z_grasp', 0.02)
        self.declare_parameter('x_drop', 0.20)
        self.declare_parameter('z_drop', 0.02)
        self.declare_parameter('z_safe', 0.10)
        self.declare_parameter('gripper_power', 0.5)
        self.declare_parameter('max_step_m', 0.03)
        self.declare_parameter('x_range', [0.05, 0.24])
        self.declare_parameter('z_range', [0.0, 0.20])
        self.declare_parameter('chassis_angular_speed', 0.5)
        self.declare_parameter('chassis_linear_speed', 0.3)
        self.declare_parameter('chassis_timeout', 15.0)
        self.declare_parameter('use_chassis', True)
        # 网格/料盒方位角(rad, 世界系, 相对机器人初始朝向) —— 上机实测替换
        self.declare_parameter(
            'grid_azimuth',
            '{"cell_1": 0.349, "cell_2": 0.838, "cell_3": 1.326, '
            '"cell_4": 1.815, "cell_5": 2.304, "cell_6": 2.793}')
        self.declare_parameter('bin_azimuth', '{"bin_0": -2.620, "bin_1": -1.920}')
        self.declare_parameter('confirm_via_detect', True)
        self.declare_parameter('miss_if_still_present', False)
        # ── 视觉闭环对位(前提: 相机随车转, 故"像素->臂基座系"映射与底盘位姿无关)──
        # open  = 旧行为: 用静态 grid_azimuth 转一次, 假设转完目标就在臂正前方
        # closed= 转完重测目标实际位置, 迭代到横向残差 ≤ tol 才下降
        # 闭环的意义: 底盘打滑/惯性/旋转中心≠臂基座 这些误差被"下一轮观测"吸收,
        # 不需要建模补偿 —— 它们被移出回路, 而不是被估计。
        self.declare_parameter('align_mode', 'closed')
        self.declare_parameter('align_max_iter', 8)
        self.declare_parameter('align_lateral_tol', 0.008)   # m, 横向收敛判据
        self.declare_parameter('align_settle', 0.6)          # s, 转完等稳定再测
        self.declare_parameter('align_det_timeout', 2.5)     # s, 等一条新检测
        self.declare_parameter('align_min_step', 0.03)       # rad, 最小有效转角(≥底盘死区)
        # 阻尼系数: 实际只转 gain×Δ。gain=1 在"偏置未知"时会过冲振荡, 0.8 才稳
        # (参数扫描: gain=1 时未知偏置场景 2.3~8.3% 超差; 0.8 时 0.0%)
        self.declare_parameter('align_gain', 0.8)
        self.declare_parameter('base_offset', 0.0)           # m, 臂基座到旋转中心(初值)
        self.declare_parameter('base_offset_auto', True)     # 在线自估该偏置(免拿尺子量)
        self.declare_parameter('base_offset_max', 0.35)      # m, 自估的合理上限(挡坏数据)
        self.declare_parameter('base_offset_alpha', 0.5)     # 自估的指数平滑系数
        self.declare_parameter('grasp_x_from_vision', True)  # 用实测 x 抓(替掉固定 x_grasp)
        self.declare_parameter('heading_from_odom', True)    # 用 odom 实测转角推进 heading
        self.declare_parameter('odom_topic', '')             # 空=自动 /<rm_ns>/odom
        self.declare_parameter('odom_max_age', 1.0)          # s, odom 新鲜度上限
        self.declare_parameter('odom_yaw_sign', 1.0)         # 若日志显示滑差异常大, 试 -1
        self.declare_parameter('log_dir', '~/classify_real_logs')

        self.rm_ns = str(self.get_parameter('rm_ns').value).strip('/')
        self.x_grasp = float(self.get_parameter('x_grasp').value)
        self.z_grasp = float(self.get_parameter('z_grasp').value)
        self.x_drop = float(self.get_parameter('x_drop').value)
        self.z_drop = float(self.get_parameter('z_drop').value)
        self.z_safe = float(self.get_parameter('z_safe').value)
        self.gripper_power = float(self.get_parameter('gripper_power').value)
        self.max_step = float(self.get_parameter('max_step_m').value)
        self.x_range = list(self.get_parameter('x_range').value)
        self.z_range = list(self.get_parameter('z_range').value)
        self.chassis_angular_speed = float(self.get_parameter('chassis_angular_speed').value)
        self.chassis_linear_speed = float(self.get_parameter('chassis_linear_speed').value)
        self.chassis_timeout = float(self.get_parameter('chassis_timeout').value)
        self.use_chassis = bool(self.get_parameter('use_chassis').value)
        self.grid_az = {k: float(v) for k, v in json.loads(
            self.get_parameter('grid_azimuth').value).items()}
        self.bin_az = {k: float(v) for k, v in json.loads(
            self.get_parameter('bin_azimuth').value).items()}
        self.confirm_via_detect = bool(self.get_parameter('confirm_via_detect').value)
        self.miss_if_still_present = bool(self.get_parameter('miss_if_still_present').value)
        self.align_mode = str(self.get_parameter('align_mode').value).strip().lower()
        self.align_max_iter = int(self.get_parameter('align_max_iter').value)
        self.align_lateral_tol = float(self.get_parameter('align_lateral_tol').value)
        self.align_settle = float(self.get_parameter('align_settle').value)
        self.align_det_timeout = float(self.get_parameter('align_det_timeout').value)
        self.align_min_step = float(self.get_parameter('align_min_step').value)
        self.align_gain = float(self.get_parameter('align_gain').value)
        self.base_offset = float(self.get_parameter('base_offset').value)
        self.base_offset_auto = bool(self.get_parameter('base_offset_auto').value)
        self.base_offset_max = float(self.get_parameter('base_offset_max').value)
        self.base_offset_alpha = float(self.get_parameter('base_offset_alpha').value)
        self.base_offset_est = self.base_offset   # 在线自估后的当前值
        self.grasp_x_from_vision = bool(self.get_parameter('grasp_x_from_vision').value)
        self.heading_from_odom = bool(self.get_parameter('heading_from_odom').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value).strip() \
            or '/%s/odom' % self.rm_ns
        self.odom_max_age = float(self.get_parameter('odom_max_age').value)
        self.odom_yaw_sign = float(self.get_parameter('odom_yaw_sign').value)
        self.log_dir = os.path.expanduser(str(self.get_parameter('log_dir').value))

        self._cb = ReentrantCallbackGroup()
        self.state_pub = self.create_publisher(String, '/grasp_state', 10)
        self.tcp = None
        self.create_subscription(
            PointStamped, '/%s/arm_position' % self.rm_ns, self._tcp_cb, 10)
        # 视觉: /grid_detections(std_msgs/String JSON) 与仿真版同接口。
        # 每条都带 grid_mapper 算好的 x/y(臂基座系) —— 闭环要的观测量现成就在这。
        self.dets = []
        self._det_stamp = 0.0        # 最近一条的时刻, 用于判断"转完之后的"新数据
        self.create_subscription(String, '/grid_detections', self._det_cb, 10)
        # 底盘实测转角: 驱动在 /<ns>/odom 发 nav_msgs/Odometry(含姿态)。
        # 用它把 heading 从"我发了多少"改成"实际转了多少", 麦轮打滑因此可见可查。
        self._odom_yaw = None
        self._odom_stamp = 0.0
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.move_client = ActionClient(
            self, MoveArm, '/%s/move_arm' % self.rm_ns, callback_group=self._cb)
        self.grip_client = ActionClient(
            self, GripperControl, '/%s/gripper' % self.rm_ns, callback_group=self._cb)
        self.base_client = ActionClient(
            self, Move, '/%s/move' % self.rm_ns, callback_group=self._cb)

        self._move_gh = None
        self._align_gh = None
        self._last_actual_rot = None  # 上一次 _rotate 的实测转角(供在线估偏置用)
        self._last_cmd = (self.x_grasp, self.z_safe)
        self._heading = 0.0          # 相对初始的朝向; 有 odom 时由实测转角推进
        self._x_now = self.x_grasp   # 本轮抓取实际使用的 x(闭环后=实测值, 否则=x_grasp)
        self.gripper_closed = False

        self._csv = None
        self._csvw = None
        self._t0 = 0.0

        self._as = ActionServer(
            self, ClassifyGrasp, 'classify_grasp',
            execute_callback=self._execute, goal_callback=self._goal,
            cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=self._cb)
        self.get_logger().info(
            'classify_grasp_server_real 就绪 (网格=%s 料盒=%s ns=/%s)'
            % (list(self.grid_az), list(self.bin_az), self.rm_ns))

    # ---------- 基础 ----------
    def _tcp_cb(self, m):
        self.tcp = m.point

    def _tcp_xz(self):
        if self.tcp is None:
            return None
        return float(self.tcp.x), float(self.tcp.z)

    def _det_cb(self, m):
        try:
            items = json.loads(m.data)
        except Exception:
            return
        self.dets = [it for it in items if it.get('grid')]
        self._det_stamp = _time.time()

    def _cell_present(self, grid):
        return any(d.get('grid') == grid for d in self.dets)

    def _odom_cb(self, m):
        q = m.pose.pose.orientation
        s = 2.0 * (q.w * q.z + q.x * q.y)
        c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = self.odom_yaw_sign * math.atan2(s, c)
        self._odom_stamp = _time.time()

    def _odom_yaw_now(self):
        """当前 odom yaw(rad); 未开启/未收到/过期 -> None(调用方按无反馈处理)。"""
        if not self.heading_from_odom or self._odom_yaw is None:
            return None
        if _time.time() - self._odom_stamp > self.odom_max_age:
            return None
        return float(self._odom_yaw)

    def _measure_target(self, grid, timeout=None):
        """等一条**新**的 /grid_detections, 取该网格的实测臂基座系坐标。

        返回 (x, y, score); 超时或该网格不在检测里 -> None。
        "新"是关键: 必须采转完底盘之后的帧, 否则量到的是转之前的位置(闭环失效)。
        """
        timeout = self.align_det_timeout if timeout is None else float(timeout)
        t0 = _time.time()
        while rclpy.ok() and _time.time() - t0 < timeout:
            if self._det_stamp > t0:
                break
            _time.sleep(0.03)
        picks = [d for d in self.dets if d.get('grid') == grid]
        if not picks:
            return None
        best = max(picks, key=lambda d: float(d.get('score', 0.0) or 0.0))
        try:
            return (float(best['x']), float(best['y']),
                    float(best.get('score', 0.0) or 0.0))
        except (KeyError, TypeError, ValueError):
            return None

    def _set_grasp_x(self, x_meas):
        """把"实测物体 x"变成抓取用的 x, 并钳进工作空间(安全闸门不被绕过)。"""
        if not self.grasp_x_from_vision:
            self._x_now = self.x_grasp
            return
        lo, hi = float(self.x_range[0]), float(self.x_range[1])
        x_use = min(max(float(x_meas), lo), hi)
        if abs(x_use - float(x_meas)) > 1e-9:
            self.get_logger().warn(
                '[grasp_x] 实测 x=%.4f 超工作空间[%.2f, %.2f], 已钳到 %.4f'
                % (x_meas, lo, hi, x_use))
            self._log_row('grasp_x', 'CLAMP',
                          'meas=%.4f use=%.4f' % (x_meas, x_use))
        else:
            self.get_logger().info(
                '[grasp_x] 用实测 x=%.4f 抓 (固定值 %.4f)' % (x_use, self.x_grasp))
            self._log_row('grasp_x', 'VISION',
                          'use=%.4f fixed=%.4f' % (x_use, self.x_grasp))
        self._x_now = x_use

    def _in_workspace(self, x, z):
        return (self.x_range[0] <= x <= self.x_range[1]
                and self.z_range[0] <= z <= self.z_range[1])

    def _pub_state(self, state):
        m = String(); m.data = state
        self.state_pub.publish(m)
        self.get_logger().info('[state] %s' % state)
        self._log_row('state', state, '')

    def _wait_fut(self, fut, timeout=2.0):
        # 不用 spin_until_future_complete: 它会冻结订阅
        t0 = _time.time()
        while rclpy.ok() and not fut.done() and _time.time() - t0 < timeout:
            _time.sleep(0.02)
        return fut.result() if fut.done() else None

    # ---------- CSV 日志 ----------
    def _open_csv(self):
        os.makedirs(self.log_dir, exist_ok=True)
        fn = os.path.join(self.log_dir, 'classify_real_%s.csv' % _time.strftime('%Y%m%d_%H%M%S'))
        self._csv = open(fn, 'w', newline='')
        self._csvw = csv.writer(self._csv)
        self._csvw.writerow(['t', 'type', 'tag', 'detail'])
        self._csv.flush()
        self._t0 = _time.time()
        self.get_logger().info('日志: %s' % fn)

    def _close_csv(self):
        if self._csv is not None:
            self._csv.close()
        self._csv = None
        self._csvw = None

    def _log_row(self, typ, tag, detail):
        if self._csvw is None:
            return
        self._csvw.writerow(['%.3f' % (_time.time() - self._t0), typ, tag, detail])
        self._csv.flush()

    # ---------- 底盘: move(相对 theta) ----------
    def _ensure_no_active_align(self):
        gh = self._align_gh
        if gh is not None:
            self._wait_fut(gh.cancel_goal_async(), 2.0)
            self._align_gh = None

    def _rotate(self, delta, tag):
        """相对转底盘 delta(rad), 并用 **odom 实测转角** 推进 heading。返回 (ok, msg)。

        与旧实现的唯一实质差别: 旧代码 `self._heading += delta` 把"我发了多少"当成
        "我转了多少"。麦轮打滑/惯性会让实际转角 ≠ 指令, 那个偏差会留在 heading 里
        并逐轮累积。现在转完读一次 odom yaw, 用实测差值推进 —— 打滑量写进日志可诊断,
        且不再污染后续轮次。
        """
        if abs(delta) < HEADING_DEADBAND:
            self.get_logger().info('[align] %s 已在对位死区内 (Δ=%.3f)' % (tag, delta))
            return True, 'ok'
        self._ensure_no_active_align()
        yaw0 = self._odom_yaw_now()
        g = Move.Goal()
        g.x = 0.0
        g.y = 0.0
        g.theta = float(delta)
        g.linear_speed = float(self.chassis_linear_speed)
        g.angular_speed = float(self.chassis_angular_speed)
        gh = self._wait_fut(self.base_client.send_goal_async(g), SEND_TIMEOUT)
        if gh is None or not gh.accepted:
            msg = 'move(底盘) goal 失败/被拒: %s Δ=%.3f' % (tag, delta)
            self._log_row('align', tag, 'FAIL ' + msg)
            return False, msg
        self._align_gh = gh
        res = self._wait_fut(gh.get_result_async(), self.chassis_timeout)
        self._align_gh = None
        if res is None:
            self._wait_fut(gh.cancel_goal_async(), 2.0)
            msg = 'move(底盘) 超时(>%.1fs): %s Δ=%.3f' % (self.chassis_timeout, tag, delta)
            self._log_row('align', tag, 'FAIL ' + msg)
            return False, msg
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            msg = 'move(底盘) 失败(status=%d): %s' % (res.status, tag)
            self._log_row('align', tag, 'FAIL ' + msg)
            return False, msg
        _time.sleep(0.25)                     # 等底盘停稳, 让 odom 跟上
        yaw1 = self._odom_yaw_now()
        self._last_actual_rot = None
        if yaw0 is not None and yaw1 is not None:
            actual = _wrap(yaw1 - yaw0)
            self._last_actual_rot = actual
            slip = _wrap(actual - delta)
            self._heading = _wrap(self._heading + actual)
            self.get_logger().info(
                '[align] %s 指令 Δ=%.3f 实测 %.3f (滑差 %.3f) -> heading=%.3f'
                % (tag, delta, actual, slip, self._heading))
            self._log_row('align', tag, 'OK cmd=%.4f actual=%.4f slip=%.4f heading=%.4f'
                          % (delta, actual, slip, self._heading))
            if abs(slip) > 0.5:
                self.get_logger().warn(
                    '[align] 滑差 %.3f rad 异常大 —— 若每次都这样, 检查 odom_yaw_sign '
                    '(现 %.0f) 是否与 Move.theta 同向, 或地面打滑严重'
                    % (slip, self.odom_yaw_sign))
        else:
            self._heading = _wrap(self._heading + delta)
            self.get_logger().warn(
                '[align] %s 无 odom 反馈, 退回按指令值推进 heading=%.3f' % (tag, self._heading))
            self._log_row('align', tag, 'OK cmd=%.4f actual=n/a heading=%.4f'
                          % (delta, self._heading))
        return True, 'ok'

    def _maybe_calibrate_base_offset(self, x1, y1, x2, y2, psi):
        """用"转前/转后两次实测 + 实测转角"在线更新基座偏置估计(指数平滑抗噪)。"""
        if not self.base_offset_auto:
            return
        est = estimate_base_offset(x1, y1, x2, y2, psi)
        if est is None:
            self._log_row('base_offset', 'SKIP', '估计退化(sin s 太小)')
            return
        if not (-0.05 <= est <= self.base_offset_max):
            self.get_logger().warn(
                '[base_offset] 估计 %.4f 超出合理范围 [0, %.2f], 本次忽略'
                % (est, self.base_offset_max))
            self._log_row('base_offset', 'REJECT', 'est=%.4f' % est)
            return
        old = self.base_offset_est
        a = min(max(self.base_offset_alpha, 0.0), 1.0)
        self.base_offset_est = a * est + (1 - a) * old
        self.get_logger().info(
            '[base_offset] 在线估计 %.4f -> 平滑后 %.4f (原用 %.4f)'
            % (est, self.base_offset_est, old))
        self._log_row('base_offset', 'EST',
                      'raw=%.4f smoothed=%.4f (was %.4f)' % (est, self.base_offset_est, old))

    def _align(self, azimuth, tag):
        """开环: 把静态方位角 azimuth 转到臂正前方(仅用于料盒/兜底)。返回 (ok, msg)。"""
        if not self.use_chassis:
            return True, 'ok(use_chassis=false)'
        return self._rotate(_wrap(azimuth - self._heading), tag)

    def _align_visual(self, grid, cls, tag):
        """视觉闭环对位: 转 -> 重测 -> 算横向残差 -> 再转, 直到 |y| ≤ tol。

        返回 (ok, msg, xy); xy = 最后一轮实测的 (x, y)。

        原理: 臂只有 x/z, 没有 yaw 和横向 → 横向只能靠底盘转。而"物体是否在臂的
        垂直平面内"这件事, 图像直接看得见 —— 于是不必信任"我转了多少", 只需反复问
        "物体现在偏多少"。底盘打滑/惯性/旋转中心≠臂基座 全部被观测吸收。
        """
        xy = None
        prev = None                     # 上一轮"转之前"的实测 (x, y)
        for i in range(max(1, self.align_max_iter)):
            m = self._measure_target(grid)
            if m is None:
                msg = ('%s 第%d轮测不到 %s(cls=%s): 该网格不在 /grid_detections 里'
                       ' (被臂遮挡? 检测漏了? 或物体已不在桌面)'
                       % (tag, i + 1, grid, cls))
                self._log_row('align', tag, 'FAIL ' + msg)
                return False, msg, xy
            x_o, y_o, sc = m
            # 有"上一轮转前的实测 + 本轮转后的实测 + 实测转角" -> 顺手在线校准基座偏置
            if prev is not None and self._last_actual_rot is not None:
                self._maybe_calibrate_base_offset(prev[0], prev[1], x_o, y_o,
                                                  self._last_actual_rot)
                self._last_actual_rot = None
                prev = None
            xy = (x_o, y_o)
            self.get_logger().info(
                '[vloop] %s iter=%d 实测物体(臂基座系) x=%.4f y=%.4f score=%.2f'
                % (tag, i + 1, x_o, y_o, sc))
            self._log_row('align', tag,
                          'MEAS iter=%d x=%.4f y=%.4f score=%.3f' % (i + 1, x_o, y_o, sc))
            g = self.align_gain if self.align_gain > 1e-6 else 1e-6
            d_raw, why = vloop_correction(x_o, y_o, self.align_lateral_tol,
                                          self.align_min_step / g,
                                          self.base_offset_est)
            if why == 'converged':
                return True, 'ok(闭环 %d 轮收敛, 横向残差 %.4f m)' % (i + 1, y_o), xy
            if why == 'degenerate':
                msg = ('%s 实测 (x=%.4f, y=%.4f) 与臂基座几乎重合, 方位角病态'
                       % (tag, x_o, y_o))
                self._log_row('align', tag, 'FAIL ' + msg)
                return False, msg, xy
            if why == 'too_small':
                msg = ('%s 横向残差 %.4f m > tol %.4f, 但需转 %.4f rad 乘阻尼 %.2f 后 '
                       '< 最小步长 %.4f —— 再转也不会收敛'
                       % (tag, y_o, self.align_lateral_tol,
                          math.atan2(y_o, x_o + self.base_offset_est),
                          self.align_gain, self.align_min_step))
                self._log_row('align', tag, 'FAIL ' + msg)
                return False, msg, xy
            dtheta = g * d_raw
            ok, msg = self._rotate(dtheta, '%s/i%d' % (tag, i + 1))
            if not ok:
                return False, msg, xy
            prev = (x_o, y_o)          # 记下"转前"实测, 下轮用来在线估基座偏置
            _time.sleep(self.align_settle)
        m = self._measure_target(grid)          # 轮次用尽, 最后再量一次
        if m is not None:
            xy = (m[0], m[1])
            if abs(m[1]) <= self.align_lateral_tol:
                return True, 'ok(闭环收敛, 横向残差 %.4f m)' % m[1], xy
            msg = ('%s 闭环 %d 轮未收敛, 最后横向残差 %.4f m > tol %.4f'
                   % (tag, self.align_max_iter, m[1], self.align_lateral_tol))
        else:
            msg = '%s 闭环 %d 轮未收敛, 且最后测不到目标' % (tag, self.align_max_iter)
        self._log_row('align', tag, 'FAIL ' + msg)
        return False, msg, xy

    def _align_grid(self, grid, cls, tag='ALIGN_GRID'):
        """网格对位入口: align_mode=open 走静态方位角, closed 走视觉闭环。"""
        if not self.use_chassis:
            # 不转底盘时无法做横向对位; 若装了闭环仍想量一下实测偏差, 这里只提示
            if self.align_mode != 'open':
                m = self._measure_target(grid)
                if m is not None:
                    self.get_logger().info(
                        '[vloop] use_chassis=false, 跳过对位; 实测横向偏差 y=%.4f m' % m[1])
            return True, 'ok(use_chassis=false)'
        if self.align_mode == 'open':
            if grid not in self.grid_az:
                return False, '未知网格: %s' % grid
            return self._align(self.grid_az[grid], tag)
        if grid not in self.grid_az:
            self.get_logger().info(
                '[align] 闭环模式不需要 grid_azimuth 表; %s 不在表里也无妨' % grid)
        ok, msg, xy = self._align_visual(grid, cls, tag)
        if ok and xy is not None:
            self._set_grasp_x(xy[0])
        return ok, msg

    # ---------- 臂: move_arm(单 goal) ----------
    def _ensure_no_active_move(self):
        gh = self._move_gh
        if gh is not None:
            self.get_logger().warn('上一 move_arm goal 未结束, 先 cancel')
            self._wait_fut(gh.cancel_goal_async(), 2.0)
            self._move_gh = None

    def _move_once(self, x, z, tag):
        self._ensure_no_active_move()
        g = MoveArm.Goal()
        g.x = float(x)
        g.z = float(z)
        g.relative = False
        gh = self._wait_fut(self.move_client.send_goal_async(g), SEND_TIMEOUT)
        if gh is None or not gh.accepted:
            msg = 'move_arm goal 失败/被拒: %s(%.3f,%.3f)' % (tag, x, z)
            self._log_row('wp', tag, 'FAIL ' + msg)
            return False, msg
        self._move_gh = gh
        res = self._wait_fut(gh.get_result_async(), MOVE_TIMEOUT + 1.0)
        self._move_gh = None
        if res is None:
            self._wait_fut(gh.cancel_goal_async(), 2.0)
            msg = 'move_arm 超时(>%.1fs): %s(%.3f,%.3f)' % (MOVE_TIMEOUT, tag, x, z)
            self._log_row('wp', tag, 'FAIL ' + msg)
            return False, msg
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            msg = 'move_arm 失败(status=%d): %s(%.3f,%.3f)' % (res.status, tag, x, z)
            self._log_row('wp', tag, 'FAIL ' + msg)
            return False, msg
        self._log_row('wp', tag, 'OK(%.3f,%.3f)' % (x, z))
        return True, 'ok'

    def _goto(self, tag, x, z):
        """工作空间校验 + 按 max_step_m 插值子航点 + 逐段 move_arm。返回 (ok, msg)。"""
        if not self._in_workspace(x, z):
            msg = '目标超出工作空间: %s(%.3f,%.3f) x_range=%s z_range=%s' \
                  % (tag, x, z, self.x_range, self.z_range)
            self.get_logger().error(msg)
            self._log_row('wp', tag, 'FAIL ' + msg)
            return False, msg
        cur = self._last_cmd
        dist = math.hypot(x - cur[0], z - cur[1])
        n = max(1, int(math.ceil(dist / self.max_step)))
        if n > 1:
            self.get_logger().info('[move] %s(%.3f,%.3f) %.3fm -> %d 段'
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

    # ---------- 爪 ----------
    def _gripper(self, target, tag):
        g = GripperControl.Goal()
        g.target_state = target
        g.power = float(self.gripper_power)
        gh = self._wait_fut(self.grip_client.send_goal_async(g), SEND_TIMEOUT)
        if gh is None or not gh.accepted:
            msg = 'gripper goal 失败/被拒: %s' % tag
            self._log_row('grip', tag, 'FAIL ' + msg)
            return False, msg
        res = self._wait_fut(gh.get_result_async(), GRIP_TIMEOUT + 1.0)
        if res is None:
            self._wait_fut(gh.cancel_goal_async(), 2.0)
            msg = 'gripper 超时(>%.1fs): %s' % (GRIP_TIMEOUT, tag)
            self._log_row('grip', tag, 'FAIL ' + msg)
            return False, msg
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            msg = 'gripper 失败(status=%d): %s' % (res.status, tag)
            self._log_row('grip', tag, 'FAIL ' + msg)
            return False, msg
        self.gripper_closed = (target == GripperControl.Goal.CLOSE)
        self._log_row('grip', tag, 'OK')
        return True, 'ok'

    def _confirm_lift(self, timeout=10.0):
        """LIFT 确认: 轮询 arm_position(反馈滞后数秒) 直到 z 升过阈值。"""
        th = 0.5 * (self.z_grasp + self.z_safe)
        t0 = _time.time()
        p = self._tcp_xz()
        while rclpy.ok() and _time.time() - t0 < timeout:
            p = self._tcp_xz()
            if p is not None and p[1] >= th:
                return True, 'ok'
            _time.sleep(0.2)
        if p is None:
            return False, '无 arm_position 反馈, 无法确认末端抬起'
        return False, '末端未抬起: 等 %.1fs 后 z=%.3f < %.3f' % (timeout, p[1], th)

    def _recover_home(self):
        """失败后安全回收: 先抬到 z_safe, 回 HOME 位, 夹着东西则张爪。不抛异常。"""
        try:
            cur = self._last_cmd
            if cur[1] < self.z_safe and self._in_workspace(cur[0], self.z_safe):
                self._goto('RECOVER_LIFT', cur[0], self.z_safe)
            self._goto('HOME', self.x_grasp, self.z_safe)
            if self.gripper_closed:
                self._gripper(GripperControl.Goal.OPEN, 'RECOVER_OPEN')
        except Exception as e:
            self.get_logger().error('回收 HOME 失败: %s' % e)

    # ---------- Action ----------
    def _goal(self, goal):
        return GoalResponse.ACCEPT

    async def _execute(self, goal_handle):
        goal = goal_handle.request
        grid, cls, bin_id = goal.grid_id, goal.class_id, goal.bin_id
        OPEN, CLOSE = GripperControl.Goal.OPEN, GripperControl.Goal.CLOSE

        if grid not in self.grid_az or bin_id not in self.bin_az:
            goal_handle.succeed()
            return ClassifyGrasp.Result(
                success=False, bin_id=bin_id,
                message='未知网格/料盒: %s / %s' % (grid, bin_id),
                error_code=EC_UNKNOWN_TARGET)

        fb = ClassifyGrasp.Feedback()
        fb.gripper_closed = False
        fb.current_state = 'SCAN_OK'
        goal_handle.publish_feedback(fb)

        self._open_csv()
        bad = [(t, x, z) for t, x, z in
               [('x_grasp', self.x_grasp, self.z_grasp), ('x_grasp_safe', self.x_grasp, self.z_safe),
                ('x_drop', self.x_drop, self.z_drop), ('x_drop_safe', self.x_drop, self.z_safe)]
               if not self._in_workspace(x, z)]
        if bad:
            msg = '点位超工作空间, 拒绝执行: %s (x_range=%s z_range=%s)' \
                  % (', '.join(t for t, _, _ in bad), self.x_range, self.z_range)
            self.get_logger().error(msg)
            self._log_row('error', 'SAFETY', msg)
            self._close_csv()
            goal_handle.abort()
            return ClassifyGrasp.Result(success=False, bin_id=bin_id, message=msg,
                                        error_code=EC_UNREACHABLE)

        for cli, name in ((self.move_client, 'move_arm'), (self.grip_client, 'gripper')):
            t0 = _time.time()
            while rclpy.ok() and not cli.server_is_ready() and _time.time() - t0 < 5.0:
                _time.sleep(0.1)
            if not cli.server_is_ready():
                msg = '驱动 action server 不可用: /%s/%s' % (self.rm_ns, name)
                self.get_logger().error(msg)
                self._log_row('error', 'DRIVER', msg)
                self._close_csv()
                goal_handle.abort()
                return ClassifyGrasp.Result(success=False, bin_id=bin_id, message=msg,
                                            error_code=EC_INTERNAL)

        present_before = self._cell_present(grid)
        self.get_logger().info('任务: %s(%s) -> %s | 抓(%.3f,%.3f) 放(%.3f,%.3f) 检测在场=%s'
                               % (grid, cls, bin_id, self.x_grasp, self.z_grasp,
                                  self.x_drop, self.z_drop, present_before))

        def fail(msg):
            self._pub_state('FAIL')
            self._log_row('error', 'CYCLE', msg)
            self._recover_home()
            self._close_csv()
            goal_handle.abort()
            return ClassifyGrasp.Result(success=False, bin_id=bin_id, message=msg,
                                        error_code=_err_code_of(msg))

        try:
            self._x_now = self.x_grasp     # 本轮起始用固定值; 闭环成功后会被改成实测值
            steps = [
                ('HOME', lambda: self._gripper(OPEN, 'OPEN')),
                ('HOME', lambda: self._goto('HOME', self.x_grasp, self.z_safe)),
                ('ALIGN_GRID', lambda: self._align_grid(grid, cls)),
                ('APPROACH_GRID', lambda: self._goto('APPROACH_GRID', self._x_now, self.z_safe)),
                ('DESCEND_GRID', lambda: self._goto('DESCEND_GRID', self._x_now, self.z_grasp)),
                ('GRASP', lambda: self._gripper(CLOSE, 'CLOSE')),
                ('LIFT', lambda: self._goto('LIFT', self._x_now, self.z_safe)),
                ('LIFT', self._confirm_lift),
                ('ALIGN_BIN', lambda: self._align(self.bin_az[bin_id], 'ALIGN_BIN')),
                ('DESCEND_BIN', lambda: self._goto('DESCEND_BIN', self.x_drop, self.z_drop)),
                ('RELEASE', lambda: self._gripper(OPEN, 'OPEN')),
                ('LIFT_BIN', lambda: self._goto('LIFT_BIN', self.x_drop, self.z_safe)),
            ]
            for st, fn in steps:
                if goal_handle.is_cancel_requested:
                    self._close_csv()
                    self._recover_home()
                    goal_handle.canceled()
                    return ClassifyGrasp.Result(success=False, bin_id=bin_id,
                                                message='goal 被取消',
                                                error_code=EC_MOTION_FAILED)
                self._pub_state(st)
                fb.current_state = st
                fb.gripper_closed = self.gripper_closed
                goal_handle.publish_feedback(fb)
                ok, msg = fn()
                if not ok:
                    return fail('%s: %s' % (st, msg))

            still = self._cell_present(grid) if self.confirm_via_detect else None
            if self.confirm_via_detect:
                if present_before and not still:
                    self.get_logger().info('抓取确认: %s 已从检测中消失' % grid)
                elif present_before and still:
                    self.get_logger().warn('确认提示: %s 抓取后仍在检测中(可能是遮挡或夹空)' % grid)
                    if self.miss_if_still_present:
                        return fail('夹空: 抓取后 %s 仍在检测中' % grid)

            self._pub_state('CYCLE_DONE')
            self._close_csv()
            goal_handle.succeed()
            note = '(检测确认)' if (self.confirm_via_detect and present_before and not still) else ''
            return ClassifyGrasp.Result(
                success=True, bin_id=bin_id,
                message='真机航点/爪动作全部完成%s' % note,
                error_code=EC_NONE)
        except Exception as e:
            self.get_logger().error('执行异常: %s' % e)
            return fail('执行异常: %s' % e)


def main():
    rclpy.init()
    node = ClassifyGraspServerReal()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
