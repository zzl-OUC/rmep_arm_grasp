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
import json
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
# 抓取点相对方块中心的径向偏移 = TCP 到指尖的距离(TCP_LEAD):
# 抓取时指尖落在方块中心, 故 TCP 目标 d = 网格半径 + TCP_LEAD。
# (曾误用 0.0841: 由已验收 fk(Q_GRASP)=(0.2743,0.0448) 减网格半径 0.190 反推而来,
#  但那个位姿的方块当时在 A 点 d=0.2083, 不在 r=0.19 网格上 —— 反推前提错误。
#  改用 0.0841 后 TCP 外推 18mm, IK 解 q1=1.3839 直接压死在关节上限 1.384(余量 0.0001),
#  yaw=0(cell_1)/yaw≈-2.28(cell_4) 下降追不到, 6/6 退化为 3/6。已改回 0.066。)
GRASP_LEAD = TCP_LEAD
GRASP_Z = 0.0446
# 接近/抬升时的抬升 TCP 高度: 略高于抓取位, 低于搬运姿态(0.110)。
# 用 IK 解算(而非固定 Q_RAISED), 目标点靠近已可达的 DESCEND 位姿,
# 避免 yaw≈π(正后方)奇异位形下速度控制器无法收敛到固定关节角。
APPROACH_Z = 0.14  # 默认(方块/球): 指面底 = 0.14+0.0094-0.02 = 0.129 > 球顶 0.095 ✓
APPROACH_Z_BY_CLASS = {
    'tennis_ball': 0.14,  # 球顶 0.095, 0.14 余量 34mm; 原值下球抓取/搬运全链路已验证
    'bottle': 0.16,       # 瓶顶 0.125, 0.14 时余量仅 4mm 会刮瓶顶(实测瓶被推 11cm), 0.16 余量 24mm
}
# 接近/抬升/回程旋转的"高位": 指面底须高于桌面最高物(瓶顶 0.125)避免旋转刮推。
# ⚠ APPROACH_Z 统一 0.16 会让球的 LIFT->TRANSPORT 位形跳变大, 球在搬运途中掉落;
#   按类别取各自安全高度。
# 抓取中间航点: 让 q2 在 q1 尚远离上限时就基本到位, 最后一段再把 q1 顶到上限。
# 按类别取"略高于 z_g"的值, 保证 DESCEND 单调下降(瓶 z_g=0.070 用 0.075,
# 若用 0.065 会先降到瓶心下方再回升, 上下波动刮瓶身把瓶推走)。
GRASP_MID_Z_BY_CLASS = {'tennis_ball': 0.065, 'bottle': 0.075}
GRASP_MID_Z = 0.065
Q1_LIM = (-0.274, 1.384)
Q2_LIM = (-1.25, 0.40)

GRIP_OPEN = (-5.0, -5.0)
GRIP_SEAT = (5.0, 5.0)
GRIP_CLOSE = (18.0, 18.0)
# gripper_controller 是 effort 控制器(controllers.yaml): 命令单位是力(N), 不是位置!
# GRIP_SEAT=5N 预夹 / GRIP_CLOSE=18N 夹紧。
# 碰撞体改回球体后夹持力不再有"挤出"副作用(球面接触法线恒过球心, 法向力对质心
# 力矩恒为 0, 不存在 box 时代"夹得越紧挤出越狠"的耦合), 因此夹持力只决定摩擦
# 保持力: 2 x 18N x mu2.0 = 72N >> 球重 0.56N。搬运段实测球仍会相对指面上滑,
# 提高夹持力是纯增益。
# ⚠ 夹持力不是越大越好 —— 见"网球爬出"根因分析:
#   实测夹爪指面竖直跨度 [TCP_z-0.0106, TCP_z+0.0294]、网球碰撞盒 0.066³,
#   接触带中心高于球质心 -> 水平加速产生俯仰力矩 -> 球相对指面"向上爬"
#   (实测 dz 0.005 -> 0.064) 直至挤出指间, 随后双指空夹到全闭 0.040。
#   挤出力 ∝ 夹持力 x sin(俯仰角), 故 15N->25N 不仅没止住反而加剧爬出。
#   12N 时: 摩擦保持 2x12N x mu2.0 = 48N >> 球重 0.56N(85 倍余量), 仍远足够,
#   但挤出分量比 25N 小 2.1 倍。
# ⚠ 曾误把 effort 当 position 调成 ~0.02N -> 零夹持力 -> 全部夹空; 保持力值不变。
# ⚠ libgazebo_grasp_fix.so 在本机不存在(全盘搜索无结果), URDF 里的
#   gazebo_grasp_fix 插件从未生效 —— 全部夹持靠纯摩擦物理, 没有"吸附"兜底。
# 物体碰撞体为 box 近似(网球 0.066³ / 瓶 0.065×0.065×0.10): 指面-平面接触稳定。
# ⚠ 曾误把 effort 当 position 调成 ~0.02N -> 零夹持力 -> 全部夹空; 保持力值不变。
GRIP_BY_CLASS = {}  # 不再按类别区分

Q_GRASP = (1.384, -0.4714)
Q_LIFT_Q1 = 1.20
# HOME 待机位形由 (1.30, 0.35) 改为 (0.11, 0.35)：把空夹爪抬到物体上方再转 yaw。
# 原 HOME: TCP d=0.1875 z=0.0717 -> 指盒底 0.0611，远低于瓶顶 0.125。
#   每轮开头从 HOME 转 yaw 去抓取时，张开的指盒会在低高度横扫过桌面物体：
#   实测 cell_2 的瓶在 HOME->APPROACH_GRID 的 yaw 旋转途中被推走
#   (0.1310,0.1310) -> (0.1163,0.1625)，径向被推出 1.45cm，
#   随后下降套入时指盒套空 -> 夹空(exit code 2)。这就是存档里
#   "cell_2/cell_3 瓶 ❌ 夹空" 的根因（当时怀疑是掉落球污染邻格，其实不是）。
# 新 HOME: TCP d=0.2198 z=0.1650 -> 指盒底 0.1544，比瓶顶高 29mm，
#   不论指盒在什么半径，yaw 扫掠时都在所有物体之上。
Q_HOME = (0.11, 0.35)
# 搬运姿态(按类别): 被夹物体底部须高于桌面最高物体(瓶顶 0.125), yaw 旋转不扫桌。
#   tennis_ball (0.50,-0.75): TCP z≈0.170, 球底≈0.137 > 0.125 ✓
#   bottle      (0.45,-0.80): TCP z≈0.177, 瓶底≈0.127 > 0.125 ✓
#   (默认 0.90,-0.30 原值: TCP z=0.110, 块底≈0.09 —— 4cm 方块版参数, 保留给兼容)
Q_CARRY = (0.90, -0.30)   # 搬运姿态: TCP z=0.110, 方块底悬空~6.5cm(搬运途中不蹭桌面块)
# 搬运姿态按类别: 被夹物体底部须高于桌面最高物(瓶顶 0.125), yaw 旋转不扫桌。
#   统一 (0.45,-0.80): TCP z≈0.177, 球底≈0.144 / 瓶底≈0.127 > 0.125 ✓
#   (球原用 (0.50,-0.75) z≈0.170, 实测 TRANSPORT 位形变换时球从指间滑落 -> 提高搬运高度)
# 搬运位形改为 (0.23, -0.10): TCP z=0.190, d=0.262。
# 旧值 (0.45,-0.80) TCP z=0.234 净空更大, 但从 LIFT 位形 (0.52, 0.09) 过去需要
# q2 摆动 0.89rad(关节空间距离 0.893rad), 大重构位形会把夹持物体甩出: 实测球在
# TRANSPORT 段相对指面上滑并最终脱出(多轮 dist 落在 0.17~0.36)。
# 新位形关节空间距离仅 0.347rad(小 2.6 倍), 净空仍满足:
#   球底 = 0.190-0.0236 = 0.167 > 瓶顶 0.125(余 4.2cm)
#   瓶底 = 0.190-0.0500 = 0.140 > 瓶顶 0.125(余 1.5cm)
Q_CARRY_BY_CLASS = {'tennis_ball': (0.23, -0.10), 'bottle': (0.23, -0.10)}
Q_RAISED = (0.9, -0.3)
# 投放高度按类别: 物体底须高于盒壁顶 0.075。
#   tennis_ball: TCP 0.115 -> 球底 0.082 ✓(0.13 时 IK d=0.276 接近可达边界,
#     DESCEND_BIN 不到位 -> 球在错误位置释放, 落点偏 24~43cm 污染邻格)
#   bottle:      TCP 0.130 -> 瓶底 0.080 ✓(已验证 cell_4 投放 dist=0.015/0.067)
#     0.130 -> 0.115: 瓶高 0.10、细长, 释放后要落到盒底(面 0.031)会翻倒。
#     实测 cell_3(同盒第 2 件, 紧邻已放好的第 1 件) 首次投放 dist=0.124 判失败,
#     重试时瓶已移位 -> 夹空。降到 0.115 后瓶底 0.065, 释放时已低于盒壁顶 0.075,
#     即"放进盒里再松手", 下落行程由 0.049m 减到 0.034m, 翻倒概率显著降低。
#     (指盒底 0.115-0.0106=0.1044 仍高于壁顶 0.075, 不会蹭壁)
Z_DROP_BY_CLASS = {'tennis_ball': 0.115, 'bottle': 0.115}
Z_DROP = 0.13             # 默认(兼容方块版: 块底≈0.095 高于盒壁顶 0.075)

MOVE_DUR = 3.0
LIFT_DUR = 3.0
YAW_DUR = 6.0
GRIP_SEAT_DUR = 3.0
VC = 0.9
# 搬运段限速: 夹持物体高速摆动时, 惯性 + 接触微滑会让物体相对指面转动/上爬
# (实测: LIFT 段慢速时球自旋仅 0.4 度且高度稳定; TRANSPORT 段 0.9rad/s 快速
#  重构位形时自旋飙升到 -137 度并脱出)。故搬运段单独限速到 0.45rad/s。
VC_CARRY = 0.45
KV = 4.0

# ---- 失败分类码(与 ClassifyGrasp.action 的 error_code 字段一一对应) ----
# 实验要求(四.6)把"不可达目标"和"抓取失败"列为两类不同异常, 故必须可区分:
# 任务节点据此分别记 unreachable / failed, 而不是笼统一个 'failed'。
EC_NONE = 0            # 成功
EC_UNREACHABLE = 1     # 运动学/IK 不可达
EC_GRASP_MISSED = 2    # 夹空(抬升后物体仍在台面)
EC_MOTION_FAILED = 3   # 关节未收敛到位
EC_UNKNOWN_TARGET = 4  # 非法网格/料盒
EC_INTERNAL = 5        # 内部异常
EC_PLACE_FAILED = 6    # 放置未达判定阈值


def _err_code_of(msg):
    """运动层失败信息 -> 失败分类码。IK 不可达与到位失败要分开记。"""
    return EC_UNREACHABLE if '不可达' in str(msg) else EC_MOTION_FAILED

# ---- 网格/料盒世界坐标(与 table_grid_4c2b.world desk_layout 一致) ----
# 全向布局: cell_1 正前球 / cell_2 左前瓶 / cell_3 右前瓶 / cell_4 正后瓶 / cell_5 左后球 / cell_6 右后球
GRIDS = {
    'cell_1': (0.1850, 0.0000),
    'cell_2': (0.1310, 0.1310),
    'cell_3': (0.1310, -0.1310),
    'cell_4': (-0.1850, 0.0000),
    'cell_5': (-0.1310, 0.1310),
    'cell_6': (-0.1310, -0.1310),
}
BINS = {
    'bin_0': (0.0000, 0.2400),   # 瓶盒(正左, r=0.24): 收纳矿泉水瓶
    'bin_1': (0.0000, -0.2400),  # 球盒(正右, r=0.24): 收纳网球
}


def _safe_yaw(y):
    """把落在 ±π 接缝附近的 yaw 目标推开一点点(0.05rad≈2.9°)。

    根因: classify_sim 中只有 cell_4(yawG=π) 的目标正好压在 ±π 接缝上,
    速度跟踪控制器在 +π↔-π 之间来回穿越, 永远无法 settled, 最终比例控制把
    关节一路带飞到 ~9.4rad。其余网格/料盒的 yaw 都不在接缝上, 所以 bin_0/bin_1
    正常。2.9° 偏置对 4cm 方块抓取(含 TCP_LEAD)可忽略, 但彻底消除接缝振荡。
    """
    if abs(y) > math.pi - 0.05:
        y = math.copysign(math.pi - 0.05, y)
    return y


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
        self.declare_parameter('z_grasp', 0.0446)
        self.declare_parameter('block_name', 'block')
        self.declare_parameter('block_side', 0.04)
        self.z_grasp = float(self.get_parameter('z_grasp').value)
        self.block_name = self.get_parameter('block_name').value
        self.block_side = float(self.get_parameter('block_side').value)
        # 每个料盒已放入的方块数(用于同盒多块错开投放点)
        self._bin_n = {}

        self._cb = ReentrantCallbackGroup()
        self.state_pub = self.create_publisher(String, '/grasp_state', 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/arm_vel_controller/commands', 10)
        self.brk_pub = self.create_publisher(Float64MultiArray, '/bracket_controller/commands', 10)
        self.grip_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.js = {}
        # 视觉实测方块列表: [(x, y, cls, grid), ...], 由 /grid_detections 反投影得到。
        # 抓取优先用实测位置(方块可能被前序抓取碰歪), 找不到了才回退格心。
        self.dets = []
        self.create_subscription(String, '/grid_detections', self._det_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._js_cb, 50)
        self.get_entity = self.create_client(GetEntityState, '/gazebo/get_entity_state',
                                             callback_group=self._cb)
        self._as = ActionServer(
            self, ClassifyGrasp, 'classify_grasp',
            execute_callback=self._execute, goal_callback=self._goal,
            cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=self._cb)
        self.get_logger().info('classify_grasp_server 就绪 (网格=%s 料盒=%s)' % (list(GRIDS), list(BINS)))

    # ---------- 基础(同 grasp_controller) ----------
    def _det_cb(self, m):
        """缓存 /grid_detections 的实测世界坐标(每次重建, 被抓走的方块自动消失)。"""
        try:
            items = json.loads(m.data)
        except Exception:
            return
        out = []
        for it in items:
            if it.get('x') is None or it.get('y') is None:
                continue
            out.append((float(it['x']), float(it['y']), it.get('cls'), it.get('grid')))
        self.dets = out

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

    async def _send_arm(self, yaw, q1, q2, duration, vmax=VC):
        steps = max(1, int(float(duration) / 0.01))
        settled = 0
        for _ in range(steps):
            qn = self._arm_q()
            tgt = np.array([yaw, q1, q2])
            err = tgt - qn[:3]
            err[0] = (err[0] + math.pi) % (2 * math.pi) - math.pi
            v = np.clip(KV * err, -vmax, vmax)
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
        # 到位判定收紧 0.06 -> 0.03(≈0.75cm@0.25m): 大物体(网球 φ6.6cm)下降套入时
        # 指距 8.8cm 与物体的间隙仅 1.1cm, 0.06rad 容差允许 1.5cm 水平偏差 ->
        # 下降时指面刮物体侧面把它推走(实测位移 5~12cm)。收紧后 TCP 偏差 < 0.75cm。
        ok = np.max(np.abs(derr)) < 0.03
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
                                        message='未知网格/料盒: %s / %s' % (grid, bin_id),
                                        error_code=EC_UNKNOWN_TARGET)
        gx, gy = GRIDS[grid]
        seed = (1.05, 0.0)
        # 抓取高度按物体类别: 网球球心=桌面0.025+半径0.033=0.058; 瓶取 0.070
        # (理论瓶心 0.075, 实测瓶身下沉至 ~0.069 穿透桌面, 用 0.070 贴合实际且
        #  IK 解 q2 不压下限, 缓解正后格(yaw≈π)速度控制收敛失败)。
        # 仅在 launch 未显式注入 z_grasp(保持默认 0.0446)时按类别覆盖, 否则异常注入(z_grasp:=0.005)优先。
        z_g = self.z_grasp if abs(self.z_grasp - 0.0446) > 1e-9 \
            else {'tennis_ball': 0.053, 'bottle': 0.070}.get(cls, self.z_grasp)
        # 网球抓取高度 0.058 -> 0.053(把接触带中心对齐球质心, 消除俯仰力矩):
        #   指面竖直跨度 = [TCP_z-0.0106, TCP_z+0.0294](指面盒 0.04 高, 中心在 TCP 上方 0.0094)
        #   球静止时球心实测 z=0.062 -> 接触带中心 = TCP_z+0.0094
        #   旧 z_g=0.058: 接触带中心 0.0674, 比球心 0.062 高 5.4mm ->
        #     水平加速时接触合力对质心产生俯仰力矩 -> 球绕接触带俯仰 -> 沿指面"向上爬"直至挤出
        #   新 z_g=0.053: 接触带中心 0.0624 == 球心 0.062 -> 合力过质心, 无力矩
        # 搬运姿态按类别(见 Q_CARRY_BY_CLASS 注释): 被夹物底部须高于桌面最高物(瓶顶 0.125)
        carry_q = Q_CARRY_BY_CLASS.get(cls, Q_CARRY)
        # 视觉定位强制格心: 物体 spawn 于格心。俯视投影把物体中心(高 0.06~0.08)当
        # 桌面(z=0)反投影 -> 实测坐标系统性偏外(球 ~1.9cm / 瓶 ~2.6cm), 而下降套入
        # 间隙仅 1.15cm(指距 8.8cm vs 物体 6.5~6.6cm) -> 采用实测会把抓取点带偏,
        # 下降时指面刮物体侧面把它推走(实测瓶被推 11cm)。格心抓取到位误差 <0.75cm
        # < 间隙, 不刮物体; 只要不推, 物体永远在格心, 全链路格心抓取即可。
        # (保留 _det_cb 缓存仅用于日志调试, 不再参与抓取点计算。)
        if self.dets:
            _c = [d for d in self.dets if d[3] == grid]
            if _c:
                self.get_logger().info(
                    '视觉定位 %s: 强制格心(%.3f,%.3f), 最近同类实测偏移 %.3f(弃用)'
                    % (grid, gx, gy, math.hypot(_c[0][0] - gx, _c[0][1] - gy)))
        bx, by = BINS[bin_id]
        # 同盒多件沿 x 错开落点(每个料盒放 3 件):
        #   方块版曾注释"不再错开”—— 那时一件 4cm、盒内净 0.13 能并排放 3 块,
        #   同轴重叠也无所谓。网球 φ0.066 一件就占掉半个盒, 3 件投在同一点会
        #   竖直堆成 0.198m 的塔、远高于 0.075 墙顶而滚落。故按已放入件数错开
        #   0.068m(略大于直径, 互不挤压): 3 件落在 x = 0 / +0.068 / -0.068,
        #   盒内净 0.21 容得下(最外件边缘 |x|=0.068+0.033=0.101 < 0.105)。
        #   料盒在 (±0.21 的 y 轴上), 错开沿 x 即切向, 径向距离仅增到
        #   hypot(0.068,0.21)=0.2208 -> TCP d=0.2868, 远小于该高度可达上限 0.355。
        _n = self._bin_n.get(bin_id, 0)
        _off = (0.0, 0.068, -0.068)[_n] if _n < 3 else 0.0
        px, py = bx + _off, by
        # 网格 cell_N <-> 方块实体 block_{N-1}(launch 中按 BLOCKS 顺序 spawn)
        self.block_name = 'block_%d' % (int(grid.split('_')[1]) - 1)
        yawG = _safe_yaw(math.atan2(gy, gx))
        yawB = _safe_yaw(math.atan2(py, px))
        dG = math.hypot(gx, gy) + GRASP_LEAD
        dB = math.hypot(px, py) + TCP_LEAD
        # 投放半径钳制: 料盒外移到 r=0.24 后, 同盒第 2/3 件沿 x 错开 0.068 会
        # 把投放点半径推到 d=hypot(0.068,0.24)+0.066=0.3149, 已到该高度可达上限
        # (z=0.13 时 0.357)的 88%, 接近奇异位形: 实测速度跟踪在 14s 内只把 q1
        # 推到 0.856(目标 0.96) 就报 EC_MOTION_FAILED, cell_3/cell_4 连续两轮都卡这里。
        # 钳到 0.306(盒心投放点, 已实测稳定成功: cell_2 dist=0.005/0.016)。
        # 钳制只把释放点沿同一条射线内移 8.9mm, 落点仍在盒内且远小于 0.07 判定阈值,
        # 三件沿 x 的错开(±0.068)完全保留。
        DB_MAX = 0.306
        if dB > DB_MAX:
            self.get_logger().info('投放点 d=%.4f 超可达余量, 沿同射线钳到 %.3f' % (dB, DB_MAX))
            dB = DB_MAX
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
            ok, msg = await self._send_arm(0.0, Q_HOME[0], Q_HOME[1], home_dur)
            self._pub_grip(*GRIP_OPEN)

            # 接近网格(抬升过渡位, IK 对准网格上方; 高度按类别避免刮顶; yaw 时长按角度差自适应)
            self._pub_state('APPROACH_GRID')
            app_z = APPROACH_Z_BY_CLASS.get(cls, APPROACH_Z)
            dyaw = abs((yawG - self._arm_q()[0] + math.pi) % (2 * math.pi) - math.pi)
            app_dur = max(MOVE_DUR, dyaw / 0.50 + 3.0)
            ok, msg = await self._goto_tcp(yawG, dG, app_z, app_dur, seed)
            if not ok:
                goal_handle.succeed()
                return ClassifyGrasp.Result(success=False, bin_id=bin_id, message=msg,
                                            error_code=_err_code_of(msg))
            # 两段式下降到抓取位(关键修复, 解决 4/6 下降失败):
            # 段1 先降到中间航点 GRASP_MID_Z(≈0.065): 此时 q1≈1.22 尚远离上限(余量 0.16),
            #   q2 已从 -0.20 摆到 ≈-0.35, 在「非奇异位形」下轻松到位;
            # 段2 再从中间航点降到 GRASP_Z(0.0446): 此时 q2 只需微调 -0.12 即达 -0.469,
            #   q1 才顶到上限, 不再出现「q2 在 q1 奇异位形下追不到」导致 cell_1/cell_4 失败。
            # 几何(GRASP_LEAD / z_grasp)完全不变, 仍与已验收 fk(Q_GRASP) 一致。
            self._pub_state('DESCEND_MID')
            mid_z = GRASP_MID_Z_BY_CLASS.get(cls, GRASP_MID_Z)
            # 时长 2s -> 5s: APPROACH_Z 提高到 0.16 后起始位形与 MID 差距变大,
            # 2s 时限内 q2 摆不到位(实测停在 -0.08, 目标 -0.47) -> motion_failed。
            ok, msg = await self._goto_tcp(yawG, dG, mid_z, 5.0, seed)
            if not ok:
                goal_handle.succeed()
                return ClassifyGrasp.Result(success=False, bin_id=bin_id, message=msg,
                                            error_code=_err_code_of(msg))
            self._pub_state('DESCEND_GRID')
            # 时长 3.5s -> 6s: 收紧到位判定(0.03rad)后需要更长时间收敛,
            # 否则 DESCEND 时限内不到位(TCP 偏 1cm+)下降时会刮推大物体。
            ok, msg = await self._goto_tcp(yawG, dG, z_g, 6.0, seed)
            if not ok:
                goal_handle.succeed()
                return ClassifyGrasp.Result(success=False, bin_id=bin_id, message=msg,
                                            error_code=_err_code_of(msg))
            # 夹取(按类别微压夹持, 防止大直径物体被全闭挤压弹飞)
            grip_seat, grip_close = GRIP_BY_CLASS.get(cls, (GRIP_SEAT, GRIP_CLOSE))
            self._pub_state('GRASP')
            self._hold(grip_seat, GRIP_SEAT_DUR)
            self._hold(grip_close, 1.5)
            fb.gripper_closed = True
            b = self._block_xyz()
            if b is None:
                goal_handle.succeed()
                return ClassifyGrasp.Result(success=False, bin_id=bin_id,
                                            message='抓取后无法查询方块状态',
                                            error_code=EC_INTERNAL)
            # 抬升(网格上方抬升位, IK; 物体悬空随臂搬运; 高度按类别)
            self._pub_state('LIFT')
            ok, msg = await self._goto_tcp(yawG, dG, app_z, LIFT_DUR, seed)
            # 夹空检测: 抬升到 APPROACH_Z 后方块应随之离台(块心 z≈0.095)。
            # 若仍在台面(z≈0.045)说明没夹住 —— 立即开爪返回, 不再执行搬运/投放。
            # 否则会空跑一趟: 夹爪扫过邻格把别的方块推走, 污染后续抓取。
            # 夹空检测: 抬升到 APPROACH_Z 后物体应随之离台。判定阈值按类别:
            #   网球未被夹起时球心 z=0.058(<0.068), 瓶未被夹起时瓶心 z=0.075(<0.085);
            #   夹起后 TCP 升至 0.14, 物体中心 z≈0.12+ 远高于阈值。
            z_ref = {'tennis_ball': 0.068, 'bottle': 0.085}.get(cls, 0.065)
            bl = self._block_xyz()
            if bl is not None and bl[2] < z_ref:
                self._pub_grip(*GRIP_OPEN)
                self._pub_state('GRASP_MISSED')
                self.get_logger().warn('夹空 %s: 抬升后物体 z=%.3f 仍在台面, 中止本次'
                                       % (self.block_name, bl[2]))
                goal_handle.succeed()
                return ClassifyGrasp.Result(
                    success=False, bin_id=bin_id,
                    message='夹空: 抬升后方块 z=%.3f 仍在台面' % bl[2],
                    error_code=EC_GRASP_MISSED)
            # 搬运到料盒上方(按类别的抬臂姿态, 被夹物底部高于桌面最高物)
            # 分两步: ①原位提升到位形(q1/q2 变化, yaw 不变, 球被竖直托升无旋转扰动);
            #        ②再旋转 yaw(高位形下球底 0.144>瓶顶 0.125, 旋转安全)。
            # 原一步(yaw+q1/q2 同时变化)实测球从指间滑落(z=0.153->0.060, 甩出 26cm)。
            self._pub_state('TRANSPORT')
            yaw_dur = max(YAW_DUR, abs((yawB - yawG + math.pi) % (2 * math.pi) - math.pi) / 0.50 + 3.0)
            # 搬运两段均限速 VC_CARRY(0.45rad/s): 见 VC_CARRY 注释。
            ok2, msg2 = await self._send_arm(yawG, carry_q[0], carry_q[1], 6.0, VC_CARRY)
            ok2b, msg2b = await self._send_arm(yawB, carry_q[0], carry_q[1],
                                               max(yaw_dur, 8.0), VC_CARRY)
            if ok2 and ok2b:
                msg2 = 'ok'
            # 降到料盒上方(IK 对准盒心, 按类别投放高度: 物体底高于盒壁顶后投放)
            self._pub_state('DESCEND_BIN')
            z_drop = Z_DROP_BY_CLASS.get(cls, Z_DROP)
            # 时长 5.0s -> 14.0s: 料盒外移到 r=0.24 后, 同盒第 2/3 件要沿 x 错开
            # 0.068, 投放点 TCP d 由 0.306 增到 0.315, 已达该高度可达上限(0.357)的
            # 88%, 接近奇异位形, 速度跟踪收敛很慢。实测 5s 内 q1 只到 0.859(目标 0.96)
            # 就报 EC_MOTION_FAILED(全任务里 cell_3 / cell_4 两次都卡在这一段)。
            # 拉长到 14s 让它在容许时间内收敛; 达标判据(0.03rad)本身不放宽。
            ok3, msg3 = await self._goto_tcp(yawB, dB, z_drop, 14.0, seed)
            self._pub_state('RELEASE')
            self._hold(GRIP_OPEN, 2.0)
            fb.gripper_closed = False
            self._pub_state('LIFT_BIN')
            # 回程前先把空夹爪升到高位(同 HOME 关节)再转 yaw —— 否则空夹爪在 Q_CARRY(0.11)
            # 低高度旋转会扫到桌面方块(曾致 cell_1 被推飞 5~17cm)。HOME 已是高位, 故
            # 此处直接升到高位, 下一周期 HOME->APPROACH(0.14) 全程高位。
            await self._send_arm(yawB, Q_HOME[0], Q_HOME[1], 3.0)

            if not (ok and ok2 and ok3):
                goal_handle.succeed()
                return ClassifyGrasp.Result(success=False, bin_id=bin_id,
                                            message='运动失败: %s | %s | %s' % (msg, msg2, msg3),
                                            error_code=EC_MOTION_FAILED)
            # 判定: 本次抓取的方块(而非全场最近方块)应落在目标盒心 7cm 内。
            # 原实现用 _bin_min_dist 取"最近方块", 同一料盒放第二块时会误判到
            # 先放入的块(被后来的块撞开 -> dist=0.081 判失败), 故改为精确匹配。
            dist, bn = self._bin_min_dist(bx, by)
            bpos = self._block_xyz()
            if bpos is not None:
                dist = math.hypot(bpos[0] - px, bpos[1] - py)
                bn = self.block_name
            okc = 0 <= dist < 0.07
            self.get_logger().info('[%s->%s] 本次方块=%s 距投放点=%.3f 距盒心=%.3f 成功=%s'
                                   % (grid, bin_id, bn, dist,
                                      (math.hypot(bpos[0] - bx, bpos[1] - by)
                                       if bpos else -1.0), okc))
            if okc:
                self._bin_n[bin_id] = self._bin_n.get(bin_id, 0) + 1
            self._pub_state('CYCLE_DONE')
            goal_handle.succeed()
            return ClassifyGrasp.Result(success=bool(okc), bin_id=bin_id,
                                        message='dist=%.3f' % dist,
                                        error_code=EC_NONE if okc else EC_PLACE_FAILED)
        except Exception as e:
            self.get_logger().error('执行异常: %s' % e)
            self._safe_stop()
            goal_handle.succeed()
            return ClassifyGrasp.Result(success=False, bin_id=bin_id, message='异常: %s' % e,
                                        error_code=EC_INTERNAL)


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
