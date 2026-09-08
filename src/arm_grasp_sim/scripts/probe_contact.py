#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EP 抓取实测(平行直进夹爪): 对称合拢 + 真实抬升测试。
用 Gazebo /gazebo/link_states 的真实 link 位姿计算 指尖 pad AABB 与 方块 AABB 重叠,
并确认爪关节是否真的合拢; 然后抬升, 看方块 z 是否升高。"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from scipy.spatial.transform import Rotation

TABLE_TOP = 0.025
ARM1_X = 0.0103961
ARM1_Z = 0.03465 + 0.0906477 + 0.030741
A1X, A1Z = 0.0018704, 0.1210238
A2X, A2Z = 0.1058557, -0.0561093
TCPX, TCPZ = 0.124 + 0.0002793, -0.039 + 0.0001815
TCP_LEAD = 0.076          # 让 pad 中心对齐方块中心 (pad 相对 gripper_link X=0.048)
Q1_LIM = (-0.274, 1.384)
Q2_LIM = (-1.25, 0.40)
GRIP_OPEN = (0.0, 0.0)
GRIP_SEAT = (5.0, 5.0)
GRIP_CLOSE = (15.0, 15.0)
BRK_OFF = 0.15   # 括架俯仰偏置: 压低 pad 夹持线对方块中心
KV = 4.0
VC = 0.9
ARM_JOINTS = ['chassis_yaw_joint', 'arm_1_joint', 'arm_2_joint', 'endpoint_bracket_joint']
GRIP_JOINTS = ['gripper_left_joint', 'gripper_right_joint']

# pad 碰撞盒在 finger link 内的局部中心 与 半尺寸
PAD_LOCAL_L = np.array([0.0, -0.05, 0.0])
PAD_LOCAL_R = np.array([0.0, 0.05, 0.0])
PAD_HALF = np.array([0.0225, 0.006, 0.020])   # box size 0.045 x 0.012 x 0.05
BLOCK_HALF = np.array([0.02, 0.02, 0.02])


def fk_dz(q1, q2):
    th1, th2 = q1, q1 + q2
    d = (ARM1_X + A1X * math.cos(th1) + A1Z * math.sin(th1)
         + A2X * math.cos(th2) + A2Z * math.sin(th2) + TCPX)
    z = (TABLE_TOP + ARM1_Z - A1X * math.sin(th1) + A1Z * math.cos(th1)
         - A2X * math.sin(th2) + A2Z * math.cos(th2) + TCPZ)
    return d, z


def ik_dz(d, z, seed=(1.0, 0.0)):
    from scipy.optimize import least_squares
    def res(x):
        fd, fz = fk_dz(x[0], x[1])
        return [fd - d, fz - z]
    lo = [Q1_LIM[0], Q2_LIM[0]]
    hi = [Q1_LIM[1], Q2_LIM[1]]
    best = None
    for s0 in [seed, (1.1, 0.0), (1.2, -0.1), (0.9, 0.2), (1.3, -0.2)]:
        r = least_squares(res, np.array(s0, dtype=float), bounds=(lo, hi))
        if best is None or r.cost < best.cost:
            best = r
        if r.cost < 1e-10:
            break
    return float(best.x[0]), float(best.x[1])


def aabb_of(center_local, half, pos, quat):
    R = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
    corners = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                lc = center_local + np.array([sx, sy, sz]) * half
                wc = np.array([pos.x, pos.y, pos.z]) + R @ lc
                corners.append(wc)
    corners = np.array(corners)
    return corners.min(axis=0), corners.max(axis=0)


class Probe(Node):
    def __init__(self):
        super().__init__('probe_contact')
        self.cb = ReentrantCallbackGroup()
        self.js = {}
        self.link_poses = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 50)
        self.create_subscription(LinkStates, '/gazebo/link_states', self._ls_cb, 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/arm_vel_controller/commands', 10)
        self.brk_pub = self.create_publisher(Float64MultiArray, '/bracket_controller/commands', 10)
        self.grip_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)

    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))

    def _ls_cb(self, m):
        for nm, ps in zip(m.name, m.pose):
            self.link_poses[nm] = (ps.position, ps.orientation)

    def _arm_q(self):
        try:
            return np.array([float(self.js[n]) for n in ARM_JOINTS])
        except Exception:
            return np.zeros(4)

    def _grip_q(self):
        try:
            return [float(self.js.get(n, 0.0)) for n in GRIP_JOINTS]
        except Exception:
            return [None, None]

    def _pub_vel(self, v3):
        m = Float64MultiArray(); m.data = [float(x) for x in v3]; self.vel_pub.publish(m)

    def _pub_bracket(self, q):
        m = Float64MultiArray(); m.data = [float(q)]; self.brk_pub.publish(m)

    def _pub_grip(self, l, r):
        m = Float64MultiArray(); m.data = [float(l), float(r)]; self.grip_pub.publish(m)

    def _send_arm(self, yaw, q1, q2, duration, vc=None):
        steps = max(1, int(float(duration) / 0.01))
        settled = 0
        cap = VC if vc is None else float(vc)
        for _ in range(steps):
            qn = self._arm_q()
            tgt = np.array([yaw, q1, q2])
            err = tgt - qn[:3]
            v = np.clip(KV * err, -cap, cap)
            self._pub_bracket(-(qn[1] + qn[2]) + BRK_OFF)
            if np.max(np.abs(err)) < 0.01:
                settled += 1; v = np.zeros(3)
            else:
                settled = 0
            self._pub_vel(v)
            time.sleep(0.01)
            if settled >= 50:
                break
        self._pub_vel(np.zeros(3))
        qn = self._arm_q()
        return np.max(np.abs(np.array([yaw, q1, q2]) - qn[:3])) < 0.03

    def _find_link(self, suffix):
        for nm, pp in self.link_poses.items():
            if nm.split('::')[-1] == suffix or nm == suffix:
                return pp
        return None

    def _report(self, tag):
        lp = self._find_link('left_finger_link')
        rp = self._find_link('right_finger_link')
        bp = self._find_link('block')
        gq = self._grip_q()
        print('\n==== %s ====' % tag)
        print('  gripper_joints L=%.4f R=%.4f (cmd close=%.2f)' % (gq[0], gq[1], GRIP_CLOSE[0]))
        if lp is None or rp is None:
            print('  缺失 finger link! known:', list(self.link_poses.keys()))
            return None
        lmin, lmax = aabb_of(PAD_LOCAL_L, PAD_HALF, lp[0], lp[1])
        rmin, rmax = aabb_of(PAD_LOCAL_R, PAD_HALF, rp[0], rp[1])
        pmin = np.minimum(lmin, rmin); pmax = np.maximum(lmax, rmax)
        print('  left  pad  Y[%.4f,%.4f] X[%.4f,%.4f] Z[%.4f,%.4f]'
              % (lmin[1], lmax[1], lmin[0], lmax[0], lmin[2], lmax[2]))
        print('  right pad  Y[%.4f,%.4f] X[%.4f,%.4f] Z[%.4f,%.4f]'
              % (rmin[1], rmax[1], rmin[0], rmax[0], rmin[2], rmax[2]))
        print('  pad 包络   X[%.4f,%.4f] Y[%.4f,%.4f] Z[%.4f,%.4f]'
              % (pmin[0], pmax[0], pmin[1], pmax[1], pmin[2], pmax[2]))
        if bp is not None:
            bmin, bmax = aabb_of(np.zeros(3), BLOCK_HALF, bp[0], bp[1])
            print('  block     X[%.4f,%.4f] Y[%.4f,%.4f] Z[%.4f,%.4f]'
                  % (bmin[0], bmax[0], bmin[1], bmax[1], bmin[2], bmax[2]))
            ov = np.minimum(pmax, bmax) - np.maximum(pmin, bmin)
            print('  重叠量(正=重叠) X=%.4f Y=%.4f Z=%.4f' % (ov[0], ov[1], ov[2]))
            print('  方块中心 (%.4f, %.4f, %.4f)' % (bp[0].x, bp[0].y, bp[0].z))
            return bp[0]
        else:
            print('  (block link 未找到, 跳过重叠计算)')
            return None


def main():
    rclpy.init()
    node = Probe()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    import threading
    threading.Thread(target=exe.spin, daemon=True).start()

    for _ in range(100):
        if node.link_poses:
            break
        time.sleep(0.1)
    print('link_states 就绪, %d links' % len(node.link_poses))

    yawA = 0.0
    dA = 0.20
    dG = dA + TCP_LEAD
    z_grasp = 0.035
    z_lift = 0.072
    q1, q2 = ik_dz(dG, z_grasp)
    print('IK: dG=%.4f z=%.4f -> q1=%.4f q2=%.4f' % (dG, z_grasp, q1, q2))

    node._pub_grip(*GRIP_OPEN)
    node._send_arm(0.0, 0.0, 0.0, 3.0)
    time.sleep(1.0)
    node._send_arm(yawA, q1, q2, 5.0)
    time.sleep(1.0)
    node._report('张开到位')
    time.sleep(0.5)

    node._pub_grip(*GRIP_CLOSE)
    time.sleep(1.5)
    b_close = node._report('对称合拢')
    time.sleep(0.5)

    print('\n==== 抬升测试 (TCP z: %.4f -> %.4f) ====' % (z_grasp, z_lift))
    b0 = node._find_link("block")
    z0 = b0[0].z if b0 else 0.045
    node._send_arm(yawA, 1.20, q2, 8.0, vc=0.4)   # 近垂直慢抬: 降 q1, q2 不动
    traj = []
    for _ in range(15):
        node._send_arm(yawA, 1.20, q2, 0.4, vc=0.4)   # 保持位姿防手臂下沉
        bp = node._find_link('block')
        if bp:
            e = Rotation.from_quat([bp[1].x, bp[1].y, bp[1].z, bp[1].w]).as_euler('xyz')
            gq = node._grip_q()
            lp2 = node._find_link('left_finger_link')
            pz = lp2[0].z if lp2 else float('nan')
            traj.append((bp[0].x, bp[0].y, bp[0].z))
            print('    t pos(%.4f,%.4f,%.4f) pitch=%.3f roll=%.3f gqL=%.4f gqR=%.4f padZ=%.4f'
                  % (bp[0].x, bp[0].y, bp[0].z, e[1], e[0], gq[0], gq[1], pz))
    b_end = node._report('抬升后')
    if traj:
        arr = np.array(traj)
        print('  方块轨迹(抬升中每0.4s):')
        for i, p in enumerate(traj):
            print('    t%d (%.4f,%.4f,%.4f)' % (i, p[0], p[1], p[2]))
        lifted = arr[-1][2] - z0
        print('  抬升量 dZ=%.4f  (>=0.02 视为抓起)' % lifted)
        print('  VERDICT: %s' % ('抓起' if lifted > 0.02 else '未抓起/滑脱'))

    node._pub_grip(*GRIP_OPEN)
    node._send_arm(0.0, 0.0, 0.0, 3.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
