#!/usr/bin/env python3
"""自包含 FK/IK 求解器（无 ROS 依赖）。

解析 URDF，用 numpy 做正运动学(FK)，用 scipy 做阻尼最小二乘逆运动学(IK)。
用于定点抓取：给定 tool0 在 table_link 下的目标位姿，求 joint1..joint6 关节角。

设计目标：
- 与具体机械臂几何无关：换 URDF 即可，任务逻辑不变（满足实验"实机只换参数"约束）。
- 离线可跑（headless），不需要 Gazebo / MoveIt。
"""
import numpy as np
from lxml import etree


def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _origin_matrix(xyz, rpy):
    x, y, z = (xyz if xyz is not None else [0, 0, 0])
    rx, ry, rz = (rpy if rpy is not None else [0, 0, 0])
    R = _rot_z(rz) @ _rot_y(ry) @ _rot_x(rx)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def _str3(v):
    return [float(x) for x in (v.split() if isinstance(v, str) else v)]


class RobotModel:
    def __init__(self, urdf_text, root_link='table_link', tip_link='tool0'):
        self.root_link = root_link
        self.tip_link = tip_link
        self.joints = {}          # name -> dict
        self.children = {}        # link -> (joint_name, child_link)
        self._parse(urdf_text)
        self.chain = self._build_chain()
        # IK 变量 = 链上的可动关节（revolute/prismatic），按链序
        self.var_joints = [j for j in self.chain if self.joints[j]['type'] in ('revolute', 'prismatic')]
        self.lower = np.array([self.joints[j]['lower'] for j in self.var_joints], dtype=float)
        self.upper = np.array([self.joints[j]['upper'] for j in self.var_joints], dtype=float)

    def _parse(self, text):
        root = etree.fromstring(text.encode() if isinstance(text, str) else text)
        for j in root.findall('joint'):
            name = j.get('name')
            typ = j.get('type')
            parent = j.find('parent').get('link')
            child = j.find('child').get('link')
            origin = j.find('origin')
            xyz = _str3(origin.get('xyz', '0 0 0')) if origin is not None else [0, 0, 0]
            rpy = _str3(origin.get('rpy', '0 0 0')) if origin is not None else [0, 0, 0]
            axis_el = j.find('axis')
            axis = _str3(axis_el.get('xyz', '1 0 0')) if axis_el is not None else [1, 0, 0]
            axis = np.array(axis, dtype=float)
            n = np.linalg.norm(axis)
            if n > 0:
                axis = axis / n
            lim = j.find('limit')
            if lim is not None:
                lower = float(lim.get('lower', '-3.14159'))
                upper = float(lim.get('upper', '3.14159'))
            else:
                lower, upper = -np.pi, np.pi
            self.joints[name] = dict(type=typ, parent=parent, child=child,
                                     xyz=xyz, rpy=rpy, axis=axis,
                                     lower=lower, upper=upper)
            self.children.setdefault(parent, []).append((name, child))

    def _build_chain(self):
        """从 root 到 tip 的关节顺序（含 fixed）。"""
        chain = []
        link = self.root_link
        visited = set()
        while link != self.tip_link and link in self.children:
            # 取第一条能通往 tip 的子链
            advanced = False
            for jname, clink in self.children[link]:
                if jname in visited:
                    continue
                # 检查该子链能否到达 tip
                if self._reaches(clink, self.tip_link):
                    chain.append(jname)
                    visited.add(jname)
                    link = clink
                    advanced = True
                    break
            if not advanced:
                break
        return chain

    def _reaches(self, start, target):
        if start == target:
            return True
        for jname, clink in self.children.get(start, []):
            if self._reaches(clink, target):
                return True
        return False

    def fk(self, q):
        """q: dict {joint_name: position}（仅需要链上可动关节）。返回 tip 相对 root 的 4x4。"""
        T = np.eye(4)
        link = self.root_link
        for jname in self.chain:
            j = self.joints[jname]
            Tj = _origin_matrix(j['xyz'], j['rpy'])
            if j['type'] == 'revolute':
                a = float(q.get(jname, 0.0))
                Tj[:3, :3] = Tj[:3, :3] @ _axis_rot(j['axis'], a)
            elif j['type'] == 'prismatic':
                d = float(q.get(jname, 0.0))
                Tj[:3, 3] = Tj[:3, 3] + j['axis'] * d
            T = T @ Tj
            link = j['child']
        return T

    def fk_array(self, q_arr):
        q = {j: float(v) for j, v in zip(self.var_joints, q_arr)}
        return self.fk(q)

    def pose_error(self, q_arr, target):
        """6D 位姿误差：前3=位置，后3=旋转向量（轴角）。"""
        T = self.fk_array(q_arr)
        pos = T[:3, 3]
        R = T[:3, :3]
        ep = pos - target[:3, 3]
        # 目标姿态
        Rt = target[:3, :3]
        Re = Rt.T @ R
        # 轴角
        ang = np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))
        if ang < 1e-8:
            ea = np.zeros(3)
        else:
            axis = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]]) / (2 * np.sin(ang))
            ea = axis * ang
        return np.concatenate([ep, ea])


def _axis_rot(axis, a):
    axis = np.asarray(axis, dtype=float)
    x, y, z = axis
    c, s = np.cos(a), np.sin(a)
    C = 1 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def solve_ik(model, target, q0=None, retries=8):
    """牛顿/阻尼最小二乘 IK。返回 (q_arr, success, residual)。"""
    from scipy.optimize import least_squares
    if q0 is None:
        q0 = np.zeros(len(model.var_joints))
    best = None
    best_cost = np.inf
    for i in range(retries):
        if i == 0:
            q_seed = np.array(q0, dtype=float)
        else:
            # 随机种子在关节限位内
            q_seed = model.lower + (model.upper - model.lower) * np.random.rand(len(model.var_joints))
        try:
            res = least_squares(lambda q: model.pose_error(q, target), q_seed,
                                bounds=(model.lower, model.upper), max_nfev=400)
        except Exception as e:
            continue
        cost = float(np.sum(res.fun ** 2))
        if cost < best_cost:
            best_cost = cost
            best = res.x
        if cost < 1e-6:
            break
    success = best_cost < 1e-4
    return best, success, best_cost
