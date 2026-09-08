#!/usr/bin/env python3
"""离线标定：用 IK 求定点抓取各 waypoint 的关节角，并 FK 回放验证。

不依赖 ROS/Gazebo，headless 跑。验证 arm_grasp URDF 能否到达桌面 A/B 点。
"""
import subprocess
import sys
import numpy as np
sys.path.insert(0, '/home/underwater/arm_grasp_ws/src/arm_grasp_sim/scripts')
from ik_solver import RobotModel, solve_ik, _rot_x

URDF = '/home/underwater/arm_grasp_ws/src/arm_grasp_sim/urdf/arm_grasp.urdf.xacro'
CONFIG = '/home/underwater/arm_grasp_ws/src/arm_grasp_sim/config/controllers.yaml'


def resolve_urdf():
    out = subprocess.run(['xacro', URDF, 'config_path:=' + CONFIG],
                         capture_output=True, text=True)
    if out.returncode != 0:
        raise RuntimeError('xacro failed:\n' + out.stderr)
    return out.stdout


def target_pose(x, y, z, flip=True):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    if flip:
        T[:3, :3] = _rot_x(np.pi)   # tool0 +Z 指向下(-Z)，平行爪从上往下夹
    return T


def main():
    model = RobotModel(resolve_urdf(), root_link='table_link', tip_link='tool0')
    print('var_joints =', model.var_joints)
    print('limits lower =', np.round(model.lower, 2))
    print('limits upper =', np.round(model.upper, 2))

    A = (0.15, 0.0)
    B = (-0.15, 0.0)
    Z_APPROACH = 0.20
    Z_GRASP = 0.095      # 见 ik_solver 注释：爪指下探包住 0.04 方块
    Z_LIFT = 0.26

    targets = {
        'A_APPROACH': target_pose(A[0], A[1], Z_APPROACH),
        'A_GRASP':    target_pose(A[0], A[1], Z_GRASP),
        'LIFT':       target_pose(A[0], A[1], Z_LIFT),
        'B_APPROACH': target_pose(B[0], B[1], Z_APPROACH),
        'B_PLACE':    target_pose(B[0], B[1], Z_GRASP),
        'LIFT2':      target_pose(B[0], B[1], Z_LIFT),
    }

    results = {}
    for name, Tgt in targets.items():
        q, ok, cost = solve_ik(model, Tgt)
        T = model.fk_array(q)
        pos = T[:3, 3]
        # 爪指向（tool0 +Z 在 table 下的方向）
        zdir = T[:3, 2]
        print('\n=== %s ===' % name)
        print('  success=%s  cost=%.2e' % (ok, cost))
        print('  joint =', np.round(q, 4).tolist())
        print('  fk_pos = (%.3f, %.3f, %.3f)  target = (%.3f, %.3f, %.3f)'
              % (pos[0], pos[1], pos[2], Tgt[0, 3], Tgt[1, 3], Tgt[2, 3]))
        print('  tool0 +Z dir = (%.2f, %.2f, %.2f)  [期望≈(0,0,-1)]' % (zdir[0], zdir[1], zdir[2]))
        results[name] = q

    # 写出可给 grasping 节点用的关节角（CSV 顺序：joint1..joint6）
    import json
    out = {k: [float(v) for v in q] for k, q in results.items()}
    with open('/tmp/waypoints_ik.json', 'w') as f:
        json.dump(out, f, indent=2)
    print('\n[ok] wrote /tmp/waypoints_ik.json')


if __name__ == '__main__':
    main()
