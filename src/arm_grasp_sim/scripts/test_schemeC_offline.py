#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""方案 C 离线单测: 合成俯视相机图(白底 + 6 个四色方块于网格像素中心),
验证 process_frame 能检出 6 框并正确分类, 且不误检白色料盒/大色块。
无需 ROS / Gazebo, 纯 cv2。运行: python3 test_schemeC_offline.py
"""
import math
import sys
import numpy as np
import cv2

# 复用真实节点的纯函数
sys.path.insert(0, '/home/underwater/arm_grasp_ws/src/arm_grasp_sim/scripts')
from vision_classifier import process_frame  # noqa: E402

# 与 grid_mapper 一致的投影(cx=400,cy=320,fx=fy=779.5)
W, H, hfov, cam_h = 800, 640, 0.95, 1.0
fx = fy = (W / 2) / math.tan(hfov / 2)
cx, cy = W / 2.0, H / 2.0


def w2p(x, y):
    return int(round(cx - fx * y / cam_h)), int(round(cy - fy * x / cam_h))


# 网格 -> (世界坐标, 期望颜色)
CELLS = {
    'cell_1': (0.190, 0.000, 'green_block'),
    'cell_2': (0.095, 0.1645, 'yellow_block'),
    'cell_3': (-0.095, 0.1645, 'green_block'),
    'cell_4': (-0.190, 0.000, 'red_block'),
    'cell_5': (0.1645, 0.0950, 'blue_block'),
    'cell_6': (-0.1645, 0.0950, 'yellow_block'),
}
COL_BGR = {
    'green_block': (0, 200, 0),
    'yellow_block': (0, 215, 255),
    'red_block': (0, 0, 255),
    'blue_block': (255, 0, 0),
}

img = np.full((H, W, 3), 240, np.uint8)  # 白底(浅灰, S 低 -> 不误检)
# 画 6 个方块
for name, (x, y, cls) in CELLS.items():
    u, v = w2p(x, y)
    c = COL_BGR[cls]
    cv2.rectangle(img, (u - 17, v - 17), (u + 17, v + 17), c, -1)
# 故意画一个"大料盒"色块(0.13m -> ~100px, 面积远超 max_area) 应被排除
cv2.rectangle(img, (60, 60), (160, 160), (0, 200, 0), -1)

dets, dbg = process_frame(img, min_area=80, max_area=4000)
print('检测到框数: %d' % len(dets))
for d in dets:
    print('  class=%s score=%.3f cx=%.0f cy=%.0f w=%.0f h=%.0f'
          % (d['class_id'], d['score'], d['cx'], d['cy'], d['w'], d['h']))

# 校验: 每检测映射到最近网格, 类别须匹配
cell_px = {n: w2p(x, y) for n, (x, y, _) in CELLS.items()}
expected = {n: cls for n, (_, _, cls) in CELLS.items()}
ok = True
matched_cells = set()
for d in dets:
    # 找最近网格中心
    best, bd = None, 1e9
    for n, (pu, pv) in cell_px.items():
        dist = math.hypot(d['cx'] - pu, d['cy'] - pv)
        if dist < bd:
            best, bd = n, dist
    got = d['class_id']
    exp = expected[best]
    matched_cells.add(best)
    flag = 'OK' if got == exp else 'FAIL'
    if got != exp:
        ok = False
    print('  网格 %s 期望 %s 实得 %s  dist=%.0f  [%s]' % (best, exp, got, bd, flag))

# 应检出全部 6 个网格, 且误检的大料盒被排除
missing = set(CELLS) - matched_cells
if missing:
    ok = False
    print('  漏检网格: %s' % missing)

# 不应有蓝色被错分, 且大料盒(面积~10000)不应出现
big = [d for d in dets if d['w'] * d['h'] > 4000]
if big:
    ok = False
    print('  误检大色块(应被 max_area 排除): %s' % big)

print('\n结果: %s' % ('PASS ✅' if ok else 'FAIL ❌'))
sys.exit(0 if ok else 1)
