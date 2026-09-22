#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验三 真机网格标定工具。

为什么用单应(homography)而不是"测 hfov + 相机高度":
  针孔模型要求 ① 严格俯视 ② 已知镜头到桌面高度 ③ 已知传感器 hfov —— 三个前提
  真机上都难保证(云台有俯仰角、高度要爬上去量、hfov 靠猜)。而"桌面是平面"这一条
  天然成立, 所以 4 个已知点就能解出 世界(x,y) -> 像素(u,v) 的单应矩阵, 一致性好、
  误差可量化, 且相机稍微歪一点也没关系。

两种用法
--------
1) 看当前标定对不对(画在实拍图上, 存 PNG 供人眼核对):
     python3 calib_real_grid.py check --topic /robomaster/camera/image_color
     python3 calib_real_grid.py check --topic ... --cells cells.json \
             --homography "[...9 个数...]"

2) 由已知对应点拟合单应, 输出可直接粘进 classify_real.yaml 的一行:
     python3 calib_real_grid.py fit points.json

对应点文件 points.json (世界坐标单位=米, 臂基座系 x 向前 y 向左; 像素用图上看):
   {
     "points": [
       {"name": "cell_1", "world": [0.190,  0.000], "pixel": [412, 301]},
       {"name": "cell_2", "world": [0.095,  0.1645], "pixel": [498, 244]},
       {"name": "cell_6", "world": [-0.190, 0.000], "pixel": [214, 301]},
       {"name": "bin_0",  "world": [0.20,  -0.25],  "pixel": [560, 420]}
     ]
   }
   至少 4 点, 且别全在同一直线上(否则单应退化)。
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np


# ---------------------------------------------------------------- 单应拟合
def fit_homography(pairs):
    """归一化 DLT: pairs=[(world_xy, pixel_uv)] -> H(3x3), 使 pixel ~ H @ world。"""
    src = np.asarray([p[0] for p in pairs], np.float64)
    dst = np.asarray([p[1] for p in pairs], np.float64)
    if len(src) < 4:
        raise ValueError('至少需要 4 个对应点, 现在 %d 个' % len(src))

    def _norm(pts):
        c = pts.mean(axis=0)
        d = np.sqrt(((pts - c) ** 2).sum(axis=1)).mean()
        s = np.sqrt(2.0) / (d if d > 1e-12 else 1.0)
        T = np.array([[s, 0, -s * c[0]], [0, s, -s * c[1]], [0, 0, 1]])
        homo = np.hstack([pts, np.ones((len(pts), 1))])
        return (homo @ T.T)[:, :2], T

    s_n, T_s = _norm(src)
    d_n, T_d = _norm(dst)
    A = []
    for (x, y), (u, v) in zip(s_n, d_n):
        A.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u])
        A.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
    _, _, Vt = np.linalg.svd(np.asarray(A, np.float64))
    Hn = Vt[-1].reshape(3, 3)
    H = np.linalg.inv(T_d) @ Hn @ T_s
    return H / H[2, 2]


def residuals(H, pairs):
    """返回 (每点像素残差, 每点世界残差(米), 汇总)。"""
    try:
        Hinv = np.linalg.inv(H)
    except np.linalg.LinAlgError:
        return None, None, {}
    pr, wr = [], []
    for (wx, wy), (ux, uy) in pairs:
        p = H @ np.array([wx, wy, 1.0])
        pu, pv = p[0] / p[2], p[1] / p[2]
        pr.append(float(np.hypot(pu - ux, pv - uy)))
        q = Hinv @ np.array([ux, uy, 1.0])
        wr.append(float(np.hypot(q[0] / q[2] - wx, q[1] / q[2] - wy)))
    pr, wr = np.asarray(pr), np.asarray(wr)
    return pr, wr, {'rms_px': float(np.sqrt((pr ** 2).mean())),
                    'max_px': float(pr.max()),
                    'rms_m': float(np.sqrt((wr ** 2).mean())),
                    'max_m': float(wr.max())}


def load_points(path):
    d = json.load(open(path, encoding='utf-8'))
    pts = d['points'] if isinstance(d, dict) else d
    pairs, names = [], []
    for p in pts:
        pairs.append(((float(p['world'][0]), float(p['world'][1])),
                      (float(p['pixel'][0]), float(p['pixel'][1]))))
        names.append(p.get('name', '?'))
    return pairs, names


def cmd_fit(args):
    pairs, names = load_points(args.points)
    H = fit_homography(pairs)
    pr, wr, s = residuals(H, pairs)
    print('=' * 74)
    print('参与拟合的点 (%d 个):' % len(pairs))
    for n, (w, q), a, b in zip(names, pairs, pr, wr):
        print('  %-8s world=(%+.4f, %+.4f)  pixel=(%6.1f, %6.1f)  '
              '残差 %.2fpx / %.4f m' % (n, w[0], w[1], q[0], q[1], a, b))
    print('-' * 74)
    print('RMS = %.2f px  (max %.2f px)   |   %.2f mm  (max %.2f mm)'
          % (s['rms_px'], s['max_px'], s['rms_m'] * 1000, s['max_m'] * 1000))
    print('=' * 74)
    vals = ','.join('%.6f' % v for v in H.reshape(-1))
    print('粘进 config/classify_real.yaml 的 grid_mapper 段:')
    print("    homography: '[%s]'" % vals)
    print('=' * 74)
    if s['rms_m'] > 0.02:
        print('⚠ 世界残差 RMS > 20mm —— 单应拟合不好。请检查:')
        print('   · 世界坐标是不是量错/单位写成 mm 了')
        print('   · 像素点是不是点错格子(容易把相邻格记混)')
        print('   · 4 个点是否几乎共线(退化)')
    return 0


def cmd_check(args):
    pairs = []
    cells = None
    if args.points:
        pairs, _ = load_points(args.points)
        cells = {('cell_%d' % (i + 1)): p[0] for i, p in enumerate(pairs)}
    if args.cells:
        cells = json.load(open(args.cells, encoding='utf-8'))
    H = None
    if args.homography:
        H = np.asarray(json.loads(args.homography), np.float64).reshape(3, 3)
    elif pairs:
        H = fit_homography(pairs)
        print('[提示] 未给 --homography, 已用 %d 个对应点现拟合一份' % len(pairs))

    frame = None
    if args.image:
        frame = cv2.imread(args.image)
    if frame is None and args.topic:
        frame = grab_frame(args.topic, args.timeout)
    if frame is None:
        print('!! 没有可用图像: 请给 --image <png> 或用 --topic <图像话题>')
        return 2
    h, w = frame.shape[:2]
    print('图像 %dx%d, 映射=%s' % (w, h, '单应' if H is not None else '无(只画已知点)'))

    vis = frame.copy()
    if H is not None and cells:
        for name, xy in cells.items():
            p = H @ np.array([float(xy[0]), float(xy[1]), 1.0])
            if abs(p[2]) < 1e-12:
                continue
            u, v = int(round(p[0] / p[2])), int(round(p[1] / p[2]))
            inside = 0 <= u < w and 0 <= v < h
            col = (0, 200, 0) if inside else (0, 0, 220)
            cv2.drawMarker(vis, (u, v), col, cv2.MARKER_CROSS, 26, 2)
            cv2.circle(vis, (u, v), 18, col, 2)
            cv2.putText(vis, '%s(%d,%d)' % (name, u, v), (u + 20, v - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            print('  %-8s world=(%+.4f,%+.4f) -> pixel=(%d,%d) %s'
                  % (name, xy[0], xy[1], u, v, '' if inside else '⚠画面外'))
    if pairs:
        for (wx, wy), (ux, uy) in pairs:
            cv2.drawMarker(vis, (int(ux), int(uy)), (255, 0, 0),
                           cv2.MARKER_TILTED_CROSS, 18, 2)
    out = args.out or '/tmp/calib_check.png'
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    cv2.imwrite(out, vis)
    print('已存核对图: %s   (绿=投影在画面内, 红=超出画面; 蓝叉=你给的实际像素)' % out)
    print('核对标准: 每个绿十字必须压在对应格子的真实中心上。')
    return 0


def grab_frame(topic, timeout=10.0):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    holder = {}

    class G(Node):
        def __init__(self):
            super().__init__('calib_grab')
            self.b = CvBridge()
            self.create_subscription(Image, topic, self.cb, 10)

        def cb(self, m):
            if 'f' in holder:
                return
            try:
                holder['f'] = self.b.imgmsg_to_cv2(m, 'bgr8')
            except Exception:
                pass

    rclpy.init()
    n = G()
    t0 = time.time()
    while rclpy.ok() and 'f' not in holder and time.time() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_node()
    rclpy.shutdown()
    if 'f' not in holder:
        print('!! %s 在 %.0fs 内没有帧(话题名错了? 相机没开?)' % (topic, timeout))
    return holder.get('f')


def main():
    ap = argparse.ArgumentParser(description='实验三 真机网格标定')
    sub = ap.add_subparsers(dest='cmd', required=True)

    f = sub.add_parser('fit', help='由对应点拟合单应')
    f.add_argument('points', help='points.json')
    f.set_defaults(func=cmd_fit)

    c = sub.add_parser('check', help='把投影画到实拍图上核对')
    c.add_argument('--topic', default='')
    c.add_argument('--image', default='', help='也可直接给一张已存的 PNG')
    c.add_argument('--cells', default='', help='cells JSON 文件')
    c.add_argument('--points', default='', help='points.json(现拟合并画)')
    c.add_argument('--homography', default='', help='9 个数 JSON 数组')
    c.add_argument('--out', default='/tmp/calib_check.png')
    c.add_argument('--timeout', type=float, default=10.0)
    c.set_defaults(func=cmd_check)

    args = ap.parse_args()
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
