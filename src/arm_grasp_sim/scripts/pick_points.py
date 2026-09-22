# -*- coding: utf-8 -*-
"""标定点选取工具 —— 从样张上取像素坐标, 生成 calib_real_grid.py 要吃的 points.json。

**离线可用**(不联网、不需要 ROS), 建议在 Windows 侧运行(有 GUI)。

用法
----
点击模式(推荐):
    python pick_points.py cam.png -o points.json
      · 左键        = 加一个点(按点击顺序编号)
      · u           = 撤销上一个点
      · r           = 全部重来
      · q / ESC     = 结束 -> 逐个输入该点的世界坐标
      · 拖右键      = 平移(放大后)   · 滚轮 = 缩放    · 空格 = 复位

只想要带坐标网格的图(手工读数, 无 GUI 时):
    python pick_points.py cam.png --grid -o cam_grid.png

参数
----
  --grid            不点击, 只在图上画 50px 坐标网格 + 刻度, 供人眼读数
  --scale N         显示缩放(默认自动缩到宽 <= 1100, 点选会换算回原图坐标)
  --grid-step N     网格间距(默认 50 px)

世界坐标怎么给
--------------
每个点依次输入:  <名称> <x> <y>       例如:  cell_1 0.19 0.00
· 名称随意(建议用 cell_1..cell_6 / bin_0), 会写进 points.json
· x/y 是 **臂基座系** 米: x 向前, y 向左(与 grid_mapper 的 cells 同系)
· 直接回车 = 跳过该点
坐标必须是真实量出来的(尺量), 不是猜的 —— 单应拟合对错都由此决定。
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

FONT = cv2.FONT_HERSHEY_SIMPLEX


class Picker(object):
    def __init__(self, img, scale):
        self.img = img
        self.scale = scale
        self.h, self.w = img.shape[:2]
        self.dw = int(round(self.w * scale))
        self.dh = int(round(self.h * scale))
        self.pts = []
        self.pan = [0, 0]
        self.zoom = 1.0
        self.win = 'pick_points'

    def to_orig(self, dx, dy):
        """显示坐标 -> 原图坐标(含缩放与平移)。"""
        ox = (dx + self.pan[0]) / (self.scale * self.zoom)
        oy = (dy + self.pan[1]) / (self.scale * self.zoom)
        return int(round(ox)), int(round(oy))

    def render(self):
        disp = cv2.resize(self.img, (self.dw, self.dh), interpolation=cv2.INTER_AREA)
        if self.zoom != 1.0 or self.pan != [0, 0]:
            # 放大/平移: 先在原图上取窗口再缩放, 保证点选精度
            z = self.scale * self.zoom
            x0, y0 = int(self.pan[0] / z), int(self.pan[1] / z)
            x1, y1 = int(x0 + self.dw / z), int(y0 + self.dh / z)
            x0, y0 = max(0, x0), max(0, y0)
            x1, y1 = min(self.w, x1), min(self.h, y1)
            crop = self.img[y0:y1, x0:x1]
            if crop.size:
                disp = cv2.resize(crop, (self.dw, self.dh),
                                  interpolation=cv2.INTER_NEAREST)
        for i, (u, v, _n) in enumerate(self.pts):
            z = self.scale * self.zoom
            dx = int(round(u * z - self.pan[0]))
            dy = int(round(v * z - self.pan[1]))
            if not (0 <= dx < self.dw and 0 <= dy < self.dh):
                continue
            cv2.drawMarker(disp, (dx, dy), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
            cv2.circle(disp, (dx, dy), 9, (0, 0, 255), 2)
            cv2.putText(disp, '#%d(%d,%d)' % (i + 1, u, v), (dx + 12, dy - 12),
                        FONT, 0.55, (0, 0, 255), 2)
        bar = ('n=%d   L=add  u=undo  r=reset  q=done   '
               'wheel=zoom  right-drag=pan  space=reset view' % len(self.pts))
        cv2.rectangle(disp, (0, 0), (self.dw, 26), (255, 255, 255), -1)
        cv2.putText(disp, bar, (8, 18), FONT, 0.5, (0, 0, 0), 1)
        cv2.imshow(self.win, disp)

    def on_mouse(self, ev, x, y, flags, _p):
        if ev == cv2.EVENT_LBUTTONDOWN:
            u, v = self.to_orig(x, y)
            if 0 <= u < self.w and 0 <= v < self.h:
                self.pts.append((u, v, ''))
                print('  + 点 #%d 像素=(%d, %d)' % (len(self.pts), u, v))
        elif ev == cv2.EVENT_MOUSEWHEEL:
            self.zoom = min(8.0, max(0.25, self.zoom * (1.15 if flags > 0 else 1 / 1.15)))
        elif ev == cv2.EVENT_RBUTTONDOWN:
            self._drag = (x, y)
        elif ev == cv2.EVENT_MOUSEMOVE and getattr(self, '_drag', None):
            dx0, dy0 = self._drag
            self.pan[0] -= (x - dx0)
            self.pan[1] -= (y - dy0)
            self._drag = (x, y)

    def run(self):
        cv2.namedWindow(self.win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.win, self.on_mouse)
        print('=' * 66)
        print('点击模式: 左键选点(按顺序编号) / u 撤销 / r 重来 / q 结束')
        print('目标: 至少 4 个点, 且不要几乎共线')
        print('=' * 66)
        while True:
            self.render()
            k = cv2.waitKey(20) & 0xFF
            if k in (ord('q'), 27):
                break
            if k == ord('u') and self.pts:
                u, v, _ = self.pts.pop()
                print('  - 撤销 (%d, %d)  剩 %d 点' % (u, v, len(self.pts)))
            if k == ord('r'):
                self.pts = []
                print('  - 已清空')
            if k == ord(' '):
                self.zoom, self.pan = 1.0, [0, 0]
        cv2.destroyWindow(self.win)


def make_grid(img, step):
    """画坐标网格 + 刻度, 供无 GUI 时手工读数。"""
    out = img.copy()
    h, w = out.shape[:2]
    for x in range(0, w, step):
        big = (x % (step * 2) == 0)
        cv2.line(out, (x, 0), (x, h), (180, 180, 180) if not big else (120, 120, 120), 1)
        cv2.putText(out, str(x), (x + 2, 14), FONT, 0.4, (0, 0, 200), 1)
    for y in range(0, h, step):
        big = (y % (step * 2) == 0)
        cv2.line(out, (0, y), (w, y), (180, 180, 180) if not big else (120, 120, 120), 1)
        cv2.putText(out, str(y), (2, y + 13), FONT, 0.4, (0, 0, 200), 1)
    cv2.putText(out, 'grid step = %d px (origin top-left)' % step, (8, h - 8),
                FONT, 0.5, (0, 0, 200), 1)
    return out


def ask_world(pts):
    """逐个输入世界坐标, 返回 points 列表。"""
    out = []
    print('=' * 66)
    print('依次输入每个点的世界坐标(臂基座系, 米):  名称 x y')
    print('例:  cell_1 0.19 0.00        (直接回车 = 跳过该点)')
    print('=' * 66)
    for i, (u, v, _n) in enumerate(pts):
        while True:
            s = input('点 #%d 像素(%d, %d) -> ' % (i + 1, u, v)).strip()
            if not s:
                print('   跳过')
                break
            parts = s.split()
            if len(parts) != 3:
                print('   !! 格式应为: <名称> <x> <y>')
                continue
            try:
                x, y = float(parts[1]), float(parts[2])
            except ValueError:
                print('   !! x/y 必须是数字(米)')
                continue
            out.append({'name': parts[0], 'world': [x, y], 'pixel': [u, v]})
            break
    return out


def main():
    ap = argparse.ArgumentParser(description='标定点选取(离线可用)')
    ap.add_argument('image', help='样张 PNG(probe_real_camera.py 存的那张)')
    ap.add_argument('-o', '--out', default='points.json')
    ap.add_argument('--grid', action='store_true', help='只画坐标网格, 不点击')
    ap.add_argument('--grid-step', type=int, default=50)
    ap.add_argument('--scale', type=float, default=0.0, help='显示缩放, 0=自动')
    a = ap.parse_args()

    img = cv2.imread(a.image)
    if img is None:
        print('!! 读不到图: %s' % a.image)
        return 2
    h, w = img.shape[:2]
    print('图像 %d x %d' % (w, h))

    if a.grid:
        out = make_grid(img, a.grid_step)
        cv2.imwrite(a.out, out)
        print('已生成带网格图: %s' % a.out)
        print('读法: 横轴 u 向右, 纵轴 v 向下, 直接看目标中心的刻度值。')
        return 0

    scale = a.scale if a.scale > 0 else min(1.0, 1100.0 / max(w, 1))
    try:
        pk = Picker(img, scale)
        pk.run()
        pts = ask_world(pk.pts)
    except cv2.error as e:
        print('!! 无法开图形窗口(%s)' % e)
        print('   改用: python %s "%s" --grid  然后用眼睛读刻度, 手工写 points.json'
              % (os.path.basename(__file__), a.image))
        return 3

    if len(pts) < 4:
        print('!! 只有 %d 个带世界坐标的点, 单应至少要 4 个' % len(pts))
        return 1
    data = {'points': pts,
            'meta': {'image': os.path.basename(a.image),
                     'image_size': [w, h],
                     'note': 'world = 臂基座系(米), x 向前 y 向左; pixel = 原图像素'}}
    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
    print('=' * 66)
    print('已写 %s (%d 点)' % (a.out, len(pts)))
    print('下一步:')
    print('  python3 calib_real_grid.py fit %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
