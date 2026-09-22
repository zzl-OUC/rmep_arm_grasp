#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验三 真机相机探测 —— 上机第一步。

把"真机上到底有哪些图像话题、分辨率多少、帧率多少、内参有没有"一次问清楚, 并把
样张存盘, 供人眼确认"这台相机到底能不能俯视看到桌面"。

用法(先另开终端启动驱动):
    ros2 launch robomaster_ros main.launch model:=ep conn_type:=ap
    ros2 run arm_grasp_sim probe_real_camera.py
    ros2 run arm_grasp_sim probe_real_camera.py --ros-args \
        -p duration:=20.0 -p out_dir:=/tmp/cam_probe

输出:
    终端: 每个话题的 分辨率/编码/实测帧率; CameraInfo 里 K 矩阵 -> 反推 hfov
    <out_dir>/<topic>.png   样张(直接打开看视野对不对)
"""
import math
import os
import re

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def _safe(name):
    return re.sub(r'[^0-9A-Za-z_.-]', '_', name.strip('/') or 'root')


class Probe(Node):
    def __init__(self):
        super().__init__('probe_real_camera')
        self.declare_parameter('duration', 12.0)
        self.declare_parameter('out_dir', '/tmp/cam_probe')
        self.duration = float(self.get_parameter('duration').value)
        self.out_dir = str(self.get_parameter('out_dir').value)
        os.makedirs(self.out_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.stats = {}
        self.subs = []
        self.done = False
        self.deadline = 0.0
        self.create_timer(0.5, self._setup_once)

    # ---------- 话题发现 ----------
    def _setup_once(self):
        if self.stats:
            return
        found_img, found_ci = [], []
        for name, types in self.get_topic_names_and_types():
            if 'sensor_msgs/msg/Image' in types:
                found_img.append((name, 'Image'))
            elif 'sensor_msgs/msg/CompressedImage' in types:
                found_img.append((name, 'CompressedImage'))
            if 'sensor_msgs/msg/CameraInfo' in types:
                found_ci.append(name)

        print('=' * 74)
        if not found_img:
            print('!! 未发现图像话题 —— 机器人未连 / 驱动未起 / 相机未开。')
            print('   先跑: ros2 launch robomaster_ros main.launch model:=ep '
                  'conn_type:=ap')
            print('   当前全部话题:')
            for n, t in sorted(self.get_topic_names_and_types()):
                print('     %-42s %s' % (n, ','.join(x.split('/')[-1] for x in t)))
            print('=' * 74)
            self.done = True
            return

        print('发现 %d 个图像话题, 采样 %.0f 秒:' % (len(found_img), self.duration))
        for name, kind in found_img:
            self.stats[name] = {'kind': kind, 'n': 0, 't0': None, 't1': None,
                                'shape': None, 'encoding': '', 'saved': None}
            print('  - %-40s (%s)' % (name, kind))
            if kind == 'Image':
                self.subs.append(self.create_subscription(
                    Image, name, lambda m, n=name: self._on_img(m, n), 10))
            else:
                self.subs.append(self.create_subscription(
                    CompressedImage, name,
                    lambda m, n=name: self._on_cimg(m, n), 10))
        for name in found_ci:
            self.create_subscription(CameraInfo, name, self._on_ci, 10)
            print('  - %-40s (CameraInfo)' % name)
        print('=' * 74)
        self.deadline = self.get_clock().now().nanoseconds * 1e-9 + self.duration
        self.create_timer(1.0, self._maybe_report)

    # ---------- 回调 ----------
    def _tick(self, name, stamp):
        s = self.stats.get(name)
        if s is None:
            return
        if s['t0'] is None:
            s['t0'] = stamp
        s['t1'] = stamp
        s['n'] += 1

    def _on_img(self, msg, name):
        self._tick(name, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        s = self.stats[name]
        if s['shape'] is not None:
            return
        s['shape'] = (msg.width, msg.height)
        s['encoding'] = msg.encoding
        try:
            arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            p = os.path.join(self.out_dir, _safe(name) + '.png')
            cv2.imwrite(p, arr)
            s['saved'] = p
        except Exception as e:
            print('[警告] %s 转图失败: %s' % (name, e))

    def _on_cimg(self, msg, name):
        self._tick(name, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        s = self.stats[name]
        if s['shape'] is not None:
            return
        try:
            arr = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                               cv2.IMREAD_COLOR)
            if arr is None:
                return
            h, w = arr.shape[:2]
            s['shape'] = (w, h)
            s['encoding'] = msg.format or 'compressed'
            p = os.path.join(self.out_dir, _safe(name) + '.png')
            cv2.imwrite(p, arr)
            s['saved'] = p
        except Exception as e:
            print('[警告] %s 解码失败: %s' % (name, e))

    def _on_ci(self, msg):
        tag = msg.header.frame_id or '(no frame)'
        k = 'CI:' + tag
        if k in self.stats:
            return
        self.stats[k] = None
        print('[CameraInfo] %s  %dx%d  K=%s  D=%s'
              % (tag, msg.width, msg.height,
                 [round(v, 2) for v in msg.k[:9]],
                 [round(v, 4) for v in msg.d[:5]]))
        if msg.k[0] and msg.width:
            hfov = 2 * math.atan((msg.width / 2.0) / msg.k[0])
            print('             -> 由 K 反推 hfov=%.4f rad (%.1f deg)'
                  % (hfov, math.degrees(hfov)))
            print('             -> 可直接填 grid_mapper 的 hfov/image_width/'
                  'image_height')

    # ---------- 汇总 ----------
    def _maybe_report(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self.deadline:
            return
        print('=' * 74)
        print('%-40s %-10s %-8s %s' % ('话题', '分辨率', '帧率', '样张'))
        for name, s in self.stats.items():
            if not s:
                continue
            dur = (s['t1'] - s['t0']) if (s['t0'] and s['t1']) else 0.0
            fps = (s['n'] - 1) / dur if dur > 0.05 else 0.0
            wh = '%dx%d' % s['shape'] if s['shape'] else '无帧'
            print('%-40s %-10s %6.1ffps  %s' % (name, wh, fps,
                                                s['saved'] or '(未存)'))
        print('=' * 74)
        print('接着做: 打开样张确认视野 —— 必须能俯视看到桌面网格;')
        print('        否则先调相机安装/云台俯仰角, 再谈标定。')
        self.done = True


def main():
    rclpy.init()
    node = Probe()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
