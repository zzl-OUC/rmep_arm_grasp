#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时探测 /detections 与 /vision_debug, 打印若干次摘要后退出。"""
import json
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String


class Probe(Node):
    def __init__(self):
        super().__init__('probe_detect')
        self.det = []
        self.dbg = []
        self.create_subscription(Detection2DArray, '/detections', self._d, 10)
        self.create_subscription(String, '/vision_debug', self._g, 10)

    def _d(self, msg):
        self.det = [{'cls': d.results[0].hypothesis.class_id,
                     'score': round(d.results[0].hypothesis.score, 3),
                     'cx': round(d.bbox.center.position.x, 1),
                     'cy': round(d.bbox.center.position.y, 1)}
                    for d in msg.detections]

    def _g(self, msg):
        try:
            self.dbg = json.loads(msg.data)
        except Exception:
            pass


def main():
    rclpy.init()
    n = Probe()
    for _ in range(12):  # ~12s
        rclpy.spin_once(n, timeout_sec=1.0)
    print('=== /detections (%d boxes) ===' % len(n.det))
    for d in n.det:
        print('  %s score=%.3f cx=%.0f cy=%.0f' % (d['cls'], d['score'], d['cx'], d['cy']))
    print('=== /vision_debug ===')
    for g in n.dbg:
        box = g.get('box')
        cls = g.get('class')
        dh = g.get('dom_hue')
        sf = g.get('sat_frac')
        vt = g.get('votes')
        print('  box=%s class=%s dom_hue=%s sat_frac=%s votes=%s' % (box, cls, dh, sf, vt))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
