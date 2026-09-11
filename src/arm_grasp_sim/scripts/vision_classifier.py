#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验三 视觉识别节点: 订阅俯视相机 /top_camera/image_raw (sensor_msgs/Image),
HSV 颜色分割输出 vision_msgs/Detection2DArray (类别+检测框+置信度)。

类别定义(与料盒颜色对应, 仿真阶段用颜色即类别):
  - green_block : 绿色方块 (0.15,0.70,0.25)
  - yellow_block: 黄色方块 (0.85,0.65,0.20)
接口符合实验要求第二节: 检测接口 vision_msgs/Detection2DArray, 至少含类别/检测框/置信度。
真机阶段可无缝替换为 YOLO 权重(best.pt), 本节点发布话题不变。

参数:
  image_topic  默认 /top_camera/image_raw
  out_topic    默认 /detections
  min_area     最小轮廓面积(px^2, 过滤噪声) 默认 80
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import cv2
import numpy as np
from cv_bridge import CvBridge


class VisionClassifier(Node):
    def __init__(self):
        super().__init__('vision_classifier')
        self.declare_parameter('image_topic', '/top_camera/image_raw')
        self.declare_parameter('out_topic', '/detections')
        self.declare_parameter('min_area', 80)
        self.declare_parameter('max_area', 4000)
        self.img_topic = self.get_parameter('image_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.min_area = int(self.get_parameter('min_area').value)
        self.max_area = int(self.get_parameter('max_area').value)
        self.bridge = CvBridge()
        # HSV 阈值: ((h_lo,h_hi),(s_lo,s_hi),(v_lo,v_hi))  逐类可多段
        self.ranges = {
            'green_block':  [((35, 60), (60, 255), (60, 255))],   # h 35-60
            'yellow_block': [((20, 35), (120, 255), (120, 255))], # h 20-35
            'red_block':    [((0, 10), (120, 255), (120, 255)),
                               ((170, 180), (120, 255), (120, 255))],  # h 跨 0
        }
        self.pub = self.create_publisher(Detection2DArray, self.out_topic, 10)
        self.sub = self.create_subscription(Image, self.img_topic, self._cb, 10)
        self.get_logger().info('vision_classifier 就绪: %s -> %s' % (self.img_topic, self.out_topic))

    def _cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn('cv_bridge 失败: %s' % e)
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        arr = Detection2DArray()
        arr.header = msg.header
        for cls, rs in self.ranges.items():
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for (h1, h2), s, v in rs:
                m = cv2.inRange(hsv, np.array([h1, s[0], v[0]]),
                                np.array([h2, s[1], v[1]]))
                mask |= m
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area < self.min_area or area > self.max_area:
                    continue  # 过滤: 太小=噪声, 太大=料盒(静态大色块)
                x, y, w, h = cv2.boundingRect(c)
                d = Detection2D()
                d.header = msg.header
                d.bbox.center.position.x = float(x + w / 2)
                d.bbox.center.position.y = float(y + h / 2)
                d.bbox.size_x = float(w)
                d.bbox.size_y = float(h)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cls
                # 置信度: 轮廓填充率(面积/外接框面积), 方块应接近 1
                hyp.hypothesis.score = float(min(1.0, area / float(w * h + 1e-6)))
                d.results.append(hyp)
                arr.detections.append(d)
        if arr.detections:
            self.pub.publish(arr)


def main():
    rclpy.init()
    node = VisionClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
