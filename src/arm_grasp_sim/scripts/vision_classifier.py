#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验三 视觉识别节点 —— 方案 C: YOLO 检测框 + 框内颜色直方图分类。

方案 C 要点(与用户确认):
  1) 检测框: 默认用「轮廓法」(仿真/无 GPU 时零依赖即可用); 可选 YOLO 后端
     (真机或杂乱场景, 设 use_yolo:=true 并提供 weights)。两种方式都只产出
     bounding box (x, y, w, h), 与分类解耦。
  2) 分类: 对检测框内 ROI 计算颜色直方图(色相直方图, 仅统计饱和像素), 取主色相映射到
     绿/黄/红/蓝四类 —— 即「类别由框内颜色决定」。这是方案 C 的核心。

接口不变: 订阅 /top_camera/image_raw, 发布 /detections (vision_msgs/Detection2DArray)。
下游 grid_mapper / classify_task_node 完全不需要改动。

Detection2D 字段约定(下游依赖):
  d.bbox.center.position.x/y = 框中心像素坐标
  d.bbox.size_x / size_y      = 框宽/高(px)
  d.results[0].hypothesis.class_id = green_block / yellow_block / red_block / blue_block
  d.results[0].hypothesis.score     = 置信度(框内主色占比, 0~1)

调试: 另发 /vision_debug (std_msgs/String JSON), 每项含 {class, score, dom_hue, sat_frac, votes},
便于真机标定色相边界时实时观察。
"""
import json
from collections import Counter

import cv2
import numpy as np

# ---- 4 类色相边界(OpenCV hue ∈ [0,179]); 作为可调参数, 此处为仿真 Gazebo 材质默认 ----
#   yellow: 18-35   green: 35-85   blue: 85-130   red: 0-15 / 165-180
#   背景/白色/灰(S 低或 V 过亮) 不计入颜色像素。
HUE_BOUNDS = {
    'yellow_block': ((18, 35),),
    'green_block':  ((35, 85),),
    'blue_block':   ((85, 130),),
    'red_block':    ((0, 15), (165, 180)),
}


def class_of_hue(h):
    """单像素色相 -> 类名(已在调用处过滤低饱和/过亮)。None 表示边界外。"""
    if h <= 15 or h >= 165:
        return 'red_block'
    if h < 35:
        return 'yellow_block'
    if h < 85:
        return 'green_block'
    if h < 130:
        return 'blue_block'
    return None


def is_colored(s, v):
    """饱和像素掩码: 排除白/灰(低饱和)与过暗。

    注意: 不可加 v<高阈值 上限 —— 高饱和彩色方块 V 可达 255, 会被误杀。
    白底/反光靠 s>阈值(低饱和)排除即可; 大色块(料盒)靠 max_area 排除。
    """
    return (s > 40) & (v > 30)


def detect_boxes_contour(hsv, min_area=80, max_area=4000):
    """轮廓法: 任意饱和色块 -> 外接框。返回 [(x, y, w, h), ...]。"""
    s = hsv[:, :, 1]
    v = hsv[:, :, 2]
    mask = is_colored(s, v).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for c in cnts:
        a = cv2.contourArea(c)
        if a < min_area or a > max_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        boxes.append((x, y, w, h))
    return boxes


def detect_boxes_yolo(model, frame, conf=0.25):
    """YOLO 后端: 仅取框, 类别一律交给直方图。返回 [(x, y, w, h), ...]。"""
    res = model(frame, conf=conf, verbose=False)[0]
    boxes = []
    for b in res.boxes:
        x1, y1, x2, y2 = b.xyxy[0].tolist()
        boxes.append((int(x1), int(y1), int(x2 - x1), int(y2 - y1)))
    return boxes


def classify_box(hsv, box):
    """框内颜色直方图 -> (class_id, score, debug_dict)。"""
    x, y, w, h = (int(z) for z in box)
    roi = hsv[max(0, y):y + h, max(0, x):x + w]
    if roi.size == 0:
        return None, 0.0, {}
    hue = roi[:, :, 0]
    s = roi[:, :, 1]
    v = roi[:, :, 2]
    sel = is_colored(s, v)
    n = int(sel.sum())
    if n < 20:
        return None, 0.0, {'reason': 'colored_pixels<20', 'pixels': n}
    hues = hue[sel]
    cnt = Counter(class_of_hue(int(hv)) for hv in hues)
    cnt = {k: int(val) for k, val in cnt.items() if k}
    if not cnt:
        return None, 0.0, {'reason': 'no_class_vote', 'pixels': n}
    best = max(cnt, key=cnt.get)
    score = cnt[best] / n  # 主色占全部颜色像素比例
    dom_hue = int(np.median(hues)) if False else int(hues[np.argmax(np.bincount(hues))])
    return best, float(score), {
        'votes': cnt, 'pixels': n, 'dom_hue': dom_hue,
        'sat_frac': round(float(n / max(1, (roi.shape[0] * roi.shape[1]))), 2),
    }


def process_frame(frame, *, min_area=80, max_area=4000,
                  use_yolo=False, yolo_model=None, yolo_conf=0.25):
    """纯函数(不依赖 ROS): BGR 帧 -> (detections, debug_list)。

    detections: list of dict {class_id, score, cx, cy, w, h}
    离线单测与节点回调共用, 保证算法与运行时一致。
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if use_yolo and yolo_model is not None:
        boxes = detect_boxes_yolo(yolo_model, frame, yolo_conf)
    else:
        boxes = detect_boxes_contour(hsv, min_area, max_area)
    dets, dbg = [], []
    for box in boxes:
        cls, score, info = classify_box(hsv, box)
        x, y, w, h = box
        if cls is None:
            dbg.append({'box': [x, y, w, h], 'class': None, **info})
            continue
        dets.append({'class_id': cls, 'score': score,
                     'cx': x + w / 2, 'cy': y + h / 2, 'w': w, 'h': h})
        dbg.append({'box': [x, y, w, h], 'class': cls, 'score': round(score, 3), **info})
    return dets, dbg


# ======================================================================
# 以下为 ROS 节点部分; 纯函数(process_frame 等)可在无 ROS 环境单独 import。
# ======================================================================
def _ros_main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    from cv_bridge import CvBridge

    class VisionClassifier(Node):
        def __init__(self):
            super().__init__('vision_classifier')
            self.declare_parameter('image_topic', '/top_camera/image_raw')
            self.declare_parameter('out_topic', '/detections')
            self.declare_parameter('min_area', 80)
            self.declare_parameter('max_area', 4000)
            self.declare_parameter('use_yolo', False)
            self.declare_parameter('yolo_weights', '')
            self.declare_parameter('yolo_conf', 0.25)
            self.declare_parameter('publish_debug', True)
            self.img_topic = self.get_parameter('image_topic').value
            self.out_topic = self.get_parameter('out_topic').value
            self.min_area = int(self.get_parameter('min_area').value)
            self.max_area = int(self.get_parameter('max_area').value)
            self.use_yolo = bool(self.get_parameter('use_yolo').value)
            self.yolo_weights = self.get_parameter('yolo_weights').value
            self.yolo_conf = float(self.get_parameter('yolo_conf').value)
            self.publish_debug = bool(self.get_parameter('publish_debug').value)

            self.bridge = CvBridge()
            self.yolo_model = None
            if self.use_yolo:
                if not self.yolo_weights:
                    self.get_logger().error('use_yolo=true 但未给 yolo_weights, 回退轮廓法')
                    self.use_yolo = False
                else:
                    try:
                        from ultralytics import YOLO
                        self.yolo_model = YOLO(self.yolo_weights)
                        self.get_logger().info('YOLO 模型已加载: %s' % self.yolo_weights)
                    except Exception as e:
                        self.get_logger().error('YOLO 加载失败(%s), 回退轮廓法' % e)
                        self.use_yolo = False

            self.pub = self.create_publisher(Detection2DArray, self.out_topic, 10)
            self.dbg_pub = self.create_publisher(
                __import__('std_msgs.msg', fromlist=['String']).String,
                '/vision_debug', 10) if self.publish_debug else None
            self.sub = self.create_subscription(Image, self.img_topic, self._cb, 10)
            mode = 'YOLO(%s)' % self.yolo_weights if self.use_yolo else '轮廓法'
            self.get_logger().info('vision_classifier [方案C] 就绪: %s -> %s (检测=%s)'
                                   % (self.img_topic, self.out_topic, mode))

        def _cb(self, msg):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().warn('cv_bridge 失败: %s' % e)
                return
            dets, dbg = process_frame(
                frame, min_area=self.min_area, max_area=self.max_area,
                use_yolo=self.use_yolo, yolo_model=self.yolo_model,
                yolo_conf=self.yolo_conf)
            arr = Detection2DArray()
            arr.header = msg.header
            for d in dets:
                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = float(d['cx'])
                det.bbox.center.position.y = float(d['cy'])
                det.bbox.size_x = float(d['w'])
                det.bbox.size_y = float(d['h'])
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = d['class_id']
                hyp.hypothesis.score = float(d['score'])
                det.results.append(hyp)
                arr.detections.append(det)
            if arr.detections:
                self.pub.publish(arr)
            if self.dbg_pub is not None:
                m = __import__('std_msgs.msg', fromlist=['String']).String()
                m.data = json.dumps(dbg)
                self.dbg_pub.publish(m)

    rclpy.init()
    node = VisionClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    _ros_main()
