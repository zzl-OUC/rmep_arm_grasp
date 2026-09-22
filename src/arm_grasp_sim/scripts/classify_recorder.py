#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验三 验证视频录制节点 —— 把俯视相机画面录成 mp4（可选叠加检测类别框）。

用途:
  实验三(桌面物体分类整理)需要一段"6/6 自动分类"的验证视频。本节点只订阅标准话题,
  不依赖任何仿真/真机专有文件, 因此仿真(classify_sim)与真机(classify_real)都能用。

  - 主画面: /top_camera/image_raw (sensor_msgs/Image, BGR)
  - 叠加层(可选): /detections (vision_msgs/Detection2DArray) -> 画框 + 类别 + 置信度
      仿真: 类别为 green_block / yellow_block; 真机: mouse / tennis_ball
    不订阅 /detections 也能录(纯画面), 设 draw:=false 关闭叠加。

用法(仿真验收录视频, 在起好 classify_sim 的另一个终端):
  ros2 run arm_grasp_sim classify_recorder.py \
      --ros-args -p out:=/home/<用户>/videos/exp3_classify_sim_6of6.mp4

  # 或用自己的脚本:
  bash src/arm_grasp_sim/scripts/record_exp3_classify.sh

停止: Ctrl-C; 节点在退出时释放 VideoWriter 并汇报总帧数/时长。

注意:
  - 默认 fps=30; 仿真相机约 66Hz 且 RTF≈2.2, 故录下来的视频回放会比真实节奏快,
    属于正常现象(验收看的是"6 块全被识别并分入对应料盒"的过程, 不在乎倍速)。
  - 想控制文件大小可用 --every-n N(每 N 帧存 1 帧) 或 --max-seconds S 自动停止。
"""
import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def _ros_time_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class ClassifyRecorder(Node):
    def __init__(self):
        super().__init__('classify_recorder')
        # ---- 参数 ----
        self.declare_parameter('image_topic', '/top_camera/image_raw')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('out',
                               os.path.expanduser('~/videos/exp3_classify.mp4'))
        self.declare_parameter('fps', 30)
        self.declare_parameter('fourcc', 'mp4v')
        self.declare_parameter('draw', True)
        self.declare_parameter('every_n', 1)            # 每 N 帧存 1 帧(1=全存)
        self.declare_parameter('max_seconds', 0)   # >0 自动停止(秒, 整数); 0=不限时

        self.img_topic = self.get_parameter('image_topic').value
        self.det_topic = self.get_parameter('detections_topic').value
        self.out_path = os.path.expanduser(self.get_parameter('out').value)
        self.fps = float(self.get_parameter('fps').value)
        self.fourcc = self.get_parameter('fourcc').value
        self.draw = bool(self.get_parameter('draw').value)
        self.every_n = max(1, int(self.get_parameter('every_n').value))
        self.max_seconds = float(self.get_parameter('max_seconds').value)

        self.bridge = CvBridge()
        self._writer = None
        self._size = None
        self._n = 0
        self._seen_det = []          # 最近一次 /detections 的绘制信息
        self._start_t = None
        self._last_draw_t = 0.0

        # 输出目录
        od = os.path.dirname(self.out_path)
        if od and not os.path.isdir(od):
            os.makedirs(od, exist_ok=True)

        self.sub_img = self.create_subscription(
            Image, self.img_topic, self._on_img, 10)
        if self.draw:
            # vision_msgs 在导入失败时(纯画面模式不需要)也不阻塞主链路
            try:
                from vision_msgs.msg import Detection2DArray
                self.sub_det = self.create_subscription(
                    Detection2DArray, self.det_topic, self._on_det, 10)
            except Exception as e:  # pragma: no cover
                self.get_logger().warn('未加载 /detections 叠加(缺 vision_msgs): %s' % e)
                self.draw = False
        else:
            self.sub_det = None

        self.get_logger().info(
            'recorder 就绪: image=%s det=%s out=%s draw=%s every_n=%d fps=%.0f'
            % (self.img_topic, self.det_topic if self.draw else '(off)',
               self.out_path, self.draw, self.every_n, self.fps))

    # ---- 回调 ----
    def _on_det(self, msg):
        """缓存最近一次检测结果用于叠加(不逐帧重画, 用最新快照即可)。"""
        items = []
        for d in msg.detections:
            cx = d.bbox.center.position.x
            cy = d.bbox.center.position.y
            w = d.bbox.size_x
            h = d.bbox.size_y
            cls = ''
            score = 0.0
            if d.results:
                hyp = d.results[0].hypothesis
                cls = hyp.class_id or ''
                score = float(hyp.score)
            items.append((cx, cy, w, h, cls, score))
        self._seen_det = items

    def _on_img(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn('cv_bridge 失败: %s' % e)
            return
        h, w = frame.shape[:2]
        if self._size is None:
            self._size = (w, h)
            try:
                cc = cv2.VideoWriter_fourcc(*self.fourcc)
                self._writer = cv2.VideoWriter(self.out_path, cc, self.fps,
                                               self._size)
                if not self._writer.isOpened():
                    self.get_logger().error('VideoWriter 打不开: %s' % self.out_path)
                    self._writer = None
                    return
            except Exception as e:
                self.get_logger().error('VideoWriter 初始化失败: %s' % e)
                return
            self._start_t = _ros_time_sec(msg) if hasattr(msg.header, 'stamp') \
                else None
            self.get_logger().info('开始录制 %dx%d -> %s' % (w, h, self.out_path))

        # 叠加检测框
        if self.draw and self._seen_det:
            for (cx, cy, bw, bh, cls, score) in self._seen_det:
                x0 = int(round(cx - bw / 2))
                y0 = int(round(cy - bh / 2))
                x1 = int(round(cx + bw / 2))
                y1 = int(round(cy + bh / 2))
                color = (0, 255, 0) if 'green' in cls else \
                        (0, 255, 255) if 'yellow' in cls else (255, 128, 0)
                cv2.rectangle(frame, (x0, y0), (x1, y1), color, 2)
                if cls:
                    label = '%s %.2f' % (cls, score) if score else cls
                    cv2.putText(frame, label, (x0, max(0, y0 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
                                cv2.LINE_AA)

        # 节流
        self._n += 1
        if self._n % self.every_n != 0:
            return
        if self._writer is not None:
            try:
                self._writer.write(frame)
            except Exception as e:
                self.get_logger().warn('写帧失败: %s' % e)

        # 自动停止
        if self.max_seconds and self._start_t is not None:
            t = _ros_time_sec(msg) if hasattr(msg.header, 'stamp') else None
            if t is not None and (t - self._start_t) >= self.max_seconds:
                self.get_logger().info('达到 max_seconds=%.0f, 停止录制' % self.max_seconds)
                rclpy.shutdown()

    def destroy_node(self):
        if self._writer is not None:
            try:
                self._writer.release()
            except Exception:
                pass
            self._writer = None
        if self._n:
            self.get_logger().info('录制完成: 共 %d 帧 -> %s' % (self._n, self.out_path))
        super().destroy_node()


def main():
    rclpy.init()
    node = ClassifyRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
