#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验三 视觉识别节点 —— 方案 C: 检测框 + 类别判定(两种模式)。

方案 C 要点:
  1) 检测框: 默认用「轮廓法」(仿真/无 GPU 时零依赖即可用); 可选 YOLO 后端
     (真机或杂乱场景, 设 use_yolo:=true 并提供 weights)。
  2) 类别判定 —— 两种模式, 由 class_from_yolo 切换:

     模式 A(默认, class_from_yolo=false, 用于仿真):
       对检测框内 ROI 计算色相直方图(仅统计饱和像素), 取主色相映射到
       绿/黄两类 —— 「类别由框内颜色决定」。仅适用于仿真彩色方块。

     模式 B(真机, class_from_yolo=true):
       类别直接采信检测模型(YOLO)给出的 class_id, 不做任何颜色投票。
       ⚠ 真机物体是「鼠标 + 网球」, 模式 A 在真机**必然失效**:
         - 鼠标多为黑/灰/白, 饱和度低于 is_colored 阈值(s>40) -> 轮廓法连框都
           找不到 -> 该网格被下游误判成空网格(skipped_empty, 假空网格);
         - 网球虽是高饱和黄绿, 但会被色相映射错判成 'yellow_block'。
       故真机必须走模式 B。

接口不变: 订阅 image_topic, 发布 /detections (vision_msgs/Detection2DArray)。
下游 grid_mapper / classify_task_node 完全不需要改动。

Detection2D 字段约定(下游依赖):
  d.bbox.center.position.x/y = 框中心像素坐标
  d.bbox.size_x / size_y      = 框宽/高(px)
  d.results[0].hypothesis.class_id = 类别名(仿真 green_block / yellow_block;
                                     真机为模型 names 里的名字, 如 mouse / tennis_ball)
  d.results[0].hypothesis.score     = 置信度
     模式 A = 框内主色像素占比; 模式 B = 检测模型置信度(conf)
     ⚠ 模式 B 下 score 就是 conf, 所以下游 classify_task_node 的 min_score
       **不得大于** yolo_conf, 否则所有检测都会被判成 skipped_unknown。

调试: 另发 /vision_debug (std_msgs/String JSON), 每项含 {class, score, dom_hue, sat_frac, votes},
便于真机标定色相边界时实时观察。
"""
import json
from collections import Counter

import cv2
import numpy as np

try:
    from classify_arbiter import (PENDING, DEFAULT_PROMPT, AsyncArbiter,
                                  HttpJsonArbiter, normalize_class, sanity_gate)
    _HAS_ARBITER = True
except Exception:  # 仲裁模块缺失时不影响主链路
    _HAS_ARBITER = False
    PENDING = object()
    DEFAULT_PROMPT = ''
    AsyncArbiter = HttpJsonArbiter = None

    def normalize_class(text, allowed=None, aliases=None):
        return None

    def sanity_gate(box, img_shape, **kw):
        return True

# ---- 2 类色相边界(OpenCV hue ∈ [0,179]); 本实验物体为网球/矿泉水瓶两类 ----
#   tennis_ball(荧光黄绿): 18-45    bottle(高饱和蓝): 95-130
#   其余色相(红等)与 背景/白色/灰(S 低或 V 过亮) 均不计入颜色像素。
HUE_BOUNDS = {
    'tennis_ball': ((18, 45),),
    'bottle':      ((95, 130),),
}

# 未识别类别: 检出框内确有足量饱和像素(是真物体), 但主色相不属任何已知类别。
# 必须把它作为一条 detection 发出去 —— 否则该物体在下游"凭空消失", 所在网格会被
# task 误判成空网格(skipped_empty), 而实验要求(四.6)明确把"空网格"与"未识别物体"
# 列为两类不同异常, 需要分别记录。类别名 'unknown' + score 0.0 会被
# classify_task_node 的 min_score 判为 skipped_unknown。详见 2026-09-14 异常测试。
UNKNOWN_CLASS = 'unknown'


def class_of_hue(h):
    """单像素色相 -> 类名(已在调用处过滤低饱和/过亮)。None 表示非本实验类别。"""
    if 18 <= h < 45:
        return 'tennis_ball'
    if 95 <= h < 130:
        return 'bottle'
    return None


def is_colored(s, v):
    """饱和像素掩码: 排除白/灰(低饱和)与过暗。

    注意: 不可加 v<高阈值 上限 —— 高饱和彩色方块 V 可达 255, 会被误杀。
    白底/反光靠 s>阈值(低饱和)排除即可; 大色块(料盒)靠 max_area 排除。
    """
    return (s > 40) & (v > 30)


def detect_boxes_contour(hsv, min_area=80, max_area=6000):
    """轮廓法: 任意饱和色块 -> 外接框。返回 [(x, y, w, h), ...]。

    max_area 6000: 矿泉水瓶(φ6.5cm×h10cm) 俯视投影约 50x77px ≈ 3850px²,
    原 4000 上限余量不足; 料盒为中性灰(S≈0)不进掩码, 提高上限无副作用。
    """
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


def detect_yolo(model, frame, conf=0.25):
    """YOLO 后端: 返回 [(box, cls_name, score), ...]。

    cls_name 取自模型的 names(真机为 'mouse' / 'tennis_ball' 之类), 仅在
    class_from_yolo=True(真机模式 B)时被采用; 模式 A 下类别仍由框内颜色决定,
    cls_name 只写进 /vision_debug 便于排查。
    """
    res = model(frame, conf=conf, verbose=False)[0]
    names = getattr(model, 'names', None)
    if not names:
        names = getattr(res, 'names', None) or {}
    out = []
    for b in res.boxes:
        # 兼容 ultralytics 的 tensor 与普通 list/np.ndarray
        try:
            vals = np.asarray(b.xyxy[0]).reshape(-1)
            x1, y1, x2, y2 = (float(vals[0]), float(vals[1]),
                              float(vals[2]), float(vals[3]))
        except Exception:
            continue
        if x2 <= x1 or y2 <= y1:
            continue    # 退化框(xyxy 非递增)直接丢, 别把负宽高传到下游
        try:
            cid = int(np.asarray(b.cls[0]).reshape(-1)[0])
        except Exception:
            cid = -1
        if isinstance(names, dict):
            cname = names.get(cid, 'cls_%d' % cid)
        elif isinstance(names, (list, tuple)) and 0 <= cid < len(names):
            cname = names[cid]
        else:
            cname = 'cls_%d' % cid
        try:
            sc = float(np.asarray(b.conf[0]).reshape(-1)[0])
        except Exception:
            sc = 0.0
        out.append(((int(x1), int(y1), int(x2 - x1), int(y2 - y1)),
                    str(cname), sc))
    return out


def detect_boxes_yolo(model, frame, conf=0.25):
    """兼容旧签名: 仅返回框(类别仍交给颜色直方图, 即模式 A)。"""
    return [box for box, _name, _sc in detect_yolo(model, frame, conf)]


class _OnnxBox:
    """模拟 ultralytics 单框对象, 让 detect_yolo 无需改动即可复用同一套解析。"""
    __slots__ = ('xyxy', 'cls', 'conf')

    def __init__(self, x1, y1, x2, y2, cid, score):
        self.xyxy = [np.array([x1, y1, x2, y2], dtype=np.float32)]
        self.cls = [np.array([cid], dtype=np.float32)]
        self.conf = [np.array([score], dtype=np.float32)]


class _OnnxResult:
    __slots__ = ('boxes', 'names')

    def __init__(self, boxes, names):
        self.boxes = boxes
        self.names = names


class OnnxYoloModel:
    """cv2.dnn 的 ONNX 后端 —— 不依赖 torch / ultralytics。

    真机侧(WSL)常无 GPU 版 torch, 装 ultralytics 要拖几百 MB~2.5GB; 而 opencv 的
    dnn 模块自带 ONNX 推理 + NMS, **零安装**即可用。实测 cv2 5.x 可直接读
    ultralytics 导出的 yolo11 ONNX(输出 (1, 4+nc, N), 未含 NMS)。

    对外接口与 ultralytics 对齐(返回 .boxes[*].xyxy/.cls/.conf), 因此 detect_yolo()
    一行都不用改 —— 两条后端共用同一段解析与退化框防护。

    letterbox 规则与 ultralytics 一致: 等比缩放 + 灰边(114)居中, 推理后按同一
    参数反变换回原图坐标, 否则框中心会整体偏移 -> 判错网格。
    """

    def __init__(self, weights, names, imgsz=640, iou=0.7, cuda=False):
        net = cv2.dnn.readNetFromONNX(weights)
        if cuda:
            try:
                net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            except Exception:
                pass  # 该 opencv 构建无 CUDA, 静默退回 CPU
        self.net = net
        if isinstance(names, dict):
            self.names = dict(names)
        else:
            self.names = dict(enumerate(names))
        self.imgsz = int(imgsz)
        self.iou = float(iou)

    @staticmethod
    def _letterbox(img, new=640, color=114):
        h, w = img.shape[:2]
        r = min(new / float(h), new / float(w))
        nh, nw = int(round(h * r)), int(round(w * r))
        if (nh, nw) != (h, w):
            img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        top, left = (new - nh) // 2, (new - nw) // 2
        canvas = np.full((new, new, 3), color, np.uint8)
        canvas[top:top + nh, left:left + nw] = img
        return canvas, r, left, top

    def __call__(self, frame, conf=0.25, verbose=False):
        img, r, dx, dy = self._letterbox(frame, self.imgsz)
        blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (self.imgsz, self.imgsz),
                                     swapRB=True, crop=False)
        self.net.setInput(blob)
        out = self.net.forward()             # (1, 4+nc, N)
        pred = np.asarray(out)[0].T          # (N, 4+nc)
        if pred.ndim != 2 or pred.shape[1] < 5:
            return _OnnxResult([], self.names)
        nc = pred.shape[1] - 4
        scores = pred[:, 4:4 + nc]
        cid = scores.argmax(axis=1)
        confs = scores[np.arange(scores.shape[0]), cid]
        keep = confs >= float(conf)
        pred, cid, confs = pred[keep], cid[keep], confs[keep]
        if pred.shape[0] == 0:
            return _OnnxResult([], self.names)
        cx, cy, bw, bh = pred[:, 0], pred[:, 1], pred[:, 2], pred[:, 3]
        # 反 letterbox: letterbox 坐标 -> 原图坐标
        x1 = (cx - bw / 2.0 - dx) / r
        y1 = (cy - bh / 2.0 - dy) / r
        x2 = (cx + bw / 2.0 - dx) / r
        y2 = (cy + bh / 2.0 - dy) / r
        rects = [[float(a), float(b), float(c - a), float(d - b)]
                 for a, b, c, d in zip(x1, y1, x2, y2)]
        idx = cv2.dnn.NMSBoxes(rects, [float(v) for v in confs],
                               float(conf), self.iou)
        boxes = []
        for i in np.asarray(idx).reshape(-1).tolist():
            boxes.append(_OnnxBox(x1[i], y1[i], x2[i], y2[i],
                                  int(cid[i]), float(confs[i])))
        return _OnnxResult(boxes, self.names)


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
    dom_hue = int(np.median(hues)) if False else int(np.argmax(np.bincount(hues)))
    return best, float(score), {
        'votes': cnt, 'pixels': n, 'dom_hue': dom_hue,
        'sat_frac': round(float(n / max(1, (roi.shape[0] * roi.shape[1]))), 2),
    }


def _call_arbiter(arb, frame, box, yolo_class):
    """兼容 arbiter(frame, box, yolo_class) 与旧式 arbiter(frame, box) 两种签名。

    把 YOLO 的类别一起传进去, 可让 VLM 做"确认 / 纠正"式判断(比开放式提问稳)。
    """
    try:
        return arb(frame, box, yolo_class)
    except TypeError:
        return arb(frame, box)


def process_frame(frame, *, min_area=80, max_area=6000,
                  use_yolo=False, yolo_model=None, yolo_conf=0.25,
                  class_from_yolo=False,
                  arbiter=None, arbiter_high=0.60, arbiter_low=0.15,
                  arbiter_accept_score=0.99, allowed_classes=None,
                  arbiter_gate=None):
    """纯函数(不依赖 ROS): BGR 帧 -> (detections, debug_list)。

    detections: list of dict {class_id, score, cx, cy, w, h}
    离线单测与节点回调共用, 保证算法与运行时一致。

    class_from_yolo=True(真机模式 B): 类别直接取检测模型 names, 不做颜色投票。
    arbiter 不为 None 时再叠一层"低置信度仲裁"(真机):
      conf >= arbiter_high               -> 直接用模型类别(不动);
      arbiter_low <= conf < arbiter_high -> 先过 arbiter_gate(几何 sanity) 再问 arbiter;
                                            命中闭集 -> 采纳, score 改写为 arbiter_accept_score
                                                       (否则原始低 conf 会被下游 min_score 判掉);
                                            归不了   -> 发 UNKNOWN_CLASS(-> skipped_unknown);
                                            返回 PENDING(异步未就绪) -> 本帧不报该框, 等下一帧取回;
      conf < arbiter_low                 -> 当噪声丢弃。
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if use_yolo and yolo_model is not None:
        raw = detect_yolo(yolo_model, frame, yolo_conf)
    else:
        raw = [(b, None, None) for b in detect_boxes_contour(hsv, min_area, max_area)]
    dets, dbg = [], []
    for box, yolo_name, yolo_score in raw:
        x, y, w, h = box
        if class_from_yolo and yolo_name is not None:
            # 真机模式 B: 类别由检测模型给出, 跳过色相投票。
            # 鼠标黑/灰/白低饱和, 走颜色法连框都找不到 -> 会被误判成空网格。
            conf = float(yolo_score)
            cls_out, score_out, src, extra = yolo_name, conf, 'yolo', {}
            if arbiter is not None and conf < arbiter_high:
                if conf < arbiter_low:
                    dbg.append({'box': [x, y, w, h], 'class': None,
                                'score': round(conf, 3), 'src': 'dropped_low_conf',
                                'yolo_class': yolo_name,
                                'reason': 'conf < arbiter_low(%.2f)' % arbiter_low})
                    continue
                if arbiter_gate and not sanity_gate(box, frame.shape, **arbiter_gate):
                    dbg.append({'box': [x, y, w, h], 'class': None,
                                'score': round(conf, 3), 'src': 'dropped_geom',
                                'yolo_class': yolo_name})
                    continue
                raw_text, raw_all = _call_arbiter(arbiter, frame, box, yolo_name)
                if raw_text is PENDING:
                    dbg.append({'box': [x, y, w, h], 'class': None,
                                'score': round(conf, 3), 'src': 'arbiter_pending',
                                'yolo_class': yolo_name})
                    continue
                mapped = normalize_class(raw_text, allowed=allowed_classes)
                extra = {'yolo_class': yolo_name, 'yolo_conf': round(conf, 3),
                         'arbiter_text': str(raw_text), 'arbiter_raw': str(raw_all)[:300]}
                if mapped:
                    cls_out, score_out = mapped, float(arbiter_accept_score)
                    src = 'arbiter'
                else:
                    cls_out, score_out, src = UNKNOWN_CLASS, 0.0, 'arbiter_unknown'
            dets.append({'class_id': cls_out, 'score': float(score_out),
                         'cx': x + w / 2, 'cy': y + h / 2, 'w': w, 'h': h})
            dbg.append({'box': [x, y, w, h], 'class': cls_out,
                        'score': round(float(score_out), 3), 'src': src, **extra})
            continue
        cls, score, info = classify_box(hsv, box)
        if cls is None:
            # 'no_class_vote': 框内饱和像素够多(是真物体)但色相不属已知类别
            #   -> 作为 unknown 类别发出, 让下游能记为"未识别物体"并跳过。
            # 'colored_pixels<20': 疑似噪声/反光碎块 -> 维持丢弃。
            if info.get('reason') == 'no_class_vote':
                dets.append({'class_id': UNKNOWN_CLASS, 'score': 0.0,
                             'cx': x + w / 2, 'cy': y + h / 2, 'w': w, 'h': h})
                dbg.append({'box': [x, y, w, h], 'class': UNKNOWN_CLASS,
                            'score': 0.0, **info})
            else:
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
            self.declare_parameter('max_area', 6000)
            self.declare_parameter('use_yolo', False)
            self.declare_parameter('yolo_weights', '')
            self.declare_parameter('yolo_conf', 0.25)
            # 检测后端: auto=.onnx 走 cv2.dnn(零依赖), 其它走 ultralytics(需 torch)
            self.declare_parameter('yolo_backend', 'auto')
            self.declare_parameter('yolo_names', 'mouse,tennis_ball')
            self.declare_parameter('yolo_imgsz', 640)
            self.declare_parameter('yolo_iou', 0.7)
            self.declare_parameter('yolo_cuda', False)
            # 真机模式 B: 类别直接采信检测模型(YOLO)的 class_id, 不做色相投票。
            # 仿真(彩色方块)保持 False —— 仿真行为完全不变。
            self.declare_parameter('class_from_yolo', False)
            # ---- 低置信度仲裁层(可选, 默认关) ----
            self.declare_parameter('low_conf_arbiter', False)
            self.declare_parameter('arbiter_high', 0.60)
            self.declare_parameter('arbiter_low', 0.15)
            self.declare_parameter('arbiter_accept_score', 0.99)
            self.declare_parameter('arbiter_timeout', 0.8)
            self.declare_parameter('arbiter_url', '')
            self.declare_parameter('arbiter_async', True)
            self.declare_parameter('arbiter_prompt', '')
            self.declare_parameter('arbiter_payload_style', 'openai')
            self.declare_parameter('arbiter_model', '')
            self.declare_parameter('arbiter_max_tokens', 8)
            self.declare_parameter('arbiter_allowed', '')
            self.declare_parameter('arbiter_min_area', 300)
            self.declare_parameter('arbiter_max_area', 4000)
            self.declare_parameter('arbiter_max_aspect', 4.0)
            self.declare_parameter('publish_debug', True)
            self.img_topic = self.get_parameter('image_topic').value
            self.out_topic = self.get_parameter('out_topic').value
            self.min_area = int(self.get_parameter('min_area').value)
            self.max_area = int(self.get_parameter('max_area').value)
            self.use_yolo = bool(self.get_parameter('use_yolo').value)
            self.yolo_weights = self.get_parameter('yolo_weights').value
            self.yolo_conf = float(self.get_parameter('yolo_conf').value)
            self.class_from_yolo = bool(self.get_parameter('class_from_yolo').value)
            self.publish_debug = bool(self.get_parameter('publish_debug').value)
            self.arbiter_high = float(self.get_parameter('arbiter_high').value)
            self.arbiter_low = float(self.get_parameter('arbiter_low').value)
            self.arbiter_accept_score = float(
                self.get_parameter('arbiter_accept_score').value)
            _allowed = str(self.get_parameter('arbiter_allowed').value or '')
            self.arbiter_allowed = set(
                s.strip() for s in _allowed.split(',') if s.strip()) or None
            self.arbiter_gate = {
                'min_area': int(self.get_parameter('arbiter_min_area').value),
                'max_area': int(self.get_parameter('arbiter_max_area').value),
                'max_ar': float(self.get_parameter('arbiter_max_aspect').value),
            }
            self.arbiter = self._make_arbiter()

            self.bridge = CvBridge()
            self.yolo_model = None
            if self.use_yolo:
                if not self.yolo_weights:
                    self.get_logger().error('use_yolo=true 但未给 yolo_weights, 回退轮廓法')
                    self.use_yolo = False
                else:
                    self.yolo_model = self._load_detector()
                    if self.yolo_model is None:
                        self.use_yolo = False

            self.pub = self.create_publisher(Detection2DArray, self.out_topic, 10)
            self.dbg_pub = self.create_publisher(
                __import__('std_msgs.msg', fromlist=['String']).String,
                '/vision_debug', 10) if self.publish_debug else None
            self.sub = self.create_subscription(Image, self.img_topic, self._cb, 10)
            mode = 'YOLO(%s)' % self.yolo_weights if self.use_yolo else '轮廓法'
            cls_mode = '类别=YOLO给出' if self.class_from_yolo else '类别=色相投票'
            if self.class_from_yolo and not self.use_yolo:
                self.get_logger().warn(
                    'class_from_yolo=true 但 use_yolo=false -> 类别仍走色相投票; 真机请务必开启 use_yolo')
            self.get_logger().info('vision_classifier [方案C] 就绪: %s -> %s (检测=%s, %s, %s)'
                                   % (self.img_topic, self.out_topic, mode, cls_mode,
                                      '仲裁=开' if self.arbiter is not None else '仲裁=关'))

        def _load_detector(self):
            """按 yolo_backend 装配检测后端。

            auto: 权重以 .onnx 结尾 -> cv2.dnn(零依赖, 真机 WSL 默认走这条);
                  其它(.pt/.engine) -> ultralytics(需 torch)。
            两条后端对外接口一致, 下游 detect_yolo/process_frame 无需区分。
            """
            w = str(self.yolo_weights)
            backend = str(self.get_parameter('yolo_backend').value or 'auto').lower()
            names = [s.strip() for s in
                     str(self.get_parameter('yolo_names').value).split(',') if s.strip()]
            if backend == 'auto':
                backend = 'onnx' if w.lower().endswith('.onnx') else 'ultralytics'
            if backend == 'onnx':
                try:
                    m = OnnxYoloModel(
                        w, names,
                        imgsz=int(self.get_parameter('yolo_imgsz').value),
                        iou=float(self.get_parameter('yolo_iou').value),
                        cuda=bool(self.get_parameter('yolo_cuda').value))
                    self.get_logger().info(
                        '检测后端=cv2.dnn(ONNX) 已加载: %s  names=%s' % (w, m.names))
                    return m
                except Exception as e:
                    self.get_logger().error('ONNX 加载失败(%s)' % e)
                    return None
            try:
                from ultralytics import YOLO
                m = YOLO(w)
                self.get_logger().info('检测后端=ultralytics 已加载: %s' % w)
                return m
            except Exception as e:
                self.get_logger().error(
                    'ultralytics 加载失败(%s); 若为 .onnx 权重请设 yolo_backend:=onnx' % e)
                return None

        def _make_arbiter(self):
            """按参数装配仲裁器; 未启用/未配置 -> None(整层不生效, 行为与改前逐行一致)。"""
            if not bool(self.get_parameter('low_conf_arbiter').value):
                return None
            if not _HAS_ARBITER:
                self.get_logger().error(
                    'low_conf_arbiter=true 但 classify_arbiter 模块不可用 -> 不启用')
                return None
            url = self.get_parameter('arbiter_url').value
            if not url:
                self.get_logger().error(
                    'low_conf_arbiter=true 但未给 arbiter_url -> 不启用')
                return None
            prompt = self.get_parameter('arbiter_prompt').value or DEFAULT_PROMPT
            inner = HttpJsonArbiter(
                url, prompt=prompt,
                timeout=float(self.get_parameter('arbiter_timeout').value),
                payload_style=self.get_parameter('arbiter_payload_style').value,
                model=self.get_parameter('arbiter_model').value,
                max_tokens=int(self.get_parameter('arbiter_max_tokens').value))
            if bool(self.get_parameter('arbiter_async').value):
                inner = AsyncArbiter(inner)
            self.get_logger().info(
                '低置信度仲裁已启用: %s (band=[%.2f, %.2f), accept_score=%.2f, %s)'
                % (url, self.arbiter_low, self.arbiter_high,
                   self.arbiter_accept_score, getattr(inner, 'name', 'arbiter')))
            return inner

        def _cb(self, msg):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().warn('cv_bridge 失败: %s' % e)
                return
            dets, dbg = process_frame(
                frame, min_area=self.min_area, max_area=self.max_area,
                use_yolo=self.use_yolo, yolo_model=self.yolo_model,
                yolo_conf=self.yolo_conf, class_from_yolo=self.class_from_yolo,
                arbiter=self.arbiter, arbiter_high=self.arbiter_high,
                arbiter_low=self.arbiter_low,
                arbiter_accept_score=self.arbiter_accept_score,
                allowed_classes=self.arbiter_allowed,
                arbiter_gate=self.arbiter_gate)
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
