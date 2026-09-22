#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验三 低置信度仲裁层(可选) —— YOLO 出框, 分歧时才交 VLM 做最终类别决断。

设计对应 5 个已知坑:
  1) 低置信度必须先"活下来": YOLO 会把 conf < yolo_conf 的框直接丢掉, 要让仲裁有活干,
     必须把 yolo_conf 降到 arbiter_low 附近。为防噪声被 VLM "幻觉出类别", 仲裁前先过
     sanity_gate(面积/长宽比/不顶边)。
  2) 采纳后 score 必须改写成"已采纳"值(ACCEPTED_SCORE=arbiter_high), 否则原始低 conf
     会被 classify_task_node.min_score 判掉 -> 仲裁白做。
  3) 表外类别名绝不静默兜底: normalize_class 白名单归一化, 归不了返回 None, 上层发
     UNKNOWN_CLASS -> 记 skipped_unknown(而不是进错料盒)。
  4) 调用必须异步+超时: AsyncArbiter 用线程池"提交-取回", 未就绪时返回 PENDING,
     上层本帧不报该框(等下一帧取回), 避免阻塞图像回调。
  5) 可审计: 每次仲裁返回 raw 文本, 上层写进 /vision_debug 与任务日志。

本模块不依赖 ROS, 可离线单测。
"""
import base64
import json
import re
import urllib.request

# 未就绪哨兵: 上层据此"本帧跳过该框", 与"已决断但归不了类(None)"区分开。
PENDING = object()

# 类别别名表 -> 规范名(真机 mouse / tennis_ball; 另含仿真两类以便回归复用)
DEFAULT_ALIASES = {
    # 真机
    'mouse': 'mouse', 'mice': 'mouse', 'computer mouse': 'mouse',
    'wireless mouse': 'mouse', 'computer_mouse': 'mouse', '鼠标': 'mouse',
    'tennis_ball': 'tennis_ball', 'tennis ball': 'tennis_ball',
    'tennisball': 'tennis_ball', 'tennis': 'tennis_ball',
    'sports ball': 'tennis_ball', 'ball': 'tennis_ball', '网球': 'tennis_ball',
    # 仿真
    'green_block': 'green_block', 'green': 'green_block', '绿': 'green_block',
    'yellow_block': 'yellow_block', 'yellow': 'yellow_block', '黄': 'yellow_block',
}

DEFAULT_PROMPT = (
    '这是一张俯视桌面照片中被检测框出的物体。它属于以下哪一类？'
    '只能从这些名字里选一个：mouse, tennis_ball。'
    '若无法确定，回答 unknown。只输出一个词，不要解释。'
)


def normalize_class(text, allowed=None, aliases=None):
    """把 VLM 自由文本归一化到闭集类别名。归不了 -> None(绝不猜)。

    取"最长命中别名"，避免 'ball' 抢在 'tennis ball' 之前。
    """
    if text is None:
        return None
    t = str(text).strip().lower()
    if not t:
        return None
    t = re.sub(r'[\`\'"*。，,．.：:；;!！?？\s]+', ' ', t).strip()
    al = dict(DEFAULT_ALIASES)
    if aliases:
        al.update({str(k).lower(): v for k, v in aliases.items()})
    if t in al:
        cand = al[t]
    else:
        hits = [(k, v) for k, v in al.items() if k in t]
        if not hits:
            return None
        cand = max(hits, key=lambda kv: len(kv[0]))[1]
    if allowed and cand not in allowed:
        return None
    return cand


def sanity_gate(box, img_shape, *, min_area=300, max_area=0,
                min_ar=0.25, max_ar=4.0, allow_border=False):
    """几何 sanity: 太小/太大/过于细长/顶到画面边缘 -> False(不当真物体, 不问 VLM)。"""
    try:
        x, y, w, h = (float(v) for v in box[:4])
    except Exception:
        return False
    if w <= 1 or h <= 1:
        return False
    area = w * h
    if area < float(min_area):
        return False
    if max_area and area > float(max_area):
        return False
    ar = w / h
    if ar < float(min_ar) or ar > float(max_ar):
        return False
    if not allow_border and img_shape is not None and len(img_shape) >= 2:
        H, W = int(img_shape[0]), int(img_shape[1])
        if x <= 0 or y <= 0 or x + w >= W - 1 or y + h >= H - 1:
            return False
    return True


class NullArbiter:
    """不配置时的默认: 永远 PENDING 之外地"无意见" -> 返回 (None, '')。

    注意: 上层只有在 arbiter 不为 None 时才走分层路由, 所以 NullArbiter 主要用于测试。
    """
    name = 'null'

    def __call__(self, frame, box):
        return None, ''


class CallableArbiter:
    """把 callable(frame, box) -> (class_text, raw) 包成 arbiter, 带异常兜底。"""

    def __init__(self, fn, name='callable'):
        self.fn, self.name = fn, name

    def __call__(self, frame, box):
        try:
            r = self.fn(frame, box)
        except Exception as e:
            return None, 'arbiter_error: %s' % e
        if isinstance(r, (tuple, list)):
            if not r:
                return None, ''
            return r[0], (r[1] if len(r) > 1 else '')
        return r, ''


class HttpJsonArbiter:
    """HTTP 适配器, 支持三种报文风格(由 payload_style 选):

      'ollama'(真机默认): Ollama **原生** /api/chat。
                      ⚠ 图片收**原始 base64**(不能带 `data:image/jpeg;base64,` 前缀),
                        放 messages[].images 数组; 必须 stream=false;
                        返回读 message.content。
                      ⚠ 不支持的图片格式会让 ollama **挂住而不是报错** -> 统一发 JPEG。
      'openai'      : OpenAI 兼容 /chat/completions —— LM Studio、Ollama 的 /v1、vLLM 通用;
                      图片以 data:image/jpeg;base64,... 放 image_url; 读 choices[0].message.content。
      'raw'         : 自定义契约 POST {"prompt","image","yolo_class"} ->
                      期望回 {"class":"<词>"} / {"text":...} / 纯字符串。

    三种都把检测器猜测写进 prompt("确认/纠正"式), 返回文本交 normalize_class 归一化。
    """

    name = 'http'

    def __init__(self, url, prompt=DEFAULT_PROMPT, timeout=0.8,
                 max_side=384, jpeg_quality=85, extra=None,
                 payload_style='ollama', model='', system='',
                 max_tokens=8, temperature=0.0):
        self.url = url
        self.prompt = prompt
        self.timeout = float(timeout)
        self.max_side = int(max_side)
        self.jpeg_quality = int(jpeg_quality)
        self.extra = dict(extra or {})
        self.payload_style = str(payload_style or 'ollama').lower()
        self.model = model
        self.system = system
        self.max_tokens = int(max_tokens)
        self.temperature = float(temperature)

    def _encode(self, frame, box):
        import cv2
        x, y, w, h = (int(round(float(v))) for v in box[:4])
        crop = frame[max(0, y):y + h, max(0, x):x + w]
        if crop.size == 0:
            raise RuntimeError('empty crop')
        m = max(crop.shape[:2])
        if self.max_side and m > self.max_side:
            s = self.max_side / float(m)
            crop = cv2.resize(crop, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode('.jpg', crop,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            raise RuntimeError('jpeg encode failed')
        return base64.b64encode(buf.tobytes()).decode('ascii')

    def __call__(self, frame, box, yolo_class=None):
        try:
            b64 = self._encode(frame, box)
        except Exception as e:
            return None, 'encode_error: %s' % e
        prompt = self.prompt
        if yolo_class:
            # 把检测器的猜测一并给 VLM -> "确认/纠正"式提问, 比开放式提问稳得多。
            prompt = '%s（检测器猜测是 %s，请确认或纠正）' % (self.prompt, yolo_class)
        payload = dict(self.extra)
        if self.payload_style == 'ollama':
            # Ollama 原生 /api/chat: images 收【原始 base64】, 不能带 data: 前缀
            payload.update({
                'model': self.model or 'local-model',
                'messages': [{'role': 'user', 'content': prompt,
                              'images': [b64]}],
                'stream': False,
                'options': {'temperature': self.temperature,
                            'num_predict': self.max_tokens},
            })
        elif self.payload_style == 'openai':
            msgs = []
            if self.system:
                msgs.append({'role': 'system', 'content': self.system})
            msgs.append({'role': 'user', 'content': [
                {'type': 'text', 'text': prompt},
                {'type': 'image_url',
                 'image_url': {'url': 'data:image/jpeg;base64,' + b64}},
            ]})
            payload.update({'model': self.model or 'local-model',
                            'messages': msgs,
                            'max_tokens': self.max_tokens,
                            'temperature': self.temperature,
                            'stream': False})
        else:
            payload.update({'prompt': prompt, 'image': b64,
                            'yolo_class': yolo_class or ''})
        try:
            req = urllib.request.Request(
                self.url, data=json.dumps(payload).encode('utf-8'),
                headers={'Content-Type': 'application/json'}, method='POST')
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                body = json.loads(resp.read().decode('utf-8', 'ignore'))
        except Exception as e:
            return None, 'http_error: %s' % e
        if isinstance(body, dict):
            if self.payload_style == 'ollama':
                try:
                    txt = (body.get('message') or {}).get('content') or ''
                except Exception:
                    txt = ''
                return txt, json.dumps(body, ensure_ascii=False)[:300]
            if self.payload_style == 'openai':
                try:
                    txt = body['choices'][0]['message']['content'] or ''
                except Exception:
                    txt = ''
                return txt, json.dumps(body, ensure_ascii=False)[:300]
            txt = body.get('class') or body.get('text') or body.get('answer') or ''
            return txt, json.dumps(body, ensure_ascii=False)[:300]
        return str(body), str(body)[:300]


class AsyncArbiter:
    """把同步 arbiter 包成"提交-取回": 同一框首次出现提交异步任务并返回 PENDING,
    后续帧取回结果。避免阻塞图像回调(pitfall 4)。

    取回时传给 inner 的是**已裁剪**的框图, box 用 (0,0,w,h)。
    """

    def __init__(self, inner, max_workers=1, max_inflight=64):
        from concurrent.futures import ThreadPoolExecutor
        self.inner = inner
        self.name = 'async(%s)' % getattr(inner, 'name', 'arbiter')
        self._pool = ThreadPoolExecutor(max_workers=max_workers)
        self._jobs = {}
        self._max_inflight = int(max_inflight)

    @staticmethod
    def _key(box):
        return tuple(int(round(float(v))) for v in box[:4])

    def __call__(self, frame, box, yolo_class=None):
        k = self._key(box)
        fut = self._jobs.get(k)
        if fut is None:
            x, y, w, h = k
            crop = frame[max(0, y):y + h, max(0, x):x + w].copy()
            if crop.size == 0:
                return None, 'empty_crop'
            if len(self._jobs) >= self._max_inflight:
                for kk in [k2 for k2, f in self._jobs.items() if f.done()][:16]:
                    self._jobs.pop(kk, None)

            def _job(_c=crop, _yc=yolo_class):
                _b = (0, 0, _c.shape[1], _c.shape[0])
                try:
                    return self.inner(_c, _b, _yc)      # 新式: 带 yolo_class
                except TypeError:
                    return self.inner(_c, _b)           # 旧式: 只有 frame+box

            self._jobs[k] = self._pool.submit(_job)
            return PENDING, 'submitted'
        if not fut.done():
            return PENDING, 'pending'
        self._jobs.pop(k, None)
        try:
            return fut.result()
        except Exception as e:
            return None, 'arbiter_error: %s' % e

    def shutdown(self):
        try:
            self._pool.shutdown(wait=False)
        except Exception:
            pass


ACCEPTED_SCORE_OK = True  # 供外部 flag 校验用的占位常量(语义见模块 docstring 第 2 条)
