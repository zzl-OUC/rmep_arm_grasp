#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验三 低置信度仲裁"值不值得"度量工具。

目的: 在引入 VLM 仲裁层(见 classify_arbiter.py)之前, 用数据回答两个问题:
  (1) 低置信度样本到底占多少?
  (2) VLM 在这些样本上真的比 YOLO 强吗?
只有两个都成立, 才值得多背一个 VLM 运行时 + 不确定源。

三种输入方式(任选):
  A) 已有预测转储:  --preds-csv preds.csv     # 表头: image,gt_cls,pred_cls,conf
  B) 图 + YOLO 标签: --images DIR --labels DIR --names mouse,tennis_ball --weights best.pt
  C) 只有图 + 清单:  --manifest gt.json       # [{"image": "...", "cls": "mouse", "box":[x,y,w,h]}]

可选叠加仲裁对比:  --arbiter-url URL --arbiter-prompt "..." --low 0.15 --high 0.60

用法示例:
  python3 measure_arbiter_value.py --preds-csv /tmp/preds.csv --low 0.15 --high 0.60
  python3 measure_arbiter_value.py --images imgs --labels labels --names mouse,tennis_ball \
      --weights best.pt --arbiter-url http://127.0.0.1:11434/api/arb
"""
import argparse
import csv
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from classify_arbiter import (  # noqa: E402
    PENDING, DEFAULT_PROMPT, HttpJsonArbiter, normalize_class, sanity_gate)


def iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1, y1 = max(ax, bx), max(ay, by)
    x2, y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    iw, ih = max(0.0, x2 - x1), max(0.0, y2 - y1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    return inter / (aw * ah + bw * bh - inter)


def load_preds_csv(path):
    rows = []
    with open(path, newline='', encoding='utf-8') as f:
        for r in csv.DictReader(f):
            rows.append({'image': r.get('image', ''),
                         'gt_cls': (r.get('gt_cls') or '').strip(),
                         'pred_cls': (r.get('pred_cls') or '').strip(),
                         'conf': float(r.get('conf') or 0.0),
                         'box': None})
    return rows


def load_yolo_dirs(images_dir, labels_dir, names):
    """读 YOLO 格式标签 -> [{image, gt_cls, box(px)}]; 需要图片尺寸来还原像素框。"""
    import cv2
    rows = []
    for fn in sorted(os.listdir(labels_dir)):
        if not fn.endswith('.txt'):
            continue
        stem = fn[:-4]
        img_path = None
        for ext in ('.jpg', '.jpeg', '.png', '.bmp'):
            p = os.path.join(images_dir, stem + ext)
            if os.path.exists(p):
                img_path = p
                break
        if img_path is None:
            continue
        img = cv2.imread(img_path)
        if img is None:
            continue
        H, W = img.shape[:2]
        with open(os.path.join(labels_dir, fn), encoding='utf-8') as f:
            for ln in f:
                parts = ln.split()
                if len(parts) < 5:
                    continue
                cid = int(float(parts[0]))
                cx, cy, bw, bh = (float(v) for v in parts[1:5])
                box = ((cx - bw / 2) * W, (cy - bh / 2) * H, bw * W, bh * H)
                rows.append({'image': img_path,
                             'gt_cls': names[cid] if 0 <= cid < len(names) else str(cid),
                             'pred_cls': '',
                             'conf': None,
                             'box': box,
                             'img_path': img_path})
    return rows


def load_manifest(path):
    with open(path, encoding='utf-8') as f:
        raw = json.load(f)
    rows = []
    for it in raw:
        rows.append({'image': it.get('image', ''),
                     'gt_cls': it.get('cls', ''),
                     'pred_cls': '',
                     'conf': None,
                     'box': it.get('box'),
                     'img_path': it.get('image')})
    return rows


def run_yolo(rows, weights, conf_floor=0.01, iou_thr=0.5):
    """对带 box 的行跑 YOLO, 按 IoU 匹配 GT, 填 pred_cls / conf。"""
    from ultralytics import YOLO
    import cv2
    model = YOLO(weights)
    names = model.names if isinstance(model.names, dict) else dict(enumerate(model.names))
    cache = {}
    by_img = {}
    for i, r in enumerate(rows):
        by_img.setdefault(r['img_path'], []).append(i)
    for img_path, idxs in by_img.items():
        img = cache.get(img_path)
        if img is None:
            img = cv2.imread(img_path)
            cache[img_path] = img
        if img is None:
            continue
        res = model(img, conf=conf_floor, verbose=False)[0]
        preds = []
        for b in res.boxes:
            xy = b.xyxy[0].tolist()
            preds.append(((xy[0], xy[1], xy[2] - xy[0], xy[3] - xy[1]),
                          names.get(int(b.cls[0]), str(int(b.cls[0]))),
                          float(b.conf[0])))
        used = set()
        for i in idxs:
            gt = rows[i]['box']
            best, bi = 0.0, -1
            for j, (pb, _c, _s) in enumerate(preds):
                if j in used:
                    continue
                v = iou(gt, pb)
                if v > best:
                    best, bi = v, j
            if bi >= 0 and best >= iou_thr:
                used.add(bi)
                rows[i]['pred_cls'] = preds[bi][1]
                rows[i]['conf'] = preds[bi][2]
            else:
                rows[i]['pred_cls'] = '<miss>'
                rows[i]['conf'] = 0.0
    return rows


def analyze(rows, low, high, allowed=None):
    n = len(rows)
    if n == 0:
        print('没有样本')
        return None
    hist = {}
    band_hit = band_n = band_ok = 0
    outside_n = outside_ok = 0
    ok_all = 0
    for r in rows:
        c = float(r.get('conf') or 0.0)
        b = round(min(c, 0.999) * 10) / 10.0
        hist[b] = hist.get(b, 0) + 1
        ok = (r['pred_cls'] == r['gt_cls'])
        ok_all += int(ok)
        if low <= c < high:
            band_n += 1
            band_ok += int(ok)
            band_hit += 1
        else:
            outside_n += 1
            outside_ok += int(ok)
    rep = {
        'n': n, 'acc_all': ok_all / n,
        'hist': dict(sorted(hist.items())),
        'band_n': band_n, 'band_frac': band_n / n,
        'band_acc': (band_ok / band_n) if band_n else None,
        'outside_n': outside_n,
        'outside_acc': (outside_ok / outside_n) if outside_n else None,
    }
    return rep


def arbiter_eval(rows, low, high, url, prompt, timeout, allowed):
    arb = HttpJsonArbiter(url, prompt=prompt, timeout=timeout)
    flips_gain = flips_loss = 0
    arb_n = arb_ok = 0
    details = []
    for r in rows:
        c = float(r.get('conf') or 0.0)
        if not (low <= c < high) or not r.get('box') or not r.get('img_path'):
            continue
        import cv2
        img = cv2.imread(r['img_path'])
        if img is None:
            continue
        box = r['box']
        if not sanity_gate(box, img.shape, min_area=int(r.get('min_area', 300))):
            details.append((r['image'], 'gated', ''))
            continue
        txt, raw = arb(img, box)
        if txt is PENDING:
            continue
        mapped = normalize_class(txt, allowed=allowed)
        arb_n += 1
        y_ok = (r['pred_cls'] == r['gt_cls'])
        a_ok = (mapped == r['gt_cls'])
        arb_ok += int(a_ok)
        if (not y_ok) and a_ok:
            flips_gain += 1
        if y_ok and (not a_ok):
            flips_loss += 1
        details.append((r['image'], r['gt_cls'], '%s->%s(raw=%s)' % (r['pred_cls'], mapped, raw[:60])))
    return {'arb_n': arb_n, 'arb_ok': arb_ok,
            'arb_acc': (arb_ok / arb_n) if arb_n else None,
            'flips_gain': flips_gain, 'flips_loss': flips_loss,
            'details': details}


def verdict(rep, ar=None, low=0.15, high=0.60):
    lines = []
    lines.append('样本数 N=%d, 整体 YOLO 正确率=%.1f%%' % (rep['n'], rep['acc_all'] * 100))
    lines.append('置信度分布: %s' % rep['hist'])
    lines.append('仲裁带内(%.2f<=conf<%.2f): %d 条(占 %.1f%%), 带内 YOLO 正确率=%s'
                 % (low, high, rep['band_n'], rep['band_frac'] * 100,
                    ('%.1f%%' % (rep['band_acc'] * 100)) if rep['band_acc'] is not None else 'n/a'))
    lines.append('带外: %d 条, 正确率=%s'
                 % (rep['outside_n'],
                    ('%.1f%%' % (rep['outside_acc'] * 100)) if rep['outside_acc'] is not None else 'n/a'))
    if ar:
        lines.append('VLM 在带内: 评估 %d 条, 正确率=%s, 救回 %d 条 / 弄错 %d 条 (净 %+d)'
                     % (ar['arb_n'],
                        ('%.1f%%' % (ar['arb_acc'] * 100)) if ar['arb_acc'] is not None else 'n/a',
                        ar['flips_gain'], ar['flips_loss'],
                        ar['flips_gain'] - ar['flips_loss']))
    # 判据(显式, 可反驳)
    if rep['band_frac'] < 0.05:
        lines.append('结论: 【不值得】仲裁带样本占比 %.1f%% < 5%%, 引入 VLM 的运行时代价换不来收益。'
                     % (rep['band_frac'] * 100))
    elif rep['band_acc'] is not None and rep['band_acc'] >= 0.9:
        lines.append('结论: 【不值得】带内 YOLO 正确率已 >= 90%%, 几乎没有留给 VLM 的空间。')
    elif ar is None:
        lines.append('结论: 【待定】带内样本量/错误率值得关注, 但还没跑 VLM 对比。'
                     '补 --arbiter-url 后再判。')
    elif (ar['flips_gain'] - ar['flips_loss']) <= 0:
        lines.append('结论: 【不值得】VLM 净增益 <= 0 (救回 %d / 弄错 %d)。'
                     % (ar['flips_gain'], ar['flips_loss']))
    else:
        lines.append('结论: 【值得, 但先看样本量】VLM 净增益 %+d 条。'
                     '建议按整体验收口径复跑两次, 确认不破坏"连续两次一致"。'
                     % (ar['flips_gain'] - ar['flips_loss']))
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--preds-csv')
    ap.add_argument('--images')
    ap.add_argument('--labels')
    ap.add_argument('--manifest')
    ap.add_argument('--names', default='mouse,tennis_ball')
    ap.add_argument('--weights')
    ap.add_argument('--low', type=float, default=0.15)
    ap.add_argument('--high', type=float, default=0.60)
    ap.add_argument('--arbiter-url')
    ap.add_argument('--arbiter-prompt', default=DEFAULT_PROMPT)
    ap.add_argument('--arbiter-timeout', type=float, default=2.0)
    ap.add_argument('--allowed', default='')
    args = ap.parse_args()

    names = [s.strip() for s in args.names.split(',') if s.strip()]
    allowed = set(s.strip() for s in args.allowed.split(',') if s.strip()) or None

    if args.preds_csv:
        rows = load_preds_csv(args.preds_csv)
    elif args.images and args.labels:
        rows = load_yolo_dirs(args.images, args.labels, names)
        if args.weights:
            rows = run_yolo(rows, args.weights)
        else:
            print('!! 未给 --weights, 只有 GT 没有预测; 请用 --preds-csv 或补 --weights')
            return 2
    elif args.manifest:
        rows = load_manifest(args.manifest)
        if args.weights:
            rows = run_yolo(rows, args.weights)
    else:
        print('!! 需要 --preds-csv / (--images + --labels) / --manifest 之一')
        return 2

    rep = analyze(rows, args.low, args.high, allowed)
    if rep is None:
        return 1
    ar = None
    if args.arbiter_url:
        ar = arbiter_eval(rows, args.low, args.high, args.arbiter_url,
                          args.arbiter_prompt, args.arbiter_timeout, allowed)
    print(verdict(rep, ar, args.low, args.high))
    if ar and ar['details']:
        print('\n带内逐条(前 30):')
        for d in ar['details'][:30]:
            print('  ', d)
    return 0


if __name__ == '__main__':
    sys.exit(main())
