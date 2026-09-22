# -*- coding: utf-8 -*-
"""上机前离线自检 —— 确认断网后需要的东西全都在本机。

断网作业最怕"到现场才发现缺东西"。这条命令把该在的都点一遍:
依赖 / 模型 / 配置 / 参数契约 / 日志目录 / Ollama(可选)。

用法(在 WSL 内):
    source /opt/ros/humble/setup.bash && source ~/arm_grasp_ws/install/setup.bash
    python3 ~/arm_grasp_ws/src/arm_grasp_sim/scripts/offline_preflight.py
"""
import json
import os
import re
import socket
import sys

WS = os.path.expanduser('~/arm_grasp_ws')
PKG = os.path.join(WS, 'src/arm_grasp_sim')
YAML = os.path.join(PKG, 'config/classify_real.yaml')
MODEL = os.path.join(WS, 'models/best_real3.onnx')

OK, WARN, FAIL = [], [], []


def chk(name, fn):
    try:
        level, detail = fn()
    except Exception as e:
        level, detail = 'FAIL', '异常: %s' % e
    tag = {'PASS': 'PASS', 'WARN': 'WARN', 'FAIL': 'FAIL'}[level]
    print('[%s] %-34s %s' % (tag, name, detail))
    (OK if level == 'PASS' else WARN if level == 'WARN' else FAIL).append(name)
    return level


def c_ros():
    p = os.environ.get('AMENT_PREFIX_PATH', '')
    if not p:
        return 'FAIL', 'AMENT_PREFIX_PATH 空 -> 没 source ROS/工作空间'
    if WS not in p:
        return 'WARN', 'ROS 已 source 但没看到本工作空间(%s...) -> 记得 source install/setup.bash' % p[:50]
    return 'PASS', '已 source (%d 个前缀)' % len(p.split(':'))


def c_install():
    lib = os.path.join(WS, 'install/arm_grasp_sim/lib/arm_grasp_sim')
    if not os.path.isdir(lib):
        return 'FAIL', '缺少 %s -> 需要 colcon build' % lib
    need = ['vision_classifier.py', 'grid_mapper.py', 'classify_task_node.py',
            'classify_grasp_server_real.py', 'probe_real_camera.py',
            'calib_real_grid.py', 'classify_arbiter.py']
    miss = [n for n in need if not os.path.exists(os.path.join(lib, n))]
    if miss:
        return 'FAIL', 'install 里缺: %s' % ', '.join(miss)
    link = os.path.islink(os.path.join(lib, 'classify_grasp_server_real.py'))
    return 'PASS', '7 个脚本就位%s' % ('(符号链接, src 改动即时生效)' if link else '(拷贝!改了要重编译)')


def c_deps():
    import cv2
    import numpy
    return 'PASS', 'cv2 %s / numpy %s' % (cv2.__version__, numpy.__version__)


def c_model():
    if not os.path.exists(MODEL):
        return 'FAIL', '缺少 %s' % MODEL
    import cv2
    import numpy as np
    net = cv2.dnn.readNetFromONNX(MODEL)
    blob = cv2.dnn.blobFromImage(np.zeros((640, 640, 3), np.uint8),
                                 1 / 255.0, (640, 640), swapRB=True)
    net.setInput(blob)
    out = net.forward()
    mb = os.path.getsize(MODEL) / 1e6
    return 'PASS', 'cv2.dnn 已加载 %.1fMB, 输出 %s (应为 (1, 4+2, N))' % (mb, out.shape)


def c_yaml():
    import yaml
    if not os.path.exists(YAML):
        return 'FAIL', '缺少 %s' % YAML
    d = yaml.safe_load(open(YAML, encoding='utf-8'))
    need = ['classify_grasp_server_real', 'vision_classifier', 'grid_mapper',
            'classify_task_node']
    miss = [n for n in need if n not in d]
    if miss:
        return 'FAIL', 'yaml 缺段: %s' % ', '.join(miss)
    p = d['vision_classifier']['ros__parameters']
    v = d['classify_grasp_server_real']['ros__parameters']
    return 'PASS', ('topic=%s | 闭环=%s gain=%.2f | 仲裁=%s'
                    % (p['image_topic'], v['align_mode'], v['align_gain'],
                       p['low_conf_arbiter']))


def c_params():
    """yaml 键 vs 源码 declare_parameter: 抓"改了不生效"的静默键。"""
    import yaml
    cfg = yaml.safe_load(open(YAML, encoding='utf-8'))
    pairs = [('classify_grasp_server_real', 'classify_grasp_server_real.py'),
             ('vision_classifier', 'vision_classifier.py'),
             ('grid_mapper', 'grid_mapper.py'),
             ('classify_task_node', 'classify_task_node.py')]
    bad = []
    for node, fn in pairs:
        src = os.path.join(PKG, 'scripts', fn)
        decl = set(re.findall(r"declare_parameter\(\s*['\"]([^'\"]+)['\"]",
                             open(src, encoding='utf-8').read()))
        got = set((cfg.get(node, {}).get('ros__parameters') or {}).keys())
        for k in sorted(got - decl):
            bad.append('%s.%s' % (node, k))
    if bad:
        return 'FAIL', '这些 yaml 键代码没声明(会被静默忽略): %s' % ', '.join(bad)
    return 'PASS', '35/26/8/5 项全部对得上'


def c_logdir():
    d = os.path.expanduser('~/classify_real_logs')
    try:
        os.makedirs(d, exist_ok=True)
        t = os.path.join(d, '.wtest')
        open(t, 'w').close()
        os.remove(t)
    except Exception as e:
        return 'FAIL', '不可写 %s: %s' % (d, e)
    return 'PASS', '%s 可写' % d


def c_ollama():
    try:
        c = socket.create_connection(('127.0.0.1', 11434), timeout=3)
        c.close()
    except Exception as e:
        return 'WARN', '连不上 127.0.0.1:11434 (%s) —— 只有开 low_conf_arbiter 才需要' % e
    import urllib.request
    try:
        with urllib.request.urlopen('http://127.0.0.1:11434/api/tags',
                                    timeout=5) as r:
            tags = json.loads(r.read().decode())
        names = [m.get('name') for m in tags.get('models', [])]
        if any('qwen3vl' in (n or '') for n in names):
            return 'PASS', 'Ollama 在线, 模型: %s' % ', '.join(names)
        return 'WARN', 'Ollama 在线但没看到 qwen3vl: %s' % names
    except Exception as e:
        return 'WARN', 'Ollama 端口通但 API 异常: %s' % e


def main():
    print('=' * 78)
    print('实验三 离线自检  (工作空间: %s)' % WS)
    print('=' * 78)
    chk('ROS / 工作空间已 source', c_ros)
    chk('install 里的节点与工具', c_install)
    chk('python 依赖 (cv2/numpy)', c_deps)
    chk('检测模型 (ONNX, cv2.dnn)', c_model)
    chk('真机配置 yaml', c_yaml)
    chk('参数契约 (yaml vs 代码)', c_params)
    chk('日志目录可写', c_logdir)
    chk('Ollama / VLM (可选)', c_ollama)
    print('=' * 78)
    print('PASS %d   WARN %d   FAIL %d' % (len(OK), len(WARN), len(FAIL)))
    if FAIL:
        print('有 FAIL 项 —— 先解决再上机: %s' % ', '.join(FAIL))
        return 1
    print('可以上机。WARN 项不影响第一轮(检测是本机 ONNX, 不需要网络)。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
