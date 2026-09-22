# -*- coding: utf-8 -*-
"""现场产物打包 —— 把上机跑出来的东西收成一个包, 方便带回联网的机器上分析。

断网作业时最亏的是"跑完了但东西没带回来"。这条命令把该带的都收齐并打 tar.gz。

用法(在 WSL 内, 上机结束后跑):
    python3 ~/arm_grasp_ws/src/arm_grasp_sim/scripts/collect_logs.py
    python3 .../collect_logs.py --tag first_run --extra ~/points.json

收什么
------
  ~/classify_real_logs/**           真机 CSV(含 align/vloop/slip/base_offset/grasp_x 行)
  /tmp/cam_probe/**                 probe_real_camera 存的样张
  /tmp/*.log, ~/*.log, ~/*.png      其他现场日志/截图
  points.json                       标定对应点(当前目录与 home 都找)
  清单 MANIFEST.txt                 文件清单 + 时间 + git 状态 + ros2 快照
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys
import time

WS = os.path.expanduser('~/arm_grasp_ws')


def run(cmd, timeout=20):
    try:
        r = subprocess.run(['bash', '-lc', cmd], capture_output=True, text=True,
                           timeout=timeout)
        return (r.stdout or '') + (r.stderr or '')
    except Exception as e:
        return '(执行失败: %s)' % e


def add_dir(man, dst, src, pat='**/*'):
    if not os.path.isdir(src):
        return 0
    n = 0
    for root, _d, files in os.walk(src):
        for f in files:
            p = os.path.join(root, f)
            rel = os.path.relpath(p, src)
            out = os.path.join(dst, os.path.basename(src.rstrip('/')), rel)
            os.makedirs(os.path.dirname(out), exist_ok=True)
            try:
                shutil.copy2(p, out)
                man.append('%10d  %s' % (os.path.getsize(p), os.path.relpath(out, dst)))
                n += 1
            except Exception as e:
                man.append('     FAIL  %s (%s)' % (p, e))
    return n


def add_globs(man, dst, patterns):
    n = 0
    for pat in patterns:
        for p in glob.glob(os.path.expanduser(pat)):
            if not os.path.isfile(p):
                continue
            out = os.path.join(dst, 'misc', os.path.basename(p))
            os.makedirs(os.path.dirname(out), exist_ok=True)
            try:
                shutil.copy2(p, out)
                man.append('%10d  %s' % (os.path.getsize(p), os.path.relpath(out, dst)))
                n += 1
            except Exception as e:
                man.append('     FAIL  %s (%s)' % (p, e))
    return n


def main():
    ap = argparse.ArgumentParser(description='现场产物打包')
    ap.add_argument('--tag', default=time.strftime('%m%d_%H%M'))
    ap.add_argument('--out', default='')
    ap.add_argument('--extra', nargs='*', default=[],
                    help='额外要带的文件(如 points.json)')
    a = ap.parse_args()

    stamp = time.strftime('%Y-%m-%d %H:%M:%S')
    base = a.out or os.path.expanduser('~/exp3_bundle_%s' % a.tag)
    if os.path.isdir(base):
        base = base + '_%d' % int(time.time())
    os.makedirs(base, exist_ok=True)
    man = []
    print('打包到: %s' % base)

    n1 = add_dir(man, base, os.path.expanduser('~/classify_real_logs'))
    n2 = add_dir(man, base, '/tmp/cam_probe')
    n3 = add_dir(man, base, os.path.expanduser('~/classify_logs'))
    pats = ['/tmp/*.log', '/tmp/*.png', '/tmp/*.json',
            '~/points.json', '~/calib*.png', '~/exp3_*.log']
    pats += [os.path.expanduser(x) for x in a.extra]
    n4 = add_globs(man, base, pats)

    snap = os.path.join(base, 'SNAPSHOT.txt')
    with open(snap, 'w', encoding='utf-8') as f:
        f.write('生成时间: %s\n' % stamp)
        f.write('标签: %s\n\n' % a.tag)
        f.write('==== ros2 topic list ====\n')
        f.write(run('source /opt/ros/humble/setup.bash; '
                    'source %s/install/setup.bash 2>/dev/null; '
                    'timeout 12 ros2 topic list' % WS))
        f.write('\n==== git status ====\n')
        f.write(run('cd %s && git status --short' % WS))
        f.write('\n==== git diff --stat ====\n')
        f.write(run('cd %s && git diff --stat' % WS))
        f.write('\n==== 权重/模型 ====\n')
        f.write(run('ls -la %s/models/ 2>/dev/null' % WS))
        f.write('\n==== ollama 模型 ====\n')
        f.write(run('curl -s -m 5 http://127.0.0.1:11434/api/tags || echo "(ollama 不可达)"'))
    man.append('     ----   SNAPSHOT.txt')

    with open(os.path.join(base, 'MANIFEST.txt'), 'w', encoding='utf-8') as f:
        f.write('实验三 现场产物清单    %s\n' % stamp)
        f.write('=' * 72 + '\n')
        f.write('真机 CSV      : %d 个文件\n' % n1)
        f.write('相机样张      : %d 个文件\n' % n2)
        f.write('判空/仿真日志 : %d 个文件\n' % n3)
        f.write('其它散件      : %d 个文件\n' % n4)
        f.write('=' * 72 + '\n')
        f.write('\n'.join(sorted(man)) + '\n')

    tgz = base + '.tar.gz'
    run('tar -czf %s -C %s .' % (tgz, base), timeout=120)
    sz = os.path.getsize(tgz) / 1e6 if os.path.exists(tgz) else 0
    print('=' * 60)
    print('共收集 %d 个文件' % (n1 + n2 + n3 + n4))
    print('目录: %s' % base)
    print('压缩: %s  (%.2f MB)' % (tgz, sz))
    print('=' * 60)
    print('带回联网机器后, 把这个 .tar.gz 拷出来给我, 我就能分析现场数据。')
    print('(U 盘 / 或从 Windows 访问 \\\\wsl$\\Ubuntu-22.04\\home\\underwater\\)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
