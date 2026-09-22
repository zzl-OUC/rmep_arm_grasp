#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验三 真机一键上机 —— 一条命令跑完：清理 -> 驱动 -> 找相机 -> 起全链 -> 看状态 -> 收尾打包。

    ros2 run arm_grasp_sim run_real.py            # 或 python3 本文件
    python3 run_real.py --first-run               # 首轮免标定试跑(推荐第一次用这个)
    python3 run_real.py --minutes 10              # 跑 10 分钟自动收尾
    python3 run_real.py --no-driver               # 驱动已在别处起过
    python3 run_real.py --topic /robomaster/camera/image_color   # 跳过自动找相机

它替你做的事
------------
  1. 清理上一次残留的节点(精确匹配脚本名, 不会误杀本进程/父终端)
  2. 起 RoboMaster 驱动, 等 /robomaster/odom 出现
  3. **自动发现相机话题与分辨率**(不用你手改 yaml)
  4. 用"临时参数覆盖"起 vision_classifier / grid_mapper / server / task —— 不动源 yaml
  5. 实时状态行: 相机帧率 / 检测到的物体与类别 / 任务状态
  6. Ctrl+C 或到点 -> 关掉全部节点 -> 自动打包现场日志 -> 打印包路径

--first-run 是什么
------------------
  不做任何实测标定: 直接用**仿真网格几何**当粗略分格依据, 并把 cell_tol 放宽。
  标签可能贴错(只影响处理顺序与日志), **不影响抓得准不准** —— 因为横向靠视觉闭环、
  前向靠实测 x(`grasp_x_from_vision`)。目的: 让你今天就先看到"识别 -> 抓"跑起来。
  真要让格子编号与桌面一致, 再按手册 §4 标定一次。
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

WS = os.path.expanduser('~/arm_grasp_ws')
PKG = os.path.join(WS, 'src/arm_grasp_sim')
YAML = os.path.join(PKG, 'config/classify_real.yaml')
TMP_YAML = '/tmp/run_real_params.yaml'
LOGDIR = '/tmp/run_real_logs'

# 仿真网格几何(与 grid_mapper.py 内置 CELLS 一致) —— first-run 当粗标签用
SIM_CELLS = {'cell_1': [0.1785, 0.0650], 'cell_2': [0.1271, 0.1412],
             'cell_3': [0.0460, 0.1844], 'cell_4': [-0.0460, 0.1844],
             'cell_5': [-0.1271, 0.1412], 'cell_6': [-0.1785, 0.0650]}

NODE_SCRIPTS = ['vision_classifier.py', 'grid_mapper.py',
                'classify_grasp_server_real.py', 'classify_task_node.py',
                'classify_grasp_server.py']
KILL_PATTERNS = NODE_SCRIPTS + ['robomaster_ros', 'gzserver', 'run_real.py']
KEEP_PATTERNS = ['run_real.py']          # 自己不算残留


# ---------------------------------------------------------------- 小工具
def say(msg, tag='*'):
    print('[%s] %s' % (tag, msg), flush=True)


def ancestors():
    """自己的全部祖先 PID —— 清理时绝不能碰它们(否则杀掉终端/父 shell)。"""
    out, pid = set(), os.getpid()
    for _ in range(24):
        out.add(pid)
        try:
            with open('/proc/%d/stat' % pid) as f:
                pid = int(f.read().split(') ', 1)[1].split()[1])
        except Exception:
            break
        if pid <= 1:
            break
    return out


def cleanup_stale():
    """精确清理上次残留。返回杀掉的进程列表。"""
    keep = ancestors()
    me = os.getpid()
    killed = []
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        pid = int(d)
        if pid in keep or pid == me:
            continue
        try:
            with open('/proc/%d/cmdline' % pid, 'rb') as f:
                cl = f.read().decode('utf-8', 'ignore').replace('\0', ' ')
        except Exception:
            continue
        if not cl:
            continue
        if any(k in cl for k in KEEP_PATTERNS):
            continue                       # 别把另一个 run_real 也杀了
        if any(p in cl for p in KILL_PATTERNS):
            try:
                os.kill(pid, signal.SIGKILL)
                killed.append('%d:%s' % (pid, cl[:60]))
            except Exception:
                pass
    return killed


def wait_topic(topic, timeout):
    """等某个话题出现。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            r = subprocess.run(['bash', '-lc',
                                'source /opt/ros/humble/setup.bash >/dev/null 2>&1; '
                                'source %s/install/setup.bash >/dev/null 2>&1; '
                                'timeout 8 ros2 topic list' % WS],
                               capture_output=True, text=True, timeout=20)
            if topic in (r.stdout or ''):
                return True
        except Exception:
            pass
        time.sleep(2.0)
    return False


def wait_driver_ready(logdir, odom_timeout=15.0, driver_timeout=30.0):
    # 等驱动就绪。优先等 /robomaster/odom(闭环对齐精度更高);
    # 若硬件未发布 odom(某些固件/配置下 chassis 不发布该话题), 退而等任意
    # /robomaster/ 话题, 确认驱动已起即放行。此时 _heading 按指令角累加、
    # base_offset 不自估, 但视觉闭环横向对齐仍生效(每轮重测物体像素位置)。
    # 返回 (ready, odom_ok)。
    import subprocess as _sp
    if wait_topic("/robomaster/odom", odom_timeout):
        say("驱动就绪 (odom 在线)", "OK")
        return True, True
    say("/robomaster/odom 未出现(%.0fs) -- 改读驱动日志确认机器人真正连上..." % odom_timeout, "!")
    t0 = time.time()
    while time.time() - t0 < driver_timeout:
        # 真正的连接证据: 驱动日志里出现 "Connected" / "Enabled modules"
        # (joint_states/robot_description 等辅助节点话题不能证明机器人连上)
        connected = False
        try:
            dlog = open(os.path.join(logdir, 'driver.log'),
                        encoding='utf-8', errors='ignore').read()
            if 'Enabled modules' in dlog or 'Connected' in dlog:
                connected = True
        except Exception:
            pass
        try:
            r = _sp.run(["bash", "-lc",
                        "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
                        "source %s/install/setup.bash >/dev/null 2>&1; "
                        "timeout 8 ros2 topic list" % WS],
                        capture_output=True, text=True, timeout=20)
            tops = [t.strip() for t in (r.stdout or "").split() if t.strip()]
            rb = [t for t in tops if t.startswith("/robomaster/")]
            if rb and connected:
                say("驱动已在线(机器人已连上, 但本机不发布 odom 话题)。可用话题: %s"
                    % ", ".join(sorted(set(rb))[:6]), "OK")
                return True, False
        except Exception:
            pass
        if not connected and (time.time() - t0) > 12:
            # 已等 >12s 仍无 Connected: 基本可判定机器人没真正连上(不是命名空间问题)
            say("等了 12s+, 驱动日志里仍没有 'Connected'/'Enabled modules' "
                "-- 机器人很可能根本没连上(之前看到的 joint_states 等是辅助节点话题, 不算)。", "!")
            say(">> sta 模式: 小车须与电脑连同一个路由器, 且小车摄像头对准二维码按按钮入网 "
                "(或先 ros2 run robomaster_ros connect <SSID> <密码>)。", "!")
            say(">> 更简单稳妥: 用 USB 线连接小车, 连接模式选 rndis(双击 .bat 按 7 切换)。", "!")
        time.sleep(2.0)
    # 兜底诊断: 打印当前所有可见话题, 看真实命名空间到底是不是 /robomaster/
    try:
        r = _sp.run(["bash", "-lc",
                    "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
                    "source %s/install/setup.bash >/dev/null 2>&1; "
                    "timeout 8 ros2 topic list" % WS],
                    capture_output=True, text=True, timeout=20)
        allt = [t.strip() for t in (r.stdout or "").split() if t.strip()]
        if allt:
            say("可见话题(共 %d 个): %s" % (len(allt), ", ".join(allt[:20])), '!')
            say("-> 上述话题不在 /robomaster/ 下。若前缀是其它(如空/rm_xxx), "
                "需让驱动用 name:=<前缀> 启动, 并同步改 yaml/image_topic 等。", '!')
        else:
            say("ros2 topic list 完全为空 -- 节点间 DDS 发现不通(仍可能多播问题)。"
                "已设 ROS_LOCALHOST_ONLY=1, 若仍空请检查防火墙/网卡。", '!')
    except Exception:
        pass
    return False, False


def spawn(cmd, logname, env):
    os.makedirs(LOGDIR, exist_ok=True)
    f = open(os.path.join(LOGDIR, logname), 'w', encoding='utf-8', errors='ignore')
    p = subprocess.Popen(['bash', '-lc', cmd], stdout=f, stderr=subprocess.STDOUT,
                         preexec_fn=os.setsid, env=env)
    p._logfile = f
    return p


def stop_all(procs):
    for p in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except Exception:
            pass
    time.sleep(2.0)
    for p in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass
    for p in procs:
        try:
            p._logfile.close()
        except Exception:
            pass


# ---------------------------------------------------------------- 相机发现
def find_camera(timeout=20.0, prefer=''):
    """订阅所有 Image 话题, 返回第一个真的出帧的 (话题, 宽, 高, 帧率)。"""
    code = r'''
import sys, time
sys.path.insert(0, %r)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

PREFER = %r
found = {}
class N(Node):
    def __init__(self):
        super().__init__('cam_find')
        self.b = CvBridge()
        names = [n for n, t in self.get_topic_names_and_types()
                 if 'sensor_msgs/msg/Image' in t]
        if PREFER and PREFER in names:
            names = [PREFER] + [n for n in names if n != PREFER]
        print('CAND ' + ','.join(names) if names else 'CAND -', flush=True)
        for n in names:
            self.create_subscription(Image, n, lambda m, n=n: self.cb(m, n), 10)
    def cb(self, m, n):
        d = found.setdefault(n, {'n': 0, 'w': 0, 'h': 0, 't0': None, 't1': None})
        d['n'] += 1
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if d['t0'] is None:
            d['t0'] = t
        d['t1'] = t
        if not d['w']:
            d['w'], d['h'] = m.width, m.height

rclpy.init()
n = N()
t0 = time.time()
while rclpy.ok() and time.time() - t0 < %f:
    rclpy.spin_once(n, timeout_sec=0.2)
    if found and all(v['w'] for v in found.values()):
        break
n.destroy_node()
rclpy.shutdown()
for k, v in found.items():
    if v['w']:
        dur = (v['t1'] - v['t0']) if v['t0'] and v['t1'] else 0
        fps = (v['n'] - 1) / dur if dur > 0.05 else 0
        print('FOUND %%s %%d %%d %%.1f' %% (k, v['w'], v['h'], fps), flush=True)
''' % (PKG + '/scripts', prefer, timeout)
    try:
        r = subprocess.run([sys.executable, '-c', code], capture_output=True,
                           text=True, timeout=timeout + 25)
    except Exception as e:
        say('相机发现失败: %s' % e, '!')
        return None
    cand, found = [], []
    for line in (r.stdout or '').splitlines():
        if line.startswith('CAND '):
            cand = [x for x in line[5:].split(',') if x and x != '-']
        if line.startswith('FOUND '):
            p = line.split()
            found.append((p[1], int(p[2]), int(p[3]), float(p[4])))
    if not found:
        say('候选话题: %s' % (cand or '无'), '!')
        return None
    for k in ([prefer] if prefer else []) + [f[0] for f in found]:
        for f in found:
            if f[0] == k:
                return f
    return found[0]


# ---------------------------------------------------------------- 参数覆盖
def build_params(topic, w, h, first_run, odom_ok=True):
    import yaml
    cfg = yaml.safe_load(open(YAML, encoding='utf-8'))
    v = cfg['vision_classifier']['ros__parameters']
    g = cfg['grid_mapper']['ros__parameters']
    t = cfg['classify_task_node']['ros__parameters']
    # 无 odom 时关闭依赖 odom 的两项(闭环横向对齐不依赖 odom, 仍生效)
    srv = cfg['classify_grasp_server_real']['ros__parameters']
    srv['heading_from_odom'] = bool(odom_ok)
    srv['base_offset_auto'] = bool(odom_ok)
    v['image_topic'] = topic
    g['image_width'], g['image_height'] = int(w), int(h)
    if first_run:
        g['cells'] = json.dumps(SIM_CELLS)
        g['cell_tol'] = 0.12          # 放宽: 容忍几何偏差, 只要物体能拿到标签
        t['scan_stable'] = 6.0
    with open(TMP_YAML, 'w', encoding='utf-8') as f:
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)
    return TMP_YAML


# ---------------------------------------------------------------- 状态
def grab_sample(topic, out_png, grid_png=''):
    """从话题抓一帧存 PNG(可另存一张带坐标网格的, 供离线选点/读数)。"""
    code = r'''
import sys, time
sys.path.insert(0, %r)
import cv2, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
got = {}
class N(Node):
    def __init__(self):
        super().__init__('grab')
        self.b = CvBridge()
        self.create_subscription(Image, %r, self.cb, 10)
    def cb(self, m):
        if 'f' in got:
            return
        try:
            got['f'] = self.b.imgmsg_to_cv2(m, 'bgr8')
        except Exception as e:
            got['err'] = str(e)
rclpy.init(); n = N()
t0 = time.time()
while rclpy.ok() and 'f' not in got and time.time() - t0 < 15:
    rclpy.spin_once(n, timeout_sec=0.2)
n.destroy_node(); rclpy.shutdown()
if 'f' not in got:
    print('ERR no_frame ' + got.get('err', ''), flush=True); sys.exit(1)
f = got['f']
cv2.imwrite(%r, f)
print('SAVED %%s %%d %%d' %% (%r, f.shape[1], f.shape[0]), flush=True)
if %r:
    g = f.copy()
    h, w = g.shape[:2]
    for x in range(0, w, 50):
        cv2.line(g, (x, 0), (x, h), (120, 120, 120), 1)
        cv2.putText(g, str(x), (x + 2, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 200), 1)
    for y in range(0, h, 50):
        cv2.line(g, (0, y), (w, y), (120, 120, 120), 1)
        cv2.putText(g, str(y), (2, y + 13), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 200), 1)
    cv2.imwrite(%r, g)
    print('GRID %%s' %% %r, flush=True)
''' % (PKG + '/scripts', topic, out_png, out_png, grid_png, grid_png, grid_png)
    r = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True,
                       timeout=40)
    out = (r.stdout or '') + (r.stderr or '')
    return out.strip()


def status_loop(procs, minutes, topic):
    """状态监视(单独进程), 输出写到 LOGDIR/status.log 由主循环转出来。

    注意: 代码写成**临时 .py 文件**再执行 —— 不要用 `python3 -c <字符串>`,
    因为把多行代码塞进 argv 时换行会被转义, 到那边就不是合法 Python 了。
    """
    code = r'''
import sys, time, json
sys.path.insert(0, %r)
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String

st = {'det': [], 'grid': [], 'state': '-'}
class N(Node):
    def __init__(self):
        super().__init__('run_real_status')
        self.create_subscription(Detection2DArray, '/detections', self.d, 10)
        self.create_subscription(String, '/grid_detections', self.g, 10)
        self.create_subscription(String, '/grasp_state', self.s, 10)
    def d(self, m):
        st['det'] = [(x.results[0].hypothesis.class_id,
                      round(float(x.results[0].hypothesis.score), 2))
                     for x in m.detections if x.results]
    def g(self, m):
        try:
            st['grid'] = [(i.get('grid'), i.get('cls')) for i in json.loads(m.data)]
        except Exception:
            pass
    def s(self, m):
        st['state'] = m.data

rclpy.init()
n = N()
t0 = time.time()
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.4)
    print('T+%%4.0fs  检测=%%-30s 分格=%%-30s 任务=%%s' %% (
        time.time() - t0, str(st['det'])[:30], str(st['grid'])[:30], st['state']),
        flush=True)
    time.sleep(3.0)
''' % (PKG + '/scripts')
    src = os.path.join(LOGDIR, 'status_src.py')
    with open(src, 'w', encoding='utf-8') as f:
        f.write(code)
    cmd = ('source /opt/ros/humble/setup.bash; source %s/install/setup.bash; '
           'exec %s -u %s' % (WS, sys.executable, src))
    return spawn(cmd, 'status.log', os.environ.copy())


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description='实验三 真机一键上机')
    ap.add_argument('--conn', default='ap', choices=['ap', 'sta', 'rndis'],
                    help='驱动连接方式(默认 ap=连小车热点)')
    ap.add_argument('--no-driver', action='store_true', help='驱动已在别处启动')
    ap.add_argument('--topic', default='', help='直接指定相机话题(跳过自动发现)')
    ap.add_argument('--minutes', type=float, default=0.0, help='跑这么久后自动收尾')
    ap.add_argument('--first-run', action='store_true',
                    help='首轮免标定试跑: 仿真网格几何 + 放宽容差')
    ap.add_argument('--no-bundle', action='store_true', help='退出时不打包日志')
    ap.add_argument('--probe-only', action='store_true',
                    help='只探相机并存样张, 不起任务节点')
    a = ap.parse_args()

    os.makedirs(LOGDIR, exist_ok=True)
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'
    # WSL / 容器里 DDS 多播发现常失败, 强制 ROS 只走本地回环, 所有节点同机运行
    env['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    say('实验三 真机一键上机   连接=%s  首轮模式=%s' % (a.conn, a.first_run))
    killed = cleanup_stale()
    say('清理残留: %s' % (', '.join(killed) if killed else '无'))

    procs = []
    try:
        odom_ok = True   # 默认假定 odom 在线; 连上后若确认缺失改为 False
        # ---- 1. 驱动 ----
        if not a.no_driver:
            say('启动 RoboMaster 驱动 (conn_type:=%s) ...' % a.conn)
            procs.append(spawn(
                'source /opt/ros/humble/setup.bash; source %s/install/setup.bash; '
                'exec ros2 launch robomaster_ros main.launch model:=ep conn_type:=%s name:=robomaster video_raw:=1 video_h264:=0 video_ffmpeg:=0'
                % (WS, a.conn), 'driver.log', env))
            # 相机原生解压图发布在 /robomaster/camera/image_color(走 SDK 内置解码器, 不依赖 ffmpeg),
            # 直接用即可, 无需额外 h264_decoder 节点。video_raw:=1 已强制开启推流(绕过 ON_DEMAND 订阅检测)。
            ready, odom_ok = wait_driver_ready(LOGDIR)
            if not ready:
                say('等不到任何 /robomaster/ 话题 -- 检查小车是否开机、是否连着它的热点、'
                    '驱动命名空间是否为 /robomaster', '!')
                say('驱动日志尾部:', '!')
                print(open(os.path.join(LOGDIR, 'driver.log'),
                           encoding='utf-8', errors='ignore').read()[-800:])
                return 2
            if not odom_ok:
                say('注意: 本机未发布 /robomaster/odom -- 闭环改用"指令角累加 heading"'
                    '且 base_offset 不自估; 视觉横向对齐仍生效。', '!')
        else:
            say('跳过驱动(按 --no-driver)', 'OK')

        # ---- 2. 相机 ----
        if a.topic:
            say('优先用你指定的话题: %s' % a.topic)
            cam = find_camera(prefer=a.topic)
            if cam and cam[0] != a.topic:
                say('指定话题没出帧, 自动换成了 %s' % cam[0], '!')
        else:
            say('自动发现相机话题(最多 20s)... 优先相机原生解压的 camera/image_color')
            cam = find_camera(prefer='/robomaster/camera/image_color')
        if not cam:
            say('没找到任何出帧的相机话题 —— 检查驱动/相机', '!')
            # 精准诊断: AP 模式 SDK 不支持相机视频流
            try:
                dlog = open(os.path.join(LOGDIR, 'driver.log'),
                            encoding='utf-8', errors='ignore').read()
            except Exception:
                dlog = ''
            # AP 模式(连小车自己的热点)现已支持相机视频流(SDK 已打 is->== 补丁)。
            if 'conn_type:ap is not supported' in dlog:
                say('>> 驱动日志仍报 AP 视频不支持 —— 说明 SDK 的 is->== 补丁未生效'
                    '(可能重装过 robomaster)。请重跑 patch_sdk_camera.py 再试。', '!')
            else:
                say('>> AP 模式视频流已支持。若仍无帧, 排查: '
                    '1) driver.log 有无 Connected / Enabled modules(确认小车真连上); '
                    '2) 用菜单 [4] 抓样张确认相机出图; '
                    '3) 机器人相机是否被遮挡/未启动。', '!')
            say('相机相关话题:', '!')
            subprocess.run(['bash', '-lc',
                            'source /opt/ros/humble/setup.bash; source %s/install/setup.bash; '
                            'ros2 topic list | grep -i camera' % WS])
            return 3
        topic, w, h, fps = cam
        say('相机: %s  %dx%d  ~%.1f fps' % (topic, w, h, fps), 'OK')

        if a.probe_only:
            png = os.path.join(LOGDIR, 'camera_sample.png')
            gp = os.path.join(LOGDIR, 'camera_sample_grid.png')
            say('抓样张...')
            out = grab_sample(topic, png, gp)
            print(out, flush=True)
            if os.path.exists(png):
                say('样张: %s' % png, 'OK')
                say('带坐标网格(离线读像素用): %s' % gp, 'OK')
                say('拷到 Windows 桌面: cp %s %s /mnt/c/Users/13907/Desktop/' % (png, gp))
                say('** 打开样张确认能俯视看到整个桌面! 看不到就先调相机, 别往下走 **', '!')
                return 0
            say('抓样张失败', '!')
            return 4

        # ---- 3. 参数覆盖 + 起节点 ----
        y = build_params(topic, w, h, a.first_run, odom_ok)
        say('参数覆盖写入 %s (未改动源 yaml)' % y, 'OK')
        if a.first_run:
            say('首轮模式: cells=仿真几何, cell_tol=0.12 —— 标签可能不准, '
                '但抓取靠视觉闭环 + 实测 x, 不受影响', '!')

        for script, log in [('vision_classifier.py', 'vision.log'),
                            ('grid_mapper.py', 'mapper.log'),
                            ('classify_grasp_server_real.py', 'server.log'),
                            ('classify_task_node.py', 'task.log')]:
            procs.append(spawn(
                'source /opt/ros/humble/setup.bash; source %s/install/setup.bash; '
                'exec ros2 run arm_grasp_sim %s --ros-args --params-file %s'
                % (WS, script, y), log, env))
            time.sleep(0.8)
        say('四个节点已起', 'OK')
        time.sleep(3.0)
        say('--- 节点自检(各日志尾部) ---')
        for log in ['vision.log', 'mapper.log', 'server.log', 'task.log']:
            p = os.path.join(LOGDIR, log)
            tail = ''
            if os.path.exists(p):
                lines = open(p, encoding='utf-8', errors='ignore').read().splitlines()
                tail = ' | '.join([l for l in lines if '就绪' in l or 'ERROR' in l][-2:])
            say('  %-12s %s' % (log, tail[:150] or '(无就绪日志, 见 %s)' % p),
                ' ' if tail else '!')

        # ---- 4. 状态 ----
        procs.append(status_loop(procs, a.minutes, topic))
        say('=' * 72)
        say('开始作业。放物体到桌面即可; Ctrl+C 结束并自动打包日志。')
        if a.minutes:
            say('将在 %.0f 分钟后自动收尾。' % a.minutes)
        say('=' * 72)

        t0 = time.time()
        listened = False
        while True:
            time.sleep(0.5)
            # 把状态输出转出来
            sp = os.path.join(LOGDIR, 'status.log')
            if os.path.exists(sp):
                with open(sp, encoding='utf-8', errors='ignore') as f:
                    lines = f.read().splitlines()
                if lines:
                    last = lines[-1]
                    if last != getattr(main, '_last', None):
                        print(last, flush=True)
                        main._last = last
            for p in procs:
                if p.poll() is not None and 'status' not in str(p.args):
                    say('!! 有进程退出(code=%s): %s' % (p.returncode, str(p.args)[:80]), '!')
            if a.minutes and time.time() - t0 > a.minutes * 60:
                say('到点, 收尾。')
                break
    except KeyboardInterrupt:
        say('收到 Ctrl+C, 收尾。')
    finally:
        say('关闭节点...')
        stop_all(procs)
        cleanup_stale()

    if not a.no_bundle:
        say('打包现场日志...')
        subprocess.run(['bash', '-lc',
                        'source /opt/ros/humble/setup.bash; source %s/install/setup.bash; '
                        'python3 %s/scripts/collect_logs.py --tag run_real --extra %s'
                        % (WS, PKG, LOGDIR)])
    say('全部结束。本次各节点日志在 %s' % LOGDIR)
    return 0


if __name__ == '__main__':
    sys.exit(main())
