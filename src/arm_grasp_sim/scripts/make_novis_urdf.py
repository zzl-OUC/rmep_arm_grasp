#!/usr/bin/env python3
"""生成去掉所有 <visual> 的机械臂 URDF，供 Gazebo spawn 使用。

背景
----
在 WSL + llvmpipe 软件渲染环境下，机械臂 URDF 引用的 .dae visual mesh 会让
Gazebo 的渲染线程崩溃，表现为 /top_camera/image_raw 与 /top_camera/camera_info
在 spawn 机械臂之后完全停止发布（publisher 还在，但帧数为 0），整个视觉链路随之
失效。逐 link 二分确认过：spawn 方块（纯 box geometry）无影响，spawn 带 visual
的机械臂必现。

对策：spawn 时使用本脚本生成的无 visual URDF。collision、inertial、ros2_control
接口、gazebo 插件（含 grasp_fix）全部保留，因此物理与抓取行为完全不变；
robot_state_publisher 仍使用带 visual 的原始 xacro，RViz 中机械臂外观正常。

注意：xacro 必须带 config_path:=<controllers.yaml>，否则 ros2_control 插件的
<parameters> 为空，controller_manager 起不来（表现为 spawner 一直等待
/controller_manager/list_controllers，所有关节状态读成 0）。

用法
----
    python3 make_novis_urdf.py [-i 源xacro] [-c controllers.yaml] [-o 输出路径]
"""

import argparse
import subprocess
import xml.etree.ElementTree as ET

DEFAULT_XACRO = ('/home/underwater/arm_grasp_ws/src/arm_grasp_sim/'
                 'urdf/arm_grasp.urdf.xacro')
DEFAULT_CONFIG = ('/home/underwater/arm_grasp_ws/install/arm_grasp_sim/'
                  'share/arm_grasp_sim/config/controllers.yaml')


def build_urdf(xacro_path, config_path=None):
    """调用 xacro 展开成完整 URDF 文本。"""
    cmd = ['xacro', xacro_path]
    if config_path:
        cmd.append('config_path:=' + config_path)
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0 or not proc.stdout.strip():
        raise RuntimeError('xacro 失败: %s' % proc.stderr.strip()[:400])
    return proc.stdout


def strip_visuals(urdf_text):
    root = ET.fromstring(urdf_text)
    removed = 0
    for link in root.findall('link'):
        for visual in list(link.findall('visual')):
            link.remove(visual)
            removed += 1
    return ET.tostring(root, encoding='unicode'), removed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-i', '--input', default=DEFAULT_XACRO, help='源 xacro')
    ap.add_argument('-c', '--config', default=DEFAULT_CONFIG,
                    help='controllers.yaml 路径, 写入 ros2_control 插件参数')
    ap.add_argument('-o', '--output', default='/tmp/arm_novis.urdf')
    args = ap.parse_args()

    text, n = strip_visuals(build_urdf(args.input, args.config))
    with open(args.output, 'w') as f:
        f.write(text)
    print('生成 %s (移除 %d 个 visual)' % (args.output, n))


if __name__ == '__main__':
    main()
