#!/usr/bin/env python3
"""实验三 一键启动: 桌面分类整理仿真（实验要求: 一个 Launch 启动相机/识别/机械臂/任务控制）。

基于已验收的 grasp_sim.launch.py 扩展:
  1. robot_state_publisher
  2. gazebo + table_grid_4c2b.world (6 取物网格 + 2 料盒 + 俯视相机)
  3. 生成机械臂 + 生成方块(按 blocks_preset 场景, 默认绿3/黄3)
  4. ros2_control 控制器组
  5. classify_grasp_server (扩展抓取 action server)
  6. vision_classifier (俯视相机 -> Detection2DArray)
  7. grid_mapper (检测框中心 -> 网格号)
  8. classify_task_node (状态机: 扫描->抓取->分类放置->异常处理->完成)

验收对照(实验要求四):
  - 自动识别并分类 >=2 类物体: green_block / yellow_block -> bin_0 / bin_1
  - >=6 物体, >=5 正确: 默认 blocks_preset=normal 启动即生成 6 块, 状态机循环至全部处理
  - 无人工干预: 全链路自动
  - 异常: 空网格/未识别/不可达/抓取失败, 分类记入日志(见 blocks_preset 注入场景)
  - 日志: ~/classify_logs/task_log.json + /grasp_state + /task_state 话题

异常注入(实验要求四.6) —— 用 launch 参数切换场景, 无需改代码:
  ros2 launch arm_grasp_sim classify_sim.launch.py blocks_preset:=empty_grid
  ros2 launch arm_grasp_sim classify_sim.launch.py blocks_preset:=unknown_obj
  ros2 launch arm_grasp_sim classify_sim.launch.py blocks_preset:=miss_grasp
  ros2 launch arm_grasp_sim classify_sim.launch.py blocks_preset:=unreachable z_grasp:=0.005
"""
import math
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = 'arm_grasp_sim'

# 物体类型 -> URDF 文件名
BLOCK_URDF = {
    'tennis': 'tennis.sdf',       # 网球(荧光黄绿; SDF 版, 含扭转摩擦防自滚)
    'bottle': 'bottle.urdf',      # 矿泉水瓶(高饱和蓝矮瓶)
    'red': 'block_red.urdf',      # 仅用于"未识别物体"异常注入(红色不属已知类别)
}

# 物体 spawn 高度(z 为模型原点/几何中心高): 桌面顶 0.025 + 物体半高
#   tennis: 球心 = 0.025 + r(0.033) = 0.058
#   bottle: 瓶心 = 0.025 + 半高(0.05) = 0.075
#   red(方块): 块心 = 0.025 + 0.02 = 0.045
SPAWN_Z = {'tennis': 0.062, 'bottle': 0.079, 'red': 0.045}  # 贴合实测静止位姿(0.0619/0.0789), 消除穿插弹出

# 6 个物体初始位姿(网格中心) —— 全向布局:
#   cell_1 正前球 / cell_2 左前瓶 / cell_3 右前瓶 / cell_4 正后瓶 / cell_5 左后球 / cell_6 右后球
# 布局原则: 网格与料盒最小间隙 = 对角格(±0.1663,±0.1663) 到正左/正右料盒(0,±0.21) ≈ 0.172m,
#   远大于"夹爪半宽 0.03 + 盒半对角 0.076"所需最小间隙, 下降抓取不会撞料盒壁。
_NORMAL = [
    # (x, y, class)  -- r=0.225 全向布局(按需求外移 4cm; 抓取 TCP d = r+0.066 = 0.291)。
    #   上一次同样外移到 0.225 时, 4 个对角格全部在 DESCEND_MID 中止、从未进入 GRASP。
    #   真因不是 IK/力矩/时间, 而是**张开指盒斜扫进料盒墙**: 开口 100mm 时指盒外宽
    #   124mm, 对角格指盒最大 |y| = 0.7071*r + 0.0596 = 0.2188, 而料盒近墙占
    #   y 0.1875~0.1925; 侧墙内沿 x=0.1400 而指盒外沿 0.1397, 只剩 0.3mm。
    #   顶到静态墙 => 速度指令非零而关节不动 => 段超时。正前/正后格指盒只在 |y|=0.062,
    #   离墙远, 所以那两格恒成功 —— 成败按"方位"分而不是按"球/瓶"分。
    #   解法(用户选定"削矮"): 近墙与侧墙顶面 z 0.075 -> 0.040, 低于指盒最低点
    #   (球 0.053-0.0106=0.0424 / 瓶 0.0594), 让指盒从墙上方跨过; 远墙保持 0.075。
    #   改为高度避让后不再依赖 xy 余量, 对实测 10.8mm 定位偏差不敏感。
    #   半径余量: 离线 IK 扫描显示 r=0.245(d=0.311) 时网球航点已不可达, 故 0.225 是上限。
    ( 0.2250,  0.0000, "tennis"),   # cell_1 正前球 -> bin_1
    ( 0.1591,  0.1591, "bottle"),   # cell_2 左前瓶 -> bin_0
    ( 0.1591, -0.1591, "bottle"),   # cell_3 右前瓶 -> bin_0
    (-0.2250,  0.0000, "bottle"),   # cell_4 正后瓶 -> bin_0
    (-0.1591,  0.1591, "tennis"),   # cell_5 左后球 -> bin_1
    (-0.1591, -0.1591, "tennis"),   # cell_6 右后球 -> bin_1
]

# ⚠ 2026-09-18 改注入方式: "把球径向外推到指尖够不到"这条老路已经走不通了, 两点原因:
#   1) 指盒沿夹爪轴前移 10mm 后, 指尖径向范围变成 [d-0.0885, d-0.0435];
#      格外移 5cm 后格心抓取 d=0.235+0.066=0.3013 -> 指尖 [0.1628, 0.2078],
#      球被推到 0.240 时球体径向 [0.207, 0.273], 与指尖只差 0.8mm, 会擦到而不是干净夹空;
#   2) 网球碰撞体已由 box 改为 sphere。box 可以悬挑在格板边缘外, sphere 不行:
#      球心 0.240 已超出 cell_1 格板外缘 0.230, 球会自己滚下格板掉出世界。
#   而"干净夹空"要求球心 > 0.2408(球内缘越过指尖外缘), 格板却只到 0.230 —— 几何上不可能。
# 改为用 z_grasp 注入(与 S4 unreachable 同一套机制, 仍是纯 launch 参数、不改运行期代码):
#   预设放 [cell_1 网球, cell_2 瓶], 配 z_grasp:=0.115 ->
#     网球顶 0.095, 指盒底 = 0.115-0.0106 = 0.1044 > 0.095 -> 闭爪夹空 ✓
#     瓶顶 0.125, 指盒仍能咬住瓶体上部 -> 期望仍能放置 ✓
#   从而得到期望的 "1 grasp_failed + 1 placed(任务不中断)"。
#   (与 S4 的 z_grasp:=0.005 方向相反: 0.005 是低于可达下限报不可达, 0.115 是正常可达但夹空。)
_MISS = (0.2250, 0.0000, 'tennis')

BLOCK_PRESETS = {
    # 正常验收场景: 6 物(网球3/瓶3), 6 网格全满
    'normal': _NORMAL,
    # 只放 4 物 -> cell_5/cell_6 为空网格(期望 2 条 skipped_empty)
    'empty_grid': _NORMAL[:4],
    # cell_4 换成红色方块(H≈0, 不在 tennis 18-45 / bottle 95-130 内)
    #   -> 期望 1 条 skipped_unknown, 其余 5 物正常放置
    'unknown_obj': _NORMAL[:3] + [(-0.2250, 0.0000, 'red')] + _NORMAL[4:],
    # cell_1 网球 + cell_2 瓶正常摆位, 夹空由 z_grasp:=0.115 注入(见 _MISS 注释)
    #   -> 期望 1 条 grasp_failed + 1 条 placed, 验证异常后任务不中断
    'miss_grasp': [_MISS, _NORMAL[1]],
    # 正常位置 2 物, 配合 z_grasp:=0.005(低于机械臂几何可达下限 z≈0.03)
    #   -> DESCEND_GRID 报 IK 不可达 -> 期望 2 条 unreachable
    'unreachable': _NORMAL[:2],
}
PRESET_HELP = '|'.join(BLOCK_PRESETS)


def _spawn_blocks(context, *args, **kwargs):
    """按 blocks_preset 生成方块。必须在运行时展开: 预设是 launch 参数,
    而 substitution 无法在构造期被 Python 迭代。"""
    preset = context.launch_configurations.get('blocks_preset', 'normal')
    if preset not in BLOCK_PRESETS:
        raise RuntimeError('未知 blocks_preset=%r, 可选: %s' % (preset, PRESET_HELP))
    share = FindPackageShare(PKG).perform(context)
    nodes = []
    for i, (x, y, cls) in enumerate(BLOCK_PRESETS[preset]):
        # 瓶碰撞体是 0.065 见方 box: 对角格(±45°)正对时切向占宽 92mm 超夹爪张开
        # 间隙(85mm), 指尖落在顶面角上把臂卡死。按格方位角旋转使 box 面 ⊥ 接近
        # 方向, 切向占宽回到 65mm(球是 sphere 不受影响, 视觉瓶体是圆柱对称隐形)。
        yaw = math.atan2(y, x) if cls == 'bottle' else 0.0
        nodes.append(Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-file', os.path.join(share, 'urdf', BLOCK_URDF[cls]),
                       '-entity', 'block_%d' % i,
                       '-x', str(x), '-y', str(y), '-z', str(SPAWN_Z[cls]),
                       '-Y', str(yaw),
                       '-timeout', '600'],
            output='screen',
        ))
    print('[classify_sim] blocks_preset=%s -> 生成 %d 个物体 %s'
          % (preset, len(nodes), BLOCK_PRESETS[preset]))
    return nodes



def _gen_vis_urdf(context, *args, **kwargs):
    """生成带 visual 的完整 URDF 供 spawn(GPU 渲染下模型可见; novis 仅 llvmpipe 兜底)。"""
    import subprocess as _sp
    share = FindPackageShare(PKG).perform(context)
    xac = os.path.join(share, 'urdf', 'arm_grasp.urdf.xacro')
    cfg = os.path.join(share, 'config', 'controllers.yaml')
    out = '/tmp/arm_vis.urdf'
    with open(out, 'w') as f:
        r = _sp.run(['xacro', xac, 'config_path:=' + cfg], stdout=f)
    print('[classify_sim] 全视觉 URDF ->', out, 'rc=', r.returncode)
    return []


def _launch_task(context, *args, **kwargs):
    """按 with_task 决定是否启动任务状态机(OpaqueFunction 以便运行时判断)。"""
    if context.launch_configurations.get('with_task', 'true').lower() != 'true':
        print('[classify_sim] with_task=false -> 不启动 classify_task_node(仅感知链路)')
        return []
    return [Node(package=PKG, executable='classify_task_node.py', output='screen')]


def generate_launch_description():
    pkg_share = FindPackageShare(PKG)
    urdf = PathJoinSubstitution([pkg_share, 'urdf', 'arm_grasp.urdf.xacro'])
    config = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])
    world = PathJoinSubstitution([pkg_share, 'worlds', 'table_grid_4c2b.world'])

    # 场景预设: 注入不同异常做验收测试(实验要求四.6)。默认 normal, 正常验收行为不变。
    preset_arg = DeclareLaunchArgument(
        'blocks_preset', default_value='normal', description='场景预设: ' + PRESET_HELP)
    # 抓取高度: 正常 0.0446; 传 0.005 可注入"IK 不可达"异常(低于几何可达下限 z≈0.03)。
    z_grasp_arg = DeclareLaunchArgument(
        'z_grasp', default_value='0.0446', description='抓取 TCP 高度(m), 异常注入用')
    # 是否启动任务状态机: false 时只起感知链路(便于视觉标定/调试, 方块不动)
    with_task_arg = DeclareLaunchArgument(
        'with_task', default_value='true', description='是否启动 classify_task_node')
    # GUI: true 时带 gzclient 窗口启动(配合 WSLg 显示到 Windows 桌面, 供演示/录屏)
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='false', description='是否启动 gzclient GUI')
    paused_arg = DeclareLaunchArgument(
        'paused', default_value='false', description='是否暂停启动(等用户按播放键)')

    # gazebo_grasp_plugin 未生成 GAZEBO_PLUGIN_PATH hook, 必须手动追加,
    # 否则 libgazebo_grasp_fix.so 加载失败 -> 夹不住方块(方块被指尖推走而非夹起)
    env_plugin = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        ['/home/underwater/arm_grasp_ws/install/gazebo_mimic_plugin/lib/'
         'gazebo_mimic_plugin', ':',
         '/home/underwater/arm_grasp_ws/install/gazebo_grasp_plugin/lib/'
         'gazebo_grasp_plugin', ':', os.environ.get('GAZEBO_PLUGIN_PATH', '')])

    # grasp_fix.so 依赖同目录的 libgazebo_grasp_msgs.so, 动态链接器需经 LD_LIBRARY_PATH 才能解析
    # (GAZEBO_PLUGIN_PATH 只告诉 Gazebo 去哪找插件本体, 不解决插件自身的运行时库依赖)
    env_ld = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        ['/home/underwater/arm_grasp_ws/install/gazebo_mimic_plugin/lib/'
         'gazebo_mimic_plugin', ':',
         '/home/underwater/arm_grasp_ws/install/gazebo_grasp_plugin/lib/'
         'gazebo_grasp_plugin', ':', os.environ.get('LD_LIBRARY_PATH', '')])

    # package://robomaster_description 碰撞网格解析: 缺 GAZEBO_MODEL_PATH 时
    # 10 个指节碰撞网格全部加载失败 -> 指节无碰撞体 -> 永远无法接触物体(夹空)
    env_model = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        ['/home/underwater/arm_grasp_ws/src/robomaster_ros', ':',
         '/home/underwater/arm_grasp_ws/install/robomaster_description/share', ':',
         os.environ.get('GAZEBO_MODEL_PATH', '')])

    # WSL/llvmpipe 下机械臂 .dae visual mesh 会杀死 Gazebo 渲染线程,
    # spawn 统一使用去掉 visual 的 URDF(物理/插件/控制器完全保留)
    novis_urdf = '/tmp/arm_novis.urdf'
    gen_novis = ExecuteProcess(
        cmd=['python3',
             '/home/underwater/arm_grasp_ws/src/arm_grasp_sim/scripts/make_novis_urdf.py',
             '-o', novis_urdf],
        output='screen')

    robot_desc = ParameterValue(Command(['xacro ', urdf, ' config_path:=', config]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen',
    )

    from launch.launch_description_sources import PythonLaunchDescriptionSource
    gazebo = __import__('launch').actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments={'world': world, 'verbose': 'false',
                          'gui': LaunchConfiguration('gui'),
                          'paused': LaunchConfiguration('paused')}.items(),
    )

    spawn_arm = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-file', '/tmp/arm_vis.urdf', '-entity', 'arm_grasp',
                   '-timeout', '600'],
        output='screen',
    )

    def ctrl(name, t):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '900'],
                    output='screen')

    classify_server = Node(
        package=PKG, executable='classify_grasp_server.py', output='screen',
        parameters=[{'z_grasp': ParameterValue(LaunchConfiguration('z_grasp'),
                                               value_type=float)}])
    vision = Node(package=PKG, executable='vision_classifier.py', output='screen')
    mapper = Node(package=PKG, executable='grid_mapper.py', output='screen')

    return LaunchDescription([preset_arg, z_grasp_arg, with_task_arg, gui_arg, paused_arg,
                              env_plugin, env_ld,
                              env_model, gen_novis, OpaqueFunction(function=_gen_vis_urdf),

        rsp,
        gazebo,
        TimerAction(period=6.0, actions=[spawn_arm]),
        TimerAction(period=26.0, actions=[OpaqueFunction(function=_spawn_blocks)]),
        TimerAction(period=12.0, actions=[ctrl('arm_vel_controller', 12)]),
        TimerAction(period=13.0, actions=[ctrl('gripper_controller', 13)]),
        TimerAction(period=13.5, actions=[ctrl('bracket_controller', 13.5)]),
        TimerAction(period=14.0, actions=[ctrl('joint_state_broadcaster', 14)]),
        TimerAction(period=16.0, actions=[classify_server]),
        TimerAction(period=18.0, actions=[vision]),
        TimerAction(period=19.0, actions=[mapper]),
        TimerAction(period=30.0, actions=[OpaqueFunction(function=_launch_task)]),
    ])
