#!/usr/bin/env python3
"""实验三 一键启动: 桌面分类整理仿真（实验要求: 一个 Launch 启动相机/识别/机械臂/任务控制）。

基于已验收的 grasp_sim.launch.py 扩展:
  1. robot_state_publisher
  2. gazebo + table_grid_4c2b.world (4 取物网格 + 2 料盒 + 俯视相机)
  3. 生成机械臂 + 生成 6 个方块(绿4/黄2, 分散在不同网格)
  4. ros2_control 控制器组
  5. classify_grasp_server (扩展抓取 action server)
  6. vision_classifier (俯视相机 -> Detection2DArray)
  7. grid_mapper (检测框中心 -> 网格号)
  8. classify_task_node (状态机: 扫描->抓取->分类放置->异常处理->完成)

验收对照(实验要求四):
  - 自动识别并分类 >=2 类物体: green_block / yellow_block -> bin_0 / bin_1
  - >=6 物体, >=5 正确: 启动即生成 6 块, 状态机循环至全部处理
  - 无人工干预: 全链路自动
  - 异常: 空网格/未识别/抓取失败(重试1次)均记录日志
  - 日志: ~/classify_logs/task_log.json + /grasp_state + /task_state 话题
"""
import json
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


# 6 个方块初始位姿(网格中心, 桌面高 0.025+块半高 0.02 = 0.045)
# cell_1 绿, cell_2 黄, cell_3 绿, cell_4 黄, bin 留空; 覆盖 4 网格中 4 块 + 2 块放 cell_1/2 叠放边
BLOCKS = [
    # (x, y, class)  —— 与 GRIDS/CELLS 的 6 个网格中心一一对应(绿 3 / 黄 3)
    (0.190, 0.000, 'green'),     # cell_1
    (0.095, 0.1645, 'yellow'),   # cell_2
    (-0.095, 0.1645, 'green'),   # cell_3
    (-0.190, 0.000, 'yellow'),   # cell_4
    (0.1645, 0.0950, 'green'),   # cell_5
    (-0.1645, 0.0950, 'yellow'), # cell_6
]


def generate_launch_description():
    pkg = 'arm_grasp_sim'
    pkg_share = FindPackageShare(pkg)
    urdf = PathJoinSubstitution([pkg_share, 'urdf', 'arm_grasp.urdf.xacro'])
    config = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])
    world = PathJoinSubstitution([pkg_share, 'worlds', 'table_grid_4c2b.world'])
    block_urdf_g = PathJoinSubstitution([pkg_share, 'urdf', 'block.urdf'])
    block_urdf_y = PathJoinSubstitution([pkg_share, 'urdf', 'block_yellow.urdf'])

    # gazebo_grasp_plugin 未生成 GAZEBO_PLUGIN_PATH hook, 必须手动追加,
    # 否则 libgazebo_grasp_fix.so 加载失败 -> 夹不住方块(方块被指尖推走而非夹起)
    env_plugin = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        ['/home/underwater/arm_grasp_ws/install/gazebo_grasp_plugin/lib/'
         'gazebo_grasp_plugin', ':', os.environ.get('GAZEBO_PLUGIN_PATH', '')])

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
                          'gui': 'false', 'paused': 'false'}.items(),
    )

    spawn_arm = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-file', novis_urdf, '-entity', 'arm_grasp',
                   '-timeout', '600'],
        output='screen',
    )

    # 生成 6 个方块: 同一 urdf 多实体, 名字 block_0..block_5
    # 注: classify_grasp_server 的 block_name 参数对应第一个块; 其余块仅作视觉目标
    spawn_blocks = []
    for i, (x, y, cls) in enumerate(BLOCKS):
        urdf = block_urdf_y if cls == 'yellow' else block_urdf_g
        spawn_blocks.append(Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-file', urdf, '-entity', 'block_%d' % i,
                       '-x', str(x), '-y', str(y), '-z', '0.045', '-timeout', '600'],
            output='screen',
        ))

    def ctrl(name, t):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '900'],
                    output='screen')

    classify_server = Node(package=pkg, executable='classify_grasp_server.py',
                           output='screen')
    vision = Node(package=pkg, executable='vision_classifier.py', output='screen')
    mapper = Node(package=pkg, executable='grid_mapper.py', output='screen')
    task = Node(package=pkg, executable='classify_task_node.py', output='screen')

    return LaunchDescription([env_plugin, gen_novis,

        rsp,
        gazebo,
        TimerAction(period=6.0, actions=[spawn_arm]),
        TimerAction(period=26.0, actions=spawn_blocks),
        TimerAction(period=12.0, actions=[ctrl('arm_vel_controller', 12)]),
        TimerAction(period=13.0, actions=[ctrl('gripper_controller', 13)]),
        TimerAction(period=13.5, actions=[ctrl('bracket_controller', 13.5)]),
        TimerAction(period=14.0, actions=[ctrl('joint_state_broadcaster', 14)]),
        TimerAction(period=16.0, actions=[classify_server]),
        TimerAction(period=18.0, actions=[vision]),
        TimerAction(period=19.0, actions=[mapper]),
        TimerAction(period=30.0, actions=[task]),
    ])
