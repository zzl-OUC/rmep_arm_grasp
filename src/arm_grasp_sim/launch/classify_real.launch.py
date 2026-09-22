#!/usr/bin/env python3
"""实验三 真机版一键启动（图像 → 识别 → 网格 → 任务 → 真机抓取）。

    ros2 launch arm_grasp_sim classify_real.launch.py conn_type:=ap

包含（满足"一个 launch 启动图像/识别/机械臂/任务控制"）:
  1. robomaster_ros 驱动 ep.launch（with_driver:=true 时；连接方式 conn_type）
  2. vision_classifier（俯视相机 -> Detection2DArray；真机建议 use_yolo:=true）
  3. grid_mapper（检测框中心 -> 网格号）
  4. classify_grasp_server_real（ClassifyGrasp action；底盘对位 + move_arm + gripper）
  5. classify_task_node（状态机：扫描->抓取->分类放置->异常处理，与仿真版共用）

所有上机标定参数在 config/classify_real.yaml（网格/料盒方位、固定 x/z 点位、
相机内外参、YOLO 权重、相机话题）。
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = 'arm_grasp_sim'
    pkg_share = FindPackageShare(pkg)
    cfg = PathJoinSubstitution([pkg_share, 'config', 'classify_real.yaml'])

    with_driver = LaunchConfiguration('with_driver')
    conn_type = LaunchConfiguration('conn_type')

    # 驱动 entry 是 XML (robomaster_ros/launch/main.launch, 声明 model/conn_type)；
    # 用 AnyLaunchDescriptionSource 兼容 XML。若驱动已在别处启动, 设 with_driver:=false。
    driver = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('robomaster_ros'), '/launch', '/main.launch']),
        launch_arguments={'model': 'ep', 'conn_type': conn_type}.items(),
        condition=IfCondition(with_driver),
    )

    vision = Node(package=pkg, executable='vision_classifier.py',
                  parameters=[cfg], output='screen')
    mapper = Node(package=pkg, executable='grid_mapper.py',
                  parameters=[cfg], output='screen')
    server = Node(package=pkg, executable='classify_grasp_server_real.py',
                  parameters=[cfg], output='screen')
    task = Node(package=pkg, executable='classify_task_node.py',
                parameters=[cfg], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('with_driver', default_value='true',
                              description='是否同时启动 robomaster_ros 驱动'),
        DeclareLaunchArgument('conn_type', default_value='ap',
                              description='驱动连接方式: ap / sta / usb'),
        driver,
        TimerAction(period=2.0, actions=[vision]),
        TimerAction(period=3.0, actions=[mapper]),
        # 等驱动/控制话题就绪后再起抓取 server 与任务
        TimerAction(period=8.0, actions=[server]),
        TimerAction(period=10.0, actions=[task]),
    ])
