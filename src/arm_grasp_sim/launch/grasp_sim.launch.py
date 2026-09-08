#!/usr/bin/env python3
"""定点抓取仿真统一启动文件（实验要求：一个 Launch 启动全部）。

启动：
  1. robot_state_publisher（发布 TF）
  2. gazebo (gzserver, headless) + 世界
  3. 生成机械臂 URDF 到 Gazebo（含 gazebo_ros2_control 插件）
  4. 生成方块 block 到 A 点（待抓取物）
  5. 启动 arm_vel_controller（JointTrajectoryController）
  6. grasp_controller（Action 服务端 + 状态机）

注意：方块初始放在 A=(0.15,0)，每次循环由 grasp_controller 通过
gazebo set_entity_state 复位，模拟人工重新摆放。
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = 'arm_grasp_sim'
    pkg_share = FindPackageShare(pkg)
    urdf_file = LaunchConfiguration('urdf_file', default='arm_grasp.urdf.xacro')
    urdf = PathJoinSubstitution([pkg_share, 'urdf', urdf_file])
    config = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])
    world = PathJoinSubstitution([pkg_share, 'worlds', 'empty_grasp.world'])
    block_urdf = PathJoinSubstitution([pkg_share, 'urdf', 'block.urdf'])

    robot_desc = ParameterValue(Command(['xacro ', urdf, ' config_path:=', config]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments={
            'world': world,
            'verbose': 'false',
            'gui': 'false',
            'paused': 'true',
        }.items(),
    )

    # 生成机械臂
    spawn_arm = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'arm_grasp'],
        output='screen',
    )

    # 生成方块（待抓取物），放在 A 点，桌面高度
    spawn_block = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', block_urdf, '-entity', 'block',
                   '-x', '0.20', '-y', '0.0', '-z', '0.045'],
        output='screen',
    )

    # 启动关节轨迹控制器
    spawn_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_vel_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '400'],
        output='screen',
    )

    # 力控夹爪控制器(effort 接口): 真夹持力, 而非运动学瞬移
    spawn_grip = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '400'],
        output='screen',
    )

    # 关节状态广播器(替代 gazebo_ros_joint_state_publisher)
    spawn_brk = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['bracket_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '400'],
        output='screen',
    )

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '400'],
        output='screen',
    )

    grasp_ctrl = Node(
        package=pkg,
        executable='grasp_controller.py',
        output='screen',
    )

    # 时序：gazebo -> 等几秒生成模型 -> 生成臂与方块 -> 等控制器可用 -> 控制器 -> 抓取节点
    # 抓取控制器（Action 服务端 + 状态机）：等控制器就绪后再启动
    grasp_ctrl_delayed = TimerAction(period=15.0, actions=[grasp_ctrl])

    return LaunchDescription([
        rsp,
        gazebo,
        TimerAction(period=6.0, actions=[spawn_arm]),
        TimerAction(period=26.0, actions=[spawn_block]),
        TimerAction(period=12.0, actions=[spawn_ctrl]),
        TimerAction(period=13.0, actions=[spawn_grip]),
        TimerAction(period=13.5, actions=[spawn_brk]),
        TimerAction(period=14.0, actions=[spawn_jsb]),
        grasp_ctrl_delayed,
    ])
