#!/usr/bin/env python3
"""诊断启动: 仅生成 机械臂+方块+控制器, 不启动抓取控制器(供 probe_contact.py 独立测量)。"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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
        launch_arguments={'world': world, 'verbose': 'false', 'gui': 'false'}.items(),
    )

    spawn_arm = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'arm_grasp'], output='screen')

    spawn_block = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-file', block_urdf, '-entity', 'block', '-x', '0.20', '-y', '0.0', '-z', '0.045'],
        output='screen')

    spawn_ctrl = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_vel_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '400'], output='screen')

    spawn_grip = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '400'], output='screen')

    spawn_brk = Node(
        package='controller_manager', executable='spawner',
        arguments=['bracket_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '400'], output='screen')

    spawn_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '400'], output='screen')

    return LaunchDescription([
        rsp,
        gazebo,
        TimerAction(period=6.0, actions=[spawn_arm]),
        TimerAction(period=26.0, actions=[spawn_block]),
        TimerAction(period=12.0, actions=[spawn_ctrl]),
        TimerAction(period=13.0, actions=[spawn_grip]),
        TimerAction(period=13.5, actions=[spawn_brk]),
        TimerAction(period=14.0, actions=[spawn_jsb]),
    ])
