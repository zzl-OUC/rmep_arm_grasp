#!/usr/bin/env python3
"""最小化 launch：仅 robot_state_publisher + FK 探针，用于标定 waypoint（无 Gazebo）。"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf = os.path.join(pkg_dir, 'urdf', 'arm_grasp.urdf.xacro')
    config = os.path.join(pkg_dir, 'config', 'controllers.yaml')
    robot_desc = Command(['xacro ', urdf, ' config_path:=', config])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen',
    )
    fk = Node(
        executable=os.path.join(pkg_dir, 'scripts', 'fk_probe.py'),
        output='screen',
    )
    return LaunchDescription([rsp, fk])
