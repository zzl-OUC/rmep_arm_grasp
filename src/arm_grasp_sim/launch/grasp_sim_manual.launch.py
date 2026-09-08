#!/usr/bin/env python3
"""Client-only launch: assumes gzserver already running (started under gdb)."""
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = "arm_grasp_sim"
    pkg_share = FindPackageShare(pkg)
    urdf = PathJoinSubstitution([pkg_share, "urdf", "arm_grasp.urdf.xacro"])
    config = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])
    block_urdf = PathJoinSubstitution([pkg_share, "urdf", "block.urdf"])
    robot_desc = Command(["xacro ", urdf, " config_path:=", config])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )
    spawn_arm = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "arm_grasp"],
        output="screen",
    )
    spawn_block = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-file", block_urdf, "-entity", "block", "-x", "0.15", "-y", "0.0", "-z", "0.045"],
        output="screen",
    )
    spawn_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"],
        output="screen",
    )
    grasp_ctrl = Node(
        package=pkg,
        executable="grasp_controller.py",
        output="screen",
    )
    return LaunchDescription([
        rsp,
        TimerAction(period=4.0, actions=[spawn_arm]),
        TimerAction(period=5.0, actions=[spawn_block]),
        TimerAction(period=10.0, actions=[spawn_ctrl]),
        TimerAction(period=14.0, actions=[grasp_ctrl]),
    ])
