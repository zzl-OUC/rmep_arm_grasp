#!/usr/bin/env python3
"""标定专用: 仅起 world(含俯视相机) + 6 个彩色方块, 无机械臂/任务/抓取。
用于静止场景下校准 4 色 HSV 阈值。
"""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

BLOCKS = [
    (0.1785, 0.0650, "green"),     (0.1271, 0.1412, "yellow"),
    (0.0460, 0.1844, "red"),     (-0.0460, 0.1844, "blue"),
    (-0.1271, 0.1412, "yellow"),     (-0.1785, 0.0650, "green"),
]


def generate_launch_description():
    pkg = "arm_grasp_sim"
    pkg_share = FindPackageShare(pkg)
    world = PathJoinSubstitution([pkg_share, "worlds", "table_grid_4c2b.world"])
    BLOCK_URDF = {"green": "block.urdf", "yellow": "block_yellow.urdf",
                  "red": "block_red.urdf", "blue": "block_blue.urdf"}
    gazebo = __import__("launch").actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]),
        launch_arguments={"world": world, "verbose": "false",
                          "gui": "false", "paused": "false"}.items())
    spawn_blocks = []
    for i, (x, y, cls) in enumerate(BLOCKS):
        urdf = PathJoinSubstitution([pkg_share, "urdf", BLOCK_URDF[cls]])
        spawn_blocks.append(Node(
            package="gazebo_ros", executable="spawn_entity.py",
            arguments=["-file", urdf, "-entity", "block_%d" % i,
                       "-x", str(x), "-y", str(y), "-z", "0.045",
                       "-timeout", "600"], output="screen"))
    acts = [gazebo] + [TimerAction(period=8.0, actions=[b]) for b in spawn_blocks]
    return LaunchDescription(acts)
