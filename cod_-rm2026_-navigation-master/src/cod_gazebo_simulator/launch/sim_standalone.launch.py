"""
COD RM2026 Gazebo 独立仿真 (修复版)
==================================
仅启动 Gazebo + 机器人 + 传感器桥接 + RViz。不启动 Nav2。

C1+M6修复: xacro→URDF, robot_description传参
C2修复:   添加 livox_frame 静态TF

使用:
  ros2 launch cod_gazebo_simulator sim_standalone.launch.py
"""
import os
import sys
sys.path.insert(0, os.path.dirname(__file__))
from _sim_common import get_pkg_paths, process_xacro

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim, _ = get_pkg_paths()

    declare_use_rviz = DeclareLaunchArgument("use_rviz", default_value="true")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_desc = process_xacro()

    empty_world = os.path.join(pkg_sim, "resource", "worlds", "empty_world.sdf")
    ign_cmd = os.popen('which ign 2>/dev/null || echo /usr/bin/ign').read().strip()

    gazebo = ExecuteProcess(
        cmd=[ign_cmd, "gazebo", "-r", empty_world, "--force-version", "6"],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    spawn = TimerAction(period=5.0, actions=[
        Node(package="ros_gz_sim", executable="create",
             arguments=["-string", robot_desc, "-name", "cod_robot", "-z", "0.2"]),
    ])

    rsp = TimerAction(period=6.0, actions=[
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"use_sim_time": True, "robot_description": robot_desc}]),
    ])

    livox_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.15",
                   "--frame-id", "base_link", "--child-frame-id", "livox_frame"],
    )

    bridge_cfg = os.path.join(pkg_sim, "config", "gz_bridge.yaml")
    gz_bridge = TimerAction(period=6.0, actions=[
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             parameters=[{"config_file": bridge_cfg}]),
    ])

    pcl2scan = TimerAction(period=7.0, actions=[
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
             remappings=[("cloud_in", "/livox/lidar"), ("scan", "/scan")],
             parameters=[{"target_frame": "base_link", "min_height": -0.3,
                         "max_height": 1.0, "range_min": 0.3, "range_max": 40.0,
                         "use_inf": True, "use_sim_time": True}]),
    ])

    rviz_cfg = os.path.join(pkg_sim, "rviz", "gazebo_nav.rviz")
    rviz = TimerAction(period=10.0, actions=[
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg],
             condition=IfCondition(use_rviz)),
    ])

    ld = LaunchDescription()
    ld.add_action(declare_use_rviz)
    ld.add_action(gazebo)
    ld.add_action(clock_bridge)
    ld.add_action(spawn)
    ld.add_action(rsp)
    ld.add_action(livox_tf)
    ld.add_action(gz_bridge)
    ld.add_action(pcl2scan)
    ld.add_action(rviz)
    return ld
