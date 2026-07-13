"""
COD RM2026 Gazebo 仿真 + SLAM 建图 (修复版)
==========================================
启动: Gazebo → 机器人 → slam_toolbox → Nav2 → RViz

C1+M6修复: xacro→URDF处理, robot_description传参
C2修复:   添加 base_link→livox_frame 静态TF
H2修复:   时钟桥接仅内联

使用:
  ros2 launch cod_gazebo_simulator gazebo_slam.launch.py world:=rmul_2025
"""
import os
import sys
sys.path.insert(0, os.path.dirname(__file__))
from _sim_common import get_pkg_paths, process_xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup_launch(context, *args, **kwargs):
    pkg_sim, pkg_bringup = get_pkg_paths()
    world = LaunchConfiguration("world").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    robot_x = LaunchConfiguration("robot_x").perform(context)
    robot_y = LaunchConfiguration("robot_y").perform(context)
    robot_z = LaunchConfiguration("robot_z").perform(context)
    robot_yaw = LaunchConfiguration("robot_yaw").perform(context)

    # C1+M6: 处理xacro为URDF
    robot_desc = process_xacro()
    print(f"[INFO] Xacro processed, URDF size: {len(robot_desc)} chars")

    # 世界路径
    world_dir = os.path.join(pkg_sim, "resource", "worlds")
    world_sdf = os.path.join(world_dir,
        "empty_world.sdf" if world == "empty" else f"{world}_world.sdf")

    # 1. Gazebo
    ign_cmd = os.popen('which ign 2>/dev/null || echo /usr/bin/ign').read().strip()
    gazebo = ExecuteProcess(
        cmd=[ign_cmd, "gazebo", "-r", world_sdf, "--force-version", "6"],
        output="screen", shell=False,
    )

    # 2. 时钟桥接
    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # 3. 生成机器人 (使用 -param 传URDF string, 而非原始xacro文件)
    spawn = TimerAction(period=5.0, actions=[
        Node(package="ros_gz_sim", executable="create",
             arguments=["-string", robot_desc, "-name", "cod_robot",
                       "-x", robot_x, "-y", robot_y,
                       "-z", robot_z, "-Y", robot_yaw]),
    ])

    # 4. Robot state publisher (含robot_description)
    rsp = TimerAction(period=6.0, actions=[
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"use_sim_time": True, "robot_description": robot_desc}]),
    ])

    # C2: base_link → livox_frame 静态TF
    livox_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="livox_to_base_link",
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.15",
                   "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                   "--frame-id", "base_link", "--child-frame-id", "livox_frame"],
    )

    # 5. 传感器桥接
    bridge_cfg = os.path.join(pkg_sim, "config", "gz_bridge.yaml")
    gz_bridge = TimerAction(period=6.0, actions=[
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             parameters=[{"config_file": bridge_cfg}]),
    ])

    # 6. pointcloud→laserscan
    pcl2scan = TimerAction(period=7.0, actions=[
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
             remappings=[("cloud_in", "/livox/lidar"), ("scan", "/scan")],
             parameters=[{"target_frame": "base_link", "min_height": -0.3,
                         "max_height": 1.0, "range_min": 0.3, "range_max": 40.0,
                         "use_inf": True, "angle_increment": 0.0087,
                         "use_sim_time": True}]),
    ])

    # 7. slam_toolbox
    slam_params = os.path.join(pkg_bringup, "params", "mapper_params_online_async.yaml")
    slam_toolbox = TimerAction(period=8.0, actions=[
        Node(package="slam_toolbox", executable="async_slam_toolbox_node",
             name="slam_toolbox",
             parameters=[slam_params, {"use_sim_time": True}]),
    ])

    # 8. Nav2
    nav2_params = os.path.join(pkg_sim, "config", "sim_nav2_params.yaml")
    nav2 = TimerAction(period=10.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, "launch", "navigation_launch.py")),
            launch_arguments={"use_sim_time": "True", "autostart": "True",
                             "params_file": nav2_params, "use_composition": "False",
                             "use_respawn": "False"}.items()),
    ])

    # 9. 自动保存地图
    auto_save = TimerAction(period=12.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, "launch", "auto_save_map.launch.py")),
            condition=IfCondition(LaunchConfiguration("auto_save"))),
    ])

    # 10. RViz
    rviz_cfg = os.path.join(pkg_sim, "rviz", "gazebo_nav.rviz")
    rviz = TimerAction(period=10.0, actions=[
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg],
             condition=IfCondition(LaunchConfiguration("use_rviz"))),
    ])

    return [gazebo, clock_bridge, spawn, rsp, livox_tf, gz_bridge,
            pcl2scan, slam_toolbox, nav2, auto_save, rviz]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("world", default_value="rmul_2025",
        choices=["rmul_2024", "rmul_2025", "rmuc_2024", "rmuc_2025", "empty"]))
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))
    ld.add_action(DeclareLaunchArgument("auto_save", default_value="false"))
    ld.add_action(DeclareLaunchArgument("robot_x", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("robot_y", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("robot_z", default_value="0.2"))
    ld.add_action(DeclareLaunchArgument("robot_yaw", default_value="0.0"))
    ld.add_action(OpaqueFunction(function=setup_launch))
    return ld
