from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="observe",
            description="observe publishes unchanged delayed scans; enforce applies the mask",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("slam_dynamic_filter"),
                "config",
                "pragmatic_filter.yaml",
            ]),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="sdf_mapping_pointcloud_to_laserscan",
            output="screen",
            remappings=[
                ("cloud_in", "/livox/lidar_filtered"),
                ("scan", "/scan_raw"),
            ],
            parameters=[{
                "target_frame": "base_link",
                "transform_tolerance": 0.01,
                "min_height": 0.1,
                "max_height": 1.0,
                "angle_min": -3.1416,
                "angle_max": 3.1416,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.5,
                "range_max": 20.0,
                "use_inf": True,
            }],
        ),
        Node(
            package="slam_dynamic_filter",
            executable="slam_dynamic_filter_node",
            name="slam_dynamic_filter_node",
            output="screen",
            remappings=[
                ("scan_raw", "/scan_raw"),
                ("odometry", "/Odometry"),
                ("scan_slam_filtered", "/scan_slam_filtered"),
                ("dynamic_mask", "/slam_dynamic_filter/dynamic_mask"),
                ("diagnostics", "/slam_dynamic_filter/diagnostics"),
                ("ready", "/slam_dynamic_filter/ready"),
                ("tracks", "/slam_dynamic_filter/tracks"),
                ("reset_session", "/slam_dynamic_filter/reset_session"),
                ("freeze", "/slam_dynamic_filter/freeze"),
            ],
            parameters=[
                params_file,
                {"mode": mode, "use_sim_time": use_sim_time},
            ],
        ),
    ])
