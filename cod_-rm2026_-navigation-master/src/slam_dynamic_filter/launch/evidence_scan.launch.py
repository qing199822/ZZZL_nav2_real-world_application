from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="sdf_evidence_pointcloud_to_laserscan",
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
        )
    ])
