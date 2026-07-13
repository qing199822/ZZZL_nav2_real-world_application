from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
          'robot_name',
          default_value='auto_sentry',
          description='node namespace of robot'),

        Node(
            package='serial_def_sdk',
            executable='uart',
            name='hardware_serial',
            parameters=[
                {'serial_port': "/dev/cod_mcu"}
            ],
            # COD导航适配: 将 /hardware/cmd_vel_api 重映射到 fake_vel_transform 输出的 aft_cmd_vel
            remappings=[
                ('/hardware/cmd_vel_api', '/aft_cmd_vel')
            ]
        ),
    ])
