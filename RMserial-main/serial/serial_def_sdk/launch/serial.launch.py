from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # parma = os.path.join(
    #   get_package_share_directory('test_api'),
    #   'parma',
    #   'real.yaml'
    #   )
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
                #{'robot_name': LaunchConfiguration('robot_name')},
                #parma
                {'serial_port': "/dev/ttyUSB0"}
                #{'serial_port': "/dev/pts/1"}
            ],
            # ======================= 这里是修改的部分 =======================
            # 添加话题重映射
            # 将节点内部订阅的 /hardware/cmd_vel_api 话题
            # 重定向到导航包发布的 /cmd_vel 话题
            remappings=[
                ('/hardware/cmd_vel_api', '/cmd_vel')
            ]
            # ======================= 修改结束 =======================
        ),
    ])
