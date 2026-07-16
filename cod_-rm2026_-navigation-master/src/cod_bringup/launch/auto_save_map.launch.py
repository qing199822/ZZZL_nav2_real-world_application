# auto_save_map.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()

    # H3修复: 使用包相对路径，不再硬编码 /home/cod-sentry
    bringup_dir = get_package_share_directory('cod_bringup')
    save_dir = os.path.join(bringup_dir, 'maps', 'auto_save')

    def create_save_command(suffix: str) -> list:
        return [
            'bash', '-c',
            f'mkdir -p {save_dir} && '
            f'ros2 run nav2_map_server map_saver_cli -f {save_dir}/auto_map_{suffix}'
        ]

    intervals = [ 30, 60, 90, 120, 150, 180, 210, 240, 270, 300]    #保存时间间隔
    for t in intervals:
        action = TimerAction(
            period=float(t),
            actions=[ExecuteProcess(cmd=create_save_command("$(date +%H%M%S)"), output='screen')]
        )
        ld.add_action(action)

    return ld
