"""
仿真启动共享函数 — C1+M6修复: xacro处理 + robot_description
"""
import os
from ament_index_python.packages import get_package_share_directory


def get_pkg_paths():
    """返回所有需要的包路径"""
    pkg_sim = get_package_share_directory("cod_gazebo_simulator")
    pkg_bringup = get_package_share_directory("cod_bringup")
    return pkg_sim, pkg_bringup


def process_xacro():
    """C1+M6修复: 将xacro文件处理为URDF字符串, 供robot_state_publisher和ros_gz_sim create使用"""
    pkg_sim, _ = get_pkg_paths()
    xacro_path = os.path.join(pkg_sim, "resource", "cod_robot.urdf.xacro")

    import xacro
    urdf_doc = xacro.process_file(xacro_path)
    return urdf_doc.toprettyxml(indent="  ")


