nmcli connection add 创建的连接是持久化的，存储在 /etc/NetworkManager/system-connections/Livox-MID360.nmconnection，拔插或重启后 NetworkManager 会自动激活
已确认，配置完全持久化：
  - autoconnect: 是
  - IP 192.168.1.50/24
  - never-default: 是（不影响外网）
  每次重启后依次执行：

  1. 连接雷达网络（USB 网卡需已插入）
  ping -c 1 192.168.1.181 || sudo nmcli connection up Livox-MID360

  2. 确认串口
  ls /dev/cod_mcu || sudo ln -sf /dev/ttyACM0 /dev/cod_mcu

  3. 终端 1 — 启动雷达驱动
  source /opt/ros/humble/setup.bash
  source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
  ros2 launch livox_ros_driver2 msg_MID360_launch.py
  # 确认看到: "livox/lidar publish use PointCloud2 format"

  4. 终端 2 — 启动导航
  source /opt/ros/humble/setup.bash
  source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
  ros2 launch cod_bringup singlenav_launch.py

  5. RViz 操作
  - 点击 2D Pose Estimate 设定初始位姿
  - 点击 Nav2 Goal 发送目标点，开始导航建图

  就这些。雷达和 MCU 保持连接，网络和串口一步确认即可。
