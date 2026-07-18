#!/bin/bash
# COD 导航系统一键启动脚本
# 用法: bash start_nav.sh

set -e

WORKSPACE="$HOME/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master"

echo "========================================="
echo "  COD 导航系统启动"
echo "========================================="

# 1. 检查雷达网络
echo "[1/4] 检查雷达网络..."
if ping -c 1 -W 2 192.168.1.181 &>/dev/null; then
    echo "  雷达网络连通 ✅"
else
    echo "  尝试连接雷达网络..."
    nmcli connection up Livox-MID360 2>/dev/null || true
    sleep 2
    if ping -c 1 -W 2 192.168.1.181 &>/dev/null; then
        echo "  雷达网络连通 ✅"
    else
        echo "  ⚠ 雷达网络不通，请检查网线"
    fi
fi

# 2. 检查串口
echo "[2/4] 检查 MCU 串口..."
if [ -e /dev/cod_mcu ]; then
    echo "  /dev/cod_mcu 已就绪 ✅"
elif [ -e /dev/ttyACM0 ]; then
    sudo ln -sf /dev/ttyACM0 /dev/cod_mcu
    echo "  已创建 /dev/cod_mcu ✅"
else
    echo "  ⚠ MCU 未连接"
fi

# 3. 加载环境
echo "[3/4] 加载 ROS2 环境..."
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

# 4. 启动雷达驱动(后台)
echo "[4/4] 启动雷达驱动 + 导航..."
echo "  启动雷达驱动..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
LIVOX_PID=$!
sleep 5

# 检查驱动是否成功启动
if ! kill -0 $LIVOX_PID 2>/dev/null; then
    echo "  ❌ 雷达驱动启动失败"
    exit 1
fi
echo "  雷达驱动已启动 (PID: $LIVOX_PID)"

# 5. 启动导航(前台)
echo "  启动导航栈..."
ros2 launch cod_bringup singlenav_launch.py

# 导航退出后，清理雷达驱动
echo "导航已退出，清理..."
kill $LIVOX_PID 2>/dev/null || true
wait $LIVOX_PID 2>/dev/null || true
echo "完成。"
