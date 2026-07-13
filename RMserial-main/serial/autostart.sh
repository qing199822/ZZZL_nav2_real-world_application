#!/bin/bash

sleep 2
cd /home/rm/serial_new/serial/

<<INFO
This script is written to launch the 
robomaster application when the computer 
poweron.

gnome-terminal [--geometry=/home/rm/serial_new/serial/] -- bash -c "/home/rm/serial_new/serial/autostart.sh"

INFO

echo "------------------------------ROS"
source /opt/ros/foxy/setup.bash

echo "--------------------------PYTHON"
source /home/rm/serial_new/serial/install/setup.bash
ros2 launch serial_def_sdk serial.launch.py  &
ls

wait
exit 0
