# 郑州大学中州烛龙战队2026赛季串口包
*修改自25赛季serial_new*
## 使用
在工作空间下进入/serial文件夹
```bash
source install/setup.bash
ros2 launch serial_def_sdk serial.launch.py
```
**对于新的ubuntu22.04系统**
先卸载冲突的两个模块
```bash
sudo apt-get purge modemmanager brltty
```
然后加入用户组
```bash
sudo usermod -a -G dialout $USER
```
重启
```bash
sudo reboot
```


看到“节点已启动”就算启用成功

## 结构

整个包的框架很简单：
1.从ROS接收导航和自瞄数据的base_controller.cpp/hpp
	同时将下位机数据（rpy）发布到ros系统
2.将数据转化翻译给下位机的uart包
3.帧头是0xA5,波特率115200，使用seasky协议中的crc校验方式uart/crc8/16.c/h（但是没有用crc校验，因为可能是跟电控的校验没对上需要后续修改）
4.实现了自瞄和导航消息的整合发送（fix_control）

## pub/sub话题
pub
/hardware/gimble_current_rpy
(从下位机接收云台实时rpy)
sub
/cmd_spin(小陀螺）
/cmd_vel（导航消息）
/vision/gimble_control（自瞄消息,包含开火指令）

## 其他文件暂时没用
test_pub.py是模拟三个话题发布消息，供电控视觉联调，测试串口包是否正常，硬件是否接收到，
如果接收不到，大概率是crc的问题
