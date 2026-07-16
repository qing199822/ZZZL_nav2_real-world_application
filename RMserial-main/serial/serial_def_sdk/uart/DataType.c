#include "DataType.h"

// 实例化全局变量
const uint8_t extra_len = 10; // 保持原有的常量定义

HeartBeatReceive heartbeat_send = {0};
HeartBeatSend heartbeat_receive = {0};

Gimbal gimbal_send = {0};
Gimbal gimbal_receive = {0};

ControlData control_data = {0};
AdditionalData additional_data = {0};

ChassisSpeed chassis_send = {0};
ChassisSpeed chassis_receive = {0};

YawData yaw_data = {0};
ImuData imu2_data = {0};
ext_game_status_t game_status_data = {0};

// 新增实例化
FixControlData fix_control_send = {0};
SentryState sentry_state_receive = {0}; 