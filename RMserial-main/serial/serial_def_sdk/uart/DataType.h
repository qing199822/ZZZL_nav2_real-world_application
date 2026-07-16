#ifndef DATA_DEFINE_H__
#define DATA_DEFINE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// 原代码定义为10，我们在重构中会精确计算，这里暂时保留定义
extern const uint8_t extra_len;

// ======================== 消息ID定义 ========================================
enum msg_id {
    heartbeat   = 0x01,
    gimbal= 0x02,
    control     = 0x03,
    additional  = 0x04,
    fix_control = 0x05,
    imu2        = 0x06,
    yaw         = 0x07,
    game_status = 0x08,
    chassis     = 0x10,
    
    // --- 新增：哨兵状态组合包 (电控发给上位机) ---
    sentry_state = 0x20 
};

// ========================== 数据结构体定义 ===================================

// 1. 心跳包 (接收)
typedef struct {
    uint16_t timestamp;
    uint8_t set_launch;
    uint8_t set_arm;
    uint8_t fault_flag;
} HeartBeatReceive;

// 2. 心跳包 (发送)
typedef struct {
    uint16_t timestamp;
    uint8_t battery;
    uint8_t life;
    uint8_t color;
    uint8_t bullet;
    uint8_t fault_flag;
} HeartBeatSend;

// 3. 云台数据 (发送/接收)
typedef struct {
    float yaw;
    float pitch;
    uint8_t fire; 
} Gimbal;

// 4. 控制数据 (视觉->电控)
typedef struct {
    float velocity_top;
    uint8_t shoot;
} ControlData;

// 5. 附加状态 (电控->视觉)
typedef struct {
    uint8_t launch;
    uint8_t arm;
    uint8_t base_hp_our;
    uint8_t base_hp_enemy;
    uint8_t judge_warning_level;
} AdditionalData;

// 6. 底盘速度 (发送/接收)
typedef struct {
    float vx, vy, vz;
} ChassisSpeed;

// 7. IMU数据
typedef struct {
    float vx, vy, vz, ax, ay, az;
} ImuData;

// 8. Yaw轴角度
typedef struct {
    float yaw;
} YawData;

// 9. 比赛状态
typedef struct {
    uint16_t stage_remain_time;
    uint8_t game_progress;
} ext_game_status_t;

// 结构：先云台(yaw, pitch, fire)，后导航(vx, vy, vz)
typedef struct {
    // Gimbal 部分
    float yaw;
    float pitch;
    uint8_t fire;
    
    // Chassis 部分
    float vx;
    float vy;
    float vz;
    float spin;
} FixControlData;

typedef struct {
    // // 1. 云台数据 (9 bytes: 4+4+1)
    // float yaw;
    // float pitch;
    // uint8_t fire; 
    
    // 2. 心跳数据 (7 bytes: 2+1+1+1+1+1)
    // uint16_t timestamp;
    float battery;
    float life;
    float color;
    float bullet;
    float fault_flag;
} SentryState;


// ======================== 全局变量声明 (保持兼容性) ========================
extern HeartBeatReceive heartbeat_send; // 注意：命名虽然叫send，但原代码逻辑似乎是用作接收的buffer，暂时保留原样
extern HeartBeatSend heartbeat_receive;
extern Gimbal gimbal_send;
extern Gimbal gimbal_receive;
extern ControlData control_data;
extern AdditionalData additional_data;
extern ChassisSpeed chassis_send;
extern ChassisSpeed chassis_receive;
extern YawData yaw_data;
extern ImuData imu2_data;
extern ext_game_status_t game_status_data;

// 新增全局变量
extern FixControlData fix_control_send;
extern SentryState sentry_state_receive;
#ifdef __cplusplus
}
#endif

#endif // DATA_DEFINE_H__
