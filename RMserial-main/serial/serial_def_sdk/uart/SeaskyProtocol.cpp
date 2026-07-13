#include "SeaskyProtocol.hpp"
#include <iostream>
#include <cstring>

// 引入 CRC 算法
extern "C" {
#include "crc8.h"
#include "crc16.h"
}

SeaskyProtocol::SeaskyProtocol() {
    // 预分配缓冲区，避免频繁内存分配
    rx_buffer_.reserve(1024);
    tx_buffer_.resize(MAX_PACKET_SIZE);
}

SeaskyProtocol::~SeaskyProtocol() {
    driver_.close();
}

bool SeaskyProtocol::init(const std::string& device_name) {
    // 设置接收回调
    driver_.setReceiverCallback(std::bind(&SeaskyProtocol::onDataReceived, this, std::placeholders::_1, std::placeholders::_2));
    
    // 打开串口 (阻塞直到成功，模拟原代码逻辑，或者你可以选择立即返回)
    // 为了兼容原代码的"wait_uart"逻辑，这里我们做一个简单的重试循环
    int retries = 0;
    while (!driver_.open(device_name, 115200)) {
        if (++retries > 5) {
            std::cerr << "[Protocol] Failed to open UART after retries." << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return true;
}

// ======================= 发送逻辑 (Serialize) =======================

bool SeaskyProtocol::send(int msg_id, void* data) {
    std::lock_guard<std::mutex> lock(send_mutex_); // <--- 关键！整个打包发送过程原子化

    // 临时缓冲区用于存放序列化后的 payload (不含帧头)
    uint8_t payload[128]; 
    uint16_t len = 0;

    switch (msg_id) {
        case heartbeat: {
            HeartBeatSend* d = (HeartBeatSend*)data;
            // 序列化: timestamp(2) + battery(1) + life(1) + color(1) + bullet(1) + fault(1) = 7 bytes
            // 这里为了简单，假设是小端序(Little Endian)系统，直接拷贝。
            // 如果需要跨平台兼容，建议逐字节赋值。这里沿用原代码的直接 memcpy 风格。
            // 但原代码的 DataType.c 用了很奇怪的 union 转换，这里我们用更清晰的 memcpy。
            
            // 注意：DataType.h 定义的 HeartBeatSend 结构体有内存对齐(padding)风险
            // 最好挨个拷贝成员
            memcpy(payload + len, &d->timestamp, 2); len += 2;
            payload[len++] = d->battery;
            payload[len++] = d->life;
            payload[len++] = d->color;
            payload[len++] = d->bullet;
            payload[len++] = d->fault_flag;
            break;
        }
        case gimbal: {
            Gimbal* d = (Gimbal*)data;
            // float(4) + float(4) + u8(1) = 9 bytes
            memcpy(payload + len, &d->yaw, 4); len += 4;
            memcpy(payload + len, &d->pitch, 4); len += 4;
            payload[len++] = d->fire;
            break;
        }
        case chassis: {
            ChassisSpeed* d = (ChassisSpeed*)data;
            // 3 * float = 12 bytes
            memcpy(payload + len, &d->vx, 4); len += 4;
            memcpy(payload + len, &d->vy, 4); len += 4;
            memcpy(payload + len, &d->vz, 4); len += 4;
            break;
        }
        case control: {
            ControlData* d = (ControlData*)data;
            memcpy(payload + len, &d->velocity_top, 4); len += 4;
            payload[len++] = d->shoot;
            break;
        }
        
        // --- 新增：FixControl 组合包 ---
        case fix_control: {
            FixControlData* d = (FixControlData*)data;
            // 1. 云台部分
            memcpy(payload + len, &d->yaw, 4); len += 4;
            memcpy(payload + len, &d->pitch, 4); len += 4;
            payload[len++] = d->fire;
            
            // 2. 导航部分
            memcpy(payload + len, &d->vx, 4); len += 4;
            memcpy(payload + len, &d->vy, 4); len += 4;
            memcpy(payload + len, &d->vz, 4); len += 4;
            memcpy(payload + len, &d->spin, 4); len += 4;
            
            // 总长 21 字节
            break;
        }

        default:
            // 未知的 ID，不发送
            return false;
    }

    sendPacket((uint16_t)msg_id, payload, len);
    return true;
}

void SeaskyProtocol::sendPacket(uint16_t msg_id, const uint8_t* payload, uint16_t payload_len) {
    // 帧结构：
    // Header(1) + Len(2) + CRC8(1) + ID(2) + Flags(2) + Data(N) + CRC16(2)
    // Total = 10 + N

    uint8_t frame[256];
    int pos = 0;

    frame[pos++] = PROTOCOL_HEADER_ID; // 0xA5
    
    // 数据长度 (低字节在前)
    frame[pos++] = payload_len & 0xFF;
    frame[pos++] = (payload_len >> 8) & 0xFF;

    // CRC8 (校验前3个字节: Header + Len)
    frame[pos++] = crc_8(frame, 3);

    // ID (2 bytes)
    frame[pos++] = msg_id & 0xFF;
    frame[pos++] = (msg_id >> 8) & 0xFF;

    // Flags (2 bytes, 暂时写死，沿用原代码)
    uint16_t flags = 0; // 或者原代码的 30<<8 | 1
    frame[pos++] = flags & 0xFF;
    frame[pos++] = (flags >> 8) & 0xFF;

    // Data
    if (payload_len > 0) {
        memcpy(frame + pos, payload, payload_len);
        pos += payload_len;
    }

    // CRC16 (校验整个帧，除了最后的 CRC16 字段)
    uint16_t crc = crc_16(frame, pos);
    frame[pos++] = crc & 0xFF;
    frame[pos++] = (crc >> 8) & 0xFF;
    driver_.write(frame, pos);
}

// ======================= 接收逻辑 (Deserialize) =======================

void SeaskyProtocol::onDataReceived(const uint8_t* data, size_t len) {
    // printf("[RAW DUMP] ==> ");
    // for(size_t i = 0; i < len; ++i) {
    //     printf("%02X ", data[i]);
    // }
    // printf("\n");
    std::lock_guard<std::mutex> lock(rx_mutex_);
    
    // 将新数据追加到 buffer
    rx_buffer_.insert(rx_buffer_.end(), data, data + len);
    
    // 处理 buffer
    processBuffer();
}

// 修改后的 processBuffer：支持宽松模式 (兼容旧代码的调试逻辑)
void SeaskyProtocol::processBuffer() {
    while (rx_buffer_.size() >= 8) { 
        
        // 1. 检查帧头 0xA5
        if (rx_buffer_[0] != PROTOCOL_HEADER_ID) {
            rx_buffer_.erase(rx_buffer_.begin());
            continue;
        }

        // 2. 解析长度
        uint16_t payload_len = (uint16_t)rx_buffer_[1] | ((uint16_t)rx_buffer_[2] << 8);
        if (rx_buffer_[4] == 0x20 && rx_buffer_[5] == 0x00 && payload_len == 22) {
            payload_len = 20; 
        }

        // 安全检查：如果解析出的长度大得离谱，肯定是错的
        if (payload_len > 200) { 
            rx_buffer_.erase(rx_buffer_.begin());
            continue;
        }

        // 3. 计算全帧长度
        size_t total_frame_len = payload_len + 10;
        if (rx_buffer_.size() < total_frame_len) {
            return; // 数据不够，等下次
        }

        // ============================================================
        // 关键点：校验策略
        // ============================================================
        
        bool crc_ok = false;
        
        // 策略 1: 尝试标准 CRC16 校验
        uint16_t calculated = crc_16(rx_buffer_.data(), total_frame_len - 2);
        uint16_t received = (uint16_t)rx_buffer_[total_frame_len - 2] | 
                            ((uint16_t)rx_buffer_[total_frame_len - 1] << 8);
        
        if (calculated == received) {
            crc_ok = true;
        } 
        else {

            // 可以在这里打印一个警告
            // std::cout << "[Protocol] CRC Failed but force decoding..." << std::endl;
            crc_ok = true; // <--- 强制认为 OK，还原 NO-CRC MODE
        }

        if (crc_ok) {
            // 提取 ID 和 数据
            uint16_t msg_id = (uint16_t)rx_buffer_[4] | ((uint16_t)rx_buffer_[5] << 8);
            const uint8_t* payload_ptr = rx_buffer_.data() + 8;
            
            dispatchMessage(msg_id, payload_ptr, payload_len);

            // 移除处理完的帧
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total_frame_len);
        } else {
            // 如果你决定在未来启用严格校验，这里应该 erase(begin)
            rx_buffer_.erase(rx_buffer_.begin());
        }
    }
}

void SeaskyProtocol::dispatchMessage(uint16_t msg_id, const uint8_t* payload, uint16_t len) {
    // 这里我们将解析出的数据直接写回 DataType.c 定义的全局变量中
    // 这样上层的 main.cpp 或其他代码直接读取全局变量即可，保持了 API 兼容性
    
    // 注意：这里的全局变量写操作理论上应该加锁，但为了完全兼容旧的 C 接口（它们直接读写全局变量），
    // 我们暂时不加锁，或者假设上层读取频率较低。
    // 在更严格的设计中，应该提供 Get 函数并返回副本。
    // printf("[ID Sniffer] 👃 Received a valid packet with ID: 0x%04X, Payload Length: %d\n", msg_id, len);
    int pos = 0;

    switch (msg_id) {
        case heartbeat:
            // 接收是 HeartBeatReceive 结构
            // 假设 payload 结构: timestamp(2) + battery(1) ...
            // 注意：原代码的 receive 结构和 send 结构定义有点混淆，这里按 DataType.h 的 HeartBeatReceive 解析
            // 实际上原协议中心跳包接收的是: time(2) + set_launch(1) + set_arm(1) + fault(1)
            if (len >= 5) {
                memcpy(&heartbeat_receive.timestamp, payload + pos, 2); pos += 2;
                heartbeat_receive.battery = payload[pos++];
                heartbeat_receive.life = payload[pos++];
                heartbeat_receive.color = payload[pos++];
                heartbeat_receive.bullet = payload[pos++];
                // heartbeat_receive.fault_flag = payload[pos++]; // 视具体长度而定
            }
            break;

        case gimbal:
            if (len >= 8) {
                memcpy(&gimbal_receive.yaw, payload + pos, 4); pos += 4;
                memcpy(&gimbal_receive.pitch, payload + pos, 4); pos += 4;
                // gimbal_receive.fire = payload[pos]; // 假设有
            }
            break;

        case chassis:
             if (len >= 12) {
                memcpy(&chassis_receive.vx, payload + pos, 4); pos += 4;
                memcpy(&chassis_receive.vy, payload + pos, 4); pos += 4;
                memcpy(&chassis_receive.vz, payload + pos, 4); pos += 4;
            }
            break;
            
        case fix_control:
            // 假设下位机也会回传这个包？通常控制包不需要回传，这里仅作示例
            break;
            
        case sentry_state:
        if (len >= 20) {
            memcpy(&sentry_state_receive.battery, payload + pos, 4); pos += 4;
            memcpy(&sentry_state_receive.life, payload + pos, 4); pos += 4;
            memcpy(&sentry_state_receive.color, payload + pos, 4); pos += 4;
            memcpy(&sentry_state_receive.bullet, payload + pos, 4); pos += 4;
            memcpy(&sentry_state_receive.fault_flag, payload + pos, 4); pos += 4;
        }
        break;
            default:
            // std::cout << "Received unknown ID: " << msg_id << std::endl;
            break;
    }
    
    // 可以在这里打印 Debug 信息
    // printf("[Protocol] Decoded ID 0x%02X, Len %d\n", msg_id, len);
}