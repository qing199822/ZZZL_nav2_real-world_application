#ifndef SEASKY_PROTOCOL_HPP
#define SEASKY_PROTOCOL_HPP

#include "SerialDriver.hpp"
#include "DataType.h" // 引用 C 语言的数据定义
#include <mutex>
#include <vector>

// 协议常量定义
#define PROTOCOL_HEADER_ID  0xA5
#define MAX_PACKET_SIZE     256

class SeaskyProtocol {
public:
    SeaskyProtocol();
    ~SeaskyProtocol();

    // 初始化：打开串口
    bool init(const std::string& device_name);

    // 核心发送接口 (线程安全)
    // 根据 msg_id 自动打包 data 指向的结构体并发送
    bool send(int msg_id, void* data);

private:
    // 串口接收回调
    void onDataReceived(const uint8_t* data, size_t len);

    // 尝试从缓冲区解析一帧数据
    void processBuffer();

    // 具体的解包处理函数
    void dispatchMessage(uint16_t msg_id, const uint8_t* payload, uint16_t len);

    // 具体的打包辅助函数
    // payload_len: 纯数据长度
    // msg_id: 消息ID
    // payload: 数据内容
    void sendPacket(uint16_t msg_id, const uint8_t* payload, uint16_t payload_len);

private:
    SerialDriver driver_;
    
    // 接收缓冲区
    std::vector<uint8_t> rx_buffer_;
    std::mutex rx_mutex_; // 保护接收缓冲区

    // 发送缓冲区 (虽然 SerialDriver 有锁，但我们在打包过程中也需要保护 buffer)
    std::vector<uint8_t> tx_buffer_; 
    std::mutex send_mutex_; // 保护发送逻辑（防止多线程 send 时数据错乱）
};

#endif // SEASKY_PROTOCOL_HPP