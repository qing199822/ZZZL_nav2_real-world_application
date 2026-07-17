#include "seasky_communicator.hpp"
#include <iostream>
#include <chrono>

// 包含所有必要的C头文件
extern "C" {
#include "uart.h"
#include "DataType.h"
}

SeaskyCommunicator::SeaskyCommunicator(const std::string& device_name) {
    // 1. 初始化C语言库
    defUartInit();

    // 2. 初始化 UartLinux 并将其指针交给C代码
    uart_ = std::make_shared<UartLinux>();
    extern std::shared_ptr<UartLinux> uart; // 引用 uart_hd.cpp 中的全局指针
    uart = uart_;

    // 3. 打开串口并启动后台接收线程
    if (!uart_->setUartName(device_name)) {
        std::cerr << "Fatal: Failed to initialize UART device: " << device_name << std::endl;
        exit(-1);
    }
    uart_->startReading(); // UartLinux 自己的C++接收线程开始工作

    // 4. 启动我们自己的轮询线程
    is_running_ = true;
    polling_thread_ = std::thread(&SeaskyCommunicator::pollingLoop, this);
}

SeaskyCommunicator::~SeaskyCommunicator() {
    is_running_ = false;
    if (polling_thread_.joinable()) {
        polling_thread_.join();
    }
    uart_->stopReading();
    defUartDeinit();
}

void SeaskyCommunicator::registerCallback(GimbalDataCallback callback) {
    data_callback_ = callback;
}

bool SeaskyCommunicator::sendGimbalData(float yaw, float pitch, bool fire) {
    // 调用C库的发送流程
    // a. 更新全局发送结构体
    gimbal_send.yaw = yaw;
    gimbal_send.pitch = pitch;
    gimbal_send.fire = fire ? 1 : 0;

    // b. 调用发送函数
    return defUartSend(gimbal); // `gimbal` 是ID
}

void SeaskyCommunicator::pollingLoop() {
    // 引用C代码中的全局接收结构体
    extern Gimbal gimbal_receive;

    // 记录上一次的值，只有在值变化时才触发回调
    float last_yaw = gimbal_receive.yaw;
    float last_pitch = gimbal_receive.pitch;
    bool last_fire = gimbal_receive.fire; // 新增

    std::cout << "[Communicator] Polling thread started." << std::endl;

    while (is_running_) {
        // 非阻塞地检查C代码中的全局变量是否被 `defUartRead`->`handleByteReceived` 更新了
        if (gimbal_receive.yaw != last_yaw || 
            gimbal_receive.pitch != last_pitch) {
            
            // 值发生了变化，更新我们的记录
            last_yaw = gimbal_receive.yaw;
            last_pitch = gimbal_receive.pitch;
            //last_fire = gimbal_receive.fire;

            // 如果上层注册了回调函数，就调用它
            if (data_callback_) {
                data_callback_(last_yaw, last_pitch);
            }
        }

        // 轮询间隔，避免CPU占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout << "[Communicator] Polling thread stopped." << std::endl;
}