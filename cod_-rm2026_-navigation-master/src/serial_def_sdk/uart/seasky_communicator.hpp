#ifndef SEASKY_COMMUNICATOR_HPP
#define SEASKY_COMMUNICATOR_HPP

#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <memory>
#include "uart_hd.h" // 包含 UartLinux 的头文件

class SeaskyCommunicator {
public:
    // 回调函数类型，当检测到云台数据更新时被调用
    using GimbalDataCallback = std::function<void(float yaw, float pitch)>;

    SeaskyCommunicator(const std::string& device_name);
    ~SeaskyCommunicator();

    // 发送云台控制数据
    bool sendGimbalData(float yaw, float pitch, bool fire);

    // 注册一个回调函数，以便在接收到数据时通知上层
    void registerCallback(GimbalDataCallback callback);

private:
    void pollingLoop(); // 用于轮询数据更新的线程函数

    std::shared_ptr<UartLinux> uart_;
    std::thread polling_thread_;
    std::atomic<bool> is_running_{false};
    GimbalDataCallback data_callback_;
};

#endif // SEASKY_COMMUNICATOR_HPP