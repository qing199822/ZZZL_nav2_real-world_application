#ifndef SERIAL_DRIVER_HPP
#define SERIAL_DRIVER_HPP

#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>

class SerialDriver {
public:
    // 定义接收回调函数类型: void(数据指针, 数据长度)
    using ReceiveCallback = std::function<void(const uint8_t*, size_t)>;

    SerialDriver();
    ~SerialDriver();

    // 打开串口
    // device: 设备路径 (e.g., "/dev/ttyUSB0")
    // baudrate: 波特率 (默认 115200)
    bool open(const std::string& device, int baudrate = 115200);

    // 关闭串口
    void close();

    // 发送数据 (线程安全)
    // return: 实际发送的字节数，失败返回 -1
    int write(const uint8_t* data, size_t len);

    // 设置接收回调函数
    // 当串口读到数据时，会通过这个函数通知上层
    void setReceiverCallback(ReceiveCallback callback);

    // 检查串口是否开启
    bool isOpen() const;

private:
    void readLoop(); // 后台读取线程的主体
    bool configureTermios(int baudrate); // 配置串口属性 (8N1, Raw Mode)

private:
    std::string device_name_;
    std::atomic<int> fd_{-1};      // 改为原子类型，防止多线程断线重连时引发竞态问题
    int current_baudrate_{115200}; // 保存当前的波特率，用于重连
    
    std::atomic<bool> is_running_; // 线程运行标志
    std::thread read_thread_;      // 读取线程

    ReceiveCallback callback_;     // 接收数据的回调
    std::mutex write_mutex_;       // 发送锁 (防止多线程同时写入导致数据交错)
};

#endif // SERIAL_DRIVER_HPP
