#ifndef UART_COMMON_H__
#define UART_COMMON_H__

#include <string>
#include <memory>
#include <iostream>
#include <vector>

using namespace std;

// 这是一个兼容类，用于让旧的 main.cpp 和 seasky_communicator.hpp 能通过编译
// 它的实际实现都在 src/uart_interface.cpp 中转发给了新的 SeaskyProtocol

class UartLinux {
public:
    // 构造函数：参数保留但实际上可能不使用
    UartLinux(string uart_name = "/dev/ttyUSB0", int nSpeed = 115200, char nEvent = 'N', int nBits = 8, int nStop = 1) {}
    ~UartLinux() {}

    // 核心兼容接口
    bool setUartName(const string& name); // 在 uart_interface.cpp 中实现
    void startReading();                  // 在 uart_interface.cpp 中实现
    void stopReading();                   // 在 uart_interface.cpp 中实现

    // 废弃接口（保留定义防止编译报错）
    string get_uart_name();
    bool init_port(int nSpeed, char nEvent, int nBits, int nStop);
    int set_opt(int fd, int nSpeed, char nEvent, int nBits, int nStop);
    bool WriteData(const unsigned char* pData, unsigned int length);
    bool ReadData();
    
    // 废弃的成员变量（如果旧代码访问了它们，这里保留以防报错）
    void readLoop(); 
};

// 全局指针声明
extern std::shared_ptr<UartLinux> uart;

#endif // UART_COMMON_H__