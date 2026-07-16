#include "uart.h"          // 包含旧的 C 接口声明
#include "SeaskyProtocol.hpp"
#include "DataType.h"
#include <memory>
#include <iostream>

// 全局的协议实例
// 使用 unique_ptr 管理生命周期
static std::unique_ptr<SeaskyProtocol> g_protocol = nullptr;

// 为了兼容 uart_hd.cpp 中的全局变量声明 (虽然 uart_hd.cpp 已经被删了，但外部可能有 extern 引用)
// 我们在这里重新定义它，以免链接错误
// 其实 SeaskyProtocol 内部已经不需要这个全局指针了，但为了保险起见：
#include "uart_hd.h"
std::shared_ptr<UartLinux> uart = nullptr; // 留个空的给旧代码看


// =================================================================
// C 接口实现 (对应 uart.h)
// =================================================================

extern "C" {

// 1. 初始化
void defUartInit() {
    if (!g_protocol) {
        g_protocol = std::make_unique<SeaskyProtocol>();
        
        // 这里硬编码了设备名，模拟原代码的行为。
        // 原代码是在 main.cpp 里调 uart->setUartName("/dev/ttyUSB0") 初始化的。
        // 但 defUartInit 在 main 开头就被调用了。
        // 为了更好的设计，我们让 g_protocol 在 send 的时候懒加载，或者在 setUartName 时初始化。
        // 这里暂时留空，真正的 open 放到 setUartName 逻辑里（见下文）。
        std::cout << "[Interface] Protocol instance created." << std::endl;
    }
}

// 2. 反初始化
void defUartDeinit() {
    g_protocol.reset();
    std::cout << "[Interface] Protocol instance destroyed." << std::endl;
}

// 3. 发送函数 (核心)
bool defUartSend(int id) {
    if (!g_protocol) {
        std::cerr << "[Interface] Error: Protocol not initialized!" << std::endl;
        return false;
    }

    // 根据 ID 选择对应的全局变量进行发送
    // 这些全局变量在 DataType.c 中定义
    switch (id) {
        case heartbeat:
            return g_protocol->send(id, &heartbeat_send);
        case gimbal:
            return g_protocol->send(id, &gimbal_send);
        case control:
            return g_protocol->send(id, &control_data);
        case chassis:
            return g_protocol->send(id, &chassis_send);
        
        // --- 新增：FixControl 支持 ---
        case fix_control:
            return g_protocol->send(id, &fix_control_send);

        default:
            std::cerr << "[Interface] Warning: Unknown send ID " << id << std::endl;
            return false;
    }
}

// 4. 接收函数 (废弃但保留空壳)
// 原代码中，main.cpp 的线程会调用这个函数把数据塞给 uart.c。
// 现在我们的 SerialDriver 自己有线程，不需要外部喂数据了。
// 所以这个函数留空即可，或者打印个警告。
void defUartRead(uint8_t* readbuff, int nread) {
    // Do nothing. 
    // The new driver handles reading internally.
}

// 5. 辅助函数 (保留空壳以防链接错误)
void cntLossConnection() {}
void print_frame(const char *desc, uint8_t *buf, int size) {}
bool WriteDataWapper(const unsigned char* pData, unsigned int length) { return false; }

} // extern "C"


// =================================================================
// UartLinux 兼容层 (对应 uart_hd.h)
// =================================================================
// 你的 main.cpp 和 seasky_communicator.cpp 依然在使用 UartLinux 类。
// 我们必须为了它们保留这个类的定义，但把内部实现嫁接到 g_protocol 上。

// 注意：这里我们不需要新建 uart_hd.cpp，直接在这里实现成员函数即可。
// 因为 UartLinux 在 uart_hd.h 里定义了。
// 但是为了代码整洁，通常还是建议放在 .cpp 里。
// 让我们把这些函数的实现写在下面：

void UartLinux::startReading() {
    // 新驱动自动开始读取，这里不需要做任何事
    std::cout << "[Compat] startReading called (auto-handled by new driver)." << std::endl;
}

void UartLinux::stopReading() {
    // 同上
}

void UartLinux::readLoop() {
    // 废弃
}

bool UartLinux::setUartName(const string& name) {
    if (!g_protocol) {
        defUartInit();
    }
    // 真正的打开串口操作
    return g_protocol->init(name);
}

// 其他未使用的函数留空
string UartLinux::get_uart_name() { return ""; }
bool UartLinux::init_port(int, char, int, int) { return true; }
int UartLinux::set_opt(int, int, char, int, int) { return 0; }
bool UartLinux::WriteData(const unsigned char*, unsigned int) { return true; }
bool UartLinux::ReadData() { return true; }