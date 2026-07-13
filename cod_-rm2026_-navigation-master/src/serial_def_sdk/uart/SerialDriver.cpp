#include "SerialDriver.hpp"
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <unistd.h>     // UNIX standard function definitions
#include <iostream>
#include <cstring>      // for memset

SerialDriver::SerialDriver() : is_running_(false) {
    fd_.store(-1);
}

SerialDriver::~SerialDriver() {
    close();
}

bool SerialDriver::open(const std::string& device, int baudrate) {
    if (isOpen()) {
        close();
    }

    device_name_ = device;
    current_baudrate_ = baudrate;

    // 尝试打开串口
    int new_fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (new_fd == -1) {
        std::cerr << "[SerialDriver] Error: Unable to open " << device << ", will retry in background..." << std::endl;
        // 注意：这里即使失败也不 return false，而是让它启动线程，在后台死循环尝试重连
    } else {
        fd_.store(new_fd);
        configureTermios(current_baudrate_);
        tcflush(new_fd, TCIOFLUSH);
        std::cout << "[SerialDriver] Port " << device << " opened successfully." << std::endl;
    }

    // 如果线程还没跑，就启动它（确保只启动一次）
    // H1修复: 仅在成功打开时才启动读线程
    bool open_ok = (new_fd != -1);
    if (open_ok && !is_running_) {
        is_running_ = true;
        read_thread_ = std::thread(&SerialDriver::readLoop, this);
    }

    return open_ok;
}

void SerialDriver::close() {
    is_running_ = false;
    
    if (read_thread_.joinable()) {
        read_thread_.join();
    }

    int current_fd = fd_.exchange(-1);
    if (current_fd != -1) {
        ::close(current_fd);
        std::cout << "[SerialDriver] Port closed." << std::endl;
    }
}

bool SerialDriver::isOpen() const {
    return fd_.load() != -1;
}

void SerialDriver::setReceiverCallback(ReceiveCallback callback) {
    callback_ = callback;
}

int SerialDriver::write(const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(write_mutex_);

    // H3修复: 在锁内重新加载 fd，避免 TOCTOU 竞争
    int current_fd = fd_.load();
    if (current_fd == -1) return -1;

    int total_written = 0;
    int remaining = len;

    while (remaining > 0) {
        int n = ::write(current_fd, data + total_written, remaining);
        if (n <= 0) {
            if (errno != EAGAIN) {
                std::cerr << "\n[SerialDriver] WARNING: Write failed, device lost!" << std::endl;
                int expected = current_fd;
                if (fd_.compare_exchange_strong(expected, -1)) {
                    ::close(current_fd);
                }
            }
            return -1;
        }
        total_written += n;
        remaining -= n;
    }
    return total_written;
}

void SerialDriver::readLoop() {
    uint8_t buffer[1024];

    while (is_running_) {
        int current_fd = fd_.load();

        // ======= 核心重连逻辑 =======
        if (current_fd == -1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 等待0.5秒再重试
            int new_fd = ::open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
            if (new_fd != -1) {
                fd_.store(new_fd);
                configureTermios(current_baudrate_);
                tcflush(new_fd, TCIOFLUSH);
                std::cout << "\n[SerialDriver] SUCCESS: Device reconnected!" << std::endl;
            }
            continue; // 如果依然失败，进入下一个循环继续重试
        }
        // ===========================

        int n = ::read(current_fd, buffer, sizeof(buffer));

        if (n > 0) {
            if (callback_) {
                callback_(buffer, n);
            }
        } else if (n == 0 || (n < 0 && errno != EAGAIN)) {
            // 发生物理断开
            std::cerr << "\n[SerialDriver] WARNING: Device disconnected! Waiting to reconnect..." << std::endl;
            int expected = current_fd;
            if (fd_.compare_exchange_strong(expected, -1)) {
                ::close(current_fd);
            }
            // 注意：不要 break，让循环回到头部去执行上面的重连逻辑
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

bool SerialDriver::configureTermios(int baudrate) {
    int current_fd = fd_.load();
    if (current_fd == -1) return false;

    struct termios options;
    if (tcgetattr(current_fd, &options) != 0) {
        perror("[SerialDriver] tcgetattr failed");
        return false;
    }

    speed_t speed;
    switch (baudrate) {
        case 9600:   speed = B9600; break;
        case 115200: speed = B115200; break;
        case 460800: speed = B460800; break;
        default:     speed = B115200; break;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1; 

    if (tcsetattr(current_fd, TCSANOW, &options) != 0) {
        perror("[SerialDriver] tcsetattr failed");
        return false;
    }

    fcntl(current_fd, F_SETFL, 0); 

    return true;
}
