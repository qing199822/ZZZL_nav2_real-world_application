#include "uart_transporter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cstring>
#include <chrono>
#include <thread>

class CmdVelSubscriber : public rclcpp::Node
{
public:
  CmdVelSubscriber()
  : Node("cmd_vel_subscriber"),
    reconnect_count_(0),
    reconnect_in_progress_(false)
  {
    uart_ = std::make_unique<UartTransporter>("/dev/ttyACM0", 115200);
    tryReconnect();

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "aft_cmd_vel", 10, std::bind(&CmdVelSubscriber::topic_callback, this, std::placeholders::_1));
  }

  ~CmdVelSubscriber() {
    if (uart_) {
      uart_->close();
    }
  }

private:
  void tryReconnect() {
    if (reconnect_in_progress_) return;
    reconnect_in_progress_ = true;
    if (uart_->open()) {
      reconnect_count_ = 0;
    } else {
      reconnect_count_++;
      auto delay = std::min(5.0, 0.5 * reconnect_count_);
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay * 1000)));
    }
    reconnect_in_progress_ = false;
  }

  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 创建数据包
    uint8_t header = 0xA5;
    float vx = msg->linear.x;
    float vy = msg->linear.y;
    float vz = msg->linear.z;

    uint8_t packet[15];
    packet[0] = header;
    std::memcpy(&packet[1], &vx, sizeof(float));
    std::memcpy(&packet[5], &vy, sizeof(float));
    std::memcpy(&packet[9], &vz, sizeof(float));

    uint16_t checksum = header;
    for (int i = 1; i < 13; ++i) {
      checksum += packet[i];
    }
    std::memcpy(&packet[13], &checksum, sizeof(uint16_t));

    // 日志降级：DEBUG 级别
    RCLCPP_DEBUG(get_logger(), "Sent cmd_vel packet: vx=%.3f vy=%.3f vz=%.3f", vx, vy, vz);

    // 直接写入已打开的串口
    if (uart_ && uart_->isOpen()) {
      size_t written = uart_->writeBuffer(packet, sizeof(packet));
      if (written != sizeof(packet)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Serial write incomplete: %zu/%zu bytes", written, sizeof(packet));
      }
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Serial not open, attempting reconnection");
      tryReconnect();
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  std::unique_ptr<UartTransporter> uart_;
  mutable int reconnect_count_;
  mutable bool reconnect_in_progress_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSubscriber>());
  rclcpp::shutdown();
  return 0;
}
