#include "uart_transporter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cstring>
#include <memory>

class CmdVelSubscriber : public rclcpp::Node
{
public:
  CmdVelSubscriber()
  : Node("cmd_vel_subscriber")
  {
    uart_ = std::make_unique<UartTransporter>("/dev/ttyACM0", 115200);

    // Open serial port once at startup, keep persistent connection
    if (uart_->open()) {
      RCLCPP_INFO(this->get_logger(), "Serial port /dev/ttyACM0 opened successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s",
                   uart_->errorMessage().c_str());
    }

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "aft_cmd_vel", 10,
      std::bind(&CmdVelSubscriber::topic_callback, this, std::placeholders::_1));
  }

  ~CmdVelSubscriber()
  {
    if (uart_ && uart_->isOpen()) {
      uart_->close();
      RCLCPP_INFO(this->get_logger(), "Serial port closed");
    }
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    // Build packet
    const uint8_t header = 0xA5;
    float vx = msg->linear.x;
    float vy = msg->linear.y;
    float vz = msg->linear.z;

    // Convert to byte array
    uint8_t packet[15];
    packet[0] = header;
    std::memcpy(&packet[1], &vx, sizeof(float));
    std::memcpy(&packet[5], &vy, sizeof(float));
    std::memcpy(&packet[9], &vz, sizeof(float));

    // Calculate checksum (simple sum of header + data bytes)
    uint16_t checksum = header;
    for (int i = 1; i < 13; ++i) {
      checksum += packet[i];
    }
    std::memcpy(&packet[13], &checksum, sizeof(uint16_t));

    // Send via persistent connection
    if (uart_ && uart_->isOpen()) {
      int written = uart_->writeBuffer(packet, sizeof(packet));
      if (written != sizeof(packet)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), const_cast<rclcpp::Clock&>(*this->get_clock()), 1000,
          "Serial write incomplete: %d/%zu bytes", written, sizeof(packet));
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), const_cast<rclcpp::Clock&>(*this->get_clock()), 5000,
        "Serial port not open. Attempting reconnect...");
      // Attempt reconnection with backoff
      const_cast<CmdVelSubscriber*>(this)->tryReconnect();
    }

    // Debug packet dump (only at DEBUG level to avoid flooding)
    RCLCPP_DEBUG(this->get_logger(), "Sent cmd_vel packet: vx=%.3f vy=%.3f vz=%.3f", vx, vy, vz);
  }

  void tryReconnect()
  {
    if (reconnect_in_progress_) {
      return;
    }
    reconnect_in_progress_ = true;

    if (uart_->open()) {
      RCLCPP_INFO(this->get_logger(), "Serial port reconnected successfully");
      reconnect_count_ = 0;
    } else {
      reconnect_count_++;
      auto delay = std::min(5.0, 0.5 * reconnect_count_);  // Exponential backoff capped at 5s
      RCLCPP_WARN(this->get_logger(), "Reconnect failed (#%d), retrying in %.1fs",
                  reconnect_count_, delay);
    }
    reconnect_in_progress_ = false;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  std::unique_ptr<UartTransporter> uart_;
  mutable int reconnect_count_{0};
  mutable bool reconnect_in_progress_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSubscriber>());
  rclcpp::shutdown();
  return 0;
}
