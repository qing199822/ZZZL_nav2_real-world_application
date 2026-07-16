#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <def_msg/msg/gimble_control.hpp>
#include "def_msg/msg/gimble_position.hpp"

class Publisher : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    Publisher(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "success");
        // 创建发布者
        command_publisher_ = this->create_publisher<def_msg::msg::GimblePosition>("vision/gimble_control", 10);
        // 创建定时器，500ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Publisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 创建消息
        def_msg::msg::GimblePosition msg;
        msg.yaw = 32.1;
        msg.pitch = 15.3;
        // 日志打印
        RCLCPP_INFO(this->get_logger(), "Publishing");
        // 发布消息
        command_publisher_->publish(msg);
    }
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者指针
    rclcpp::Publisher<def_msg::msg::GimblePosition>::SharedPtr command_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<Publisher>("publisher");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}