#include "fake_vel_transform/fake_vel_transform.hpp"

#include <tf2/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fake_vel_transform
{
FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "base_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "base_link_fake");
  this->declare_parameter<std::string>("odom_topic", "Odometry");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "cmd_vel");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "aft_cmd_vel");
  this->declare_parameter<float>("spin_speed", 0.0);
  this->declare_parameter<bool>("enable_vel_rotation", false);
  this->declare_parameter<double>("cmd_vel_timeout", 0.5);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("spin_speed", spin_speed_);
  this->get_parameter("enable_vel_rotation", enable_vel_rotation_);
  this->get_parameter("cmd_vel_timeout", cmd_vel_timeout_);
  last_cmd_time_ = this->now();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  watchdog_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(cmd_vel_timeout_ * 0.1)),
    std::bind(&FakeVelTransform::watchdogCallback, this));

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 1,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&FakeVelTransform::odomCallback, this, std::placeholders::_1));
}

void FakeVelTransform::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg->header.stamp;
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  t.transform.translation.x = msg->pose.pose.position.x;
  t.transform.translation.y = msg->pose.pose.position.y;
  t.transform.translation.z = msg->pose.pose.position.z;
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

// Transform the velocity from `robot_base_frame` to `fake_robot_base_frame`
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_time_ = this->now();
  float angle_diff = current_robot_base_angle_;

  geometry_msgs::msg::Twist aft_tf_vel;
  if (enable_vel_rotation_) {
    // Field-centric 模式：世界坐标 → 底盘坐标
    aft_tf_vel.linear.x = msg->linear.x * cos(angle_diff) + msg->linear.y * sin(angle_diff);
    aft_tf_vel.linear.y = -msg->linear.x * sin(angle_diff) + msg->linear.y * cos(angle_diff);
    aft_tf_vel.angular.z = (spin_speed_ != 0.0f) ? spin_speed_ : msg->angular.z;
  } else {
    // 直通模式：Nav2 输出直接给电控
    aft_tf_vel.linear.x = msg->linear.x;
    aft_tf_vel.linear.y = msg->linear.y;
    aft_tf_vel.angular.z = msg->angular.z;
  }

  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

void FakeVelTransform::watchdogCallback()
{
  auto now = this->now();
  auto elapsed = now - last_cmd_time_;
  if (elapsed.seconds() > cmd_vel_timeout_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "cmd_vel timeout. Publishing zero velocity.");
    geometry_msgs::msg::Twist zero_vel;
    cmd_vel_chassis_pub_->publish(zero_vel);
  }
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)