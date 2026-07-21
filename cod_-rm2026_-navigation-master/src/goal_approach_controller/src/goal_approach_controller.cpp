// goal_approach_controller.cpp
// Nav2 controller wrapper: limits velocity when approaching goal to prevent overshooting.
// Principle: transparent proxy wrapping inner controller (e.g. MPPI),
//            clamps resultant speed below approach_velocity when within approach_distance.
//            It never replaces the direction selected and collision-checked
//            by the inner controller.

#include <cmath>
#include <memory>
#include <string>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace goal_approach_controller
{

class GoalApproachController : public nav2_core::Controller
{
public:
  GoalApproachController() = default;
  ~GoalApproachController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    // Declare wrapper parameters
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".inner_plugin",
      rclcpp::ParameterValue("nav2_mppi_controller::MPPIController"));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".approach_distance",
      rclcpp::ParameterValue(1.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".approach_velocity",
      rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".approach_kp",
      rclcpp::ParameterValue(1.0));

    std::string inner_plugin_type;
    node->get_parameter(name + ".inner_plugin", inner_plugin_type);
    node->get_parameter(name + ".approach_distance", approach_distance_);
    node->get_parameter(name + ".approach_velocity", approach_velocity_);
    node->get_parameter(name + ".approach_kp", approach_kp_);

    // Load inner controller via pluginlib
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core", "nav2_core::Controller");
    inner_controller_ = loader_->createUniqueInstance(inner_plugin_type);
    inner_controller_->configure(parent, name, tf, costmap_ros);

    RCLCPP_INFO(
      logger_,
      "GoalApproachController: wrapping [%s], approach_distance=%.2f m, "
      "approach_velocity=%.2f m/s, approach_kp=%.2f",
      inner_plugin_type.c_str(), approach_distance_, approach_velocity_, approach_kp_);
  }

  void cleanup() override
  {
    inner_controller_->cleanup();
  }

  void activate() override
  {
    inner_controller_->activate();
  }

  void deactivate() override
  {
    inner_controller_->deactivate();
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if (!path.poses.empty()) {
      goal_ = path.poses.back();
    }
    inner_controller_->setPlan(path);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    auto cmd = inner_controller_->computeVelocityCommands(pose, velocity, goal_checker);

    // Transform goal to robot base frame for consistent distance calculation
    geometry_msgs::msg::PoseStamped goal_in_base;
    double dist = std::numeric_limits<double>::max();
    if (tf_ && !goal_.header.frame_id.empty()) {
      try {
        goal_in_base = tf_->transform(
          goal_, costmap_ros_->getBaseFrameID(), tf2::durationFromSec(0.1));
        dist = std::hypot(
          goal_in_base.pose.position.x, goal_in_base.pose.position.y);
      } catch (const tf2::TransformException & ex) {
        // TF failure: skip approach logic, return inner controller output unchanged
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
          "GoalApproachController: TF failed for goal (%s). "
          "Skipping approach control, passing through inner controller output.", ex.what());
        return cmd;
      }
    } else {
      // No TF or empty frame_id: skip approach logic
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
        "GoalApproachController: TF unavailable or goal frame_id empty. "
        "Skipping approach control.");
      return cmd;
    }

    if (dist < approach_distance_) {
      // Limit only the magnitude. The MPPI-selected direction remains unchanged.
      const double allowed_speed = std::min(approach_velocity_, dist * approach_kp_);
      double speed = std::hypot(cmd.twist.linear.x, cmd.twist.linear.y);
      if (speed > allowed_speed && speed > 0.0) {
        double scale = allowed_speed / speed;
        cmd.twist.linear.x *= scale;
        cmd.twist.linear.y *= scale;
      }
    }

    // Safety: reject NaN/Inf
    if (!std::isfinite(cmd.twist.linear.x) || !std::isfinite(cmd.twist.linear.y) ||
        !std::isfinite(cmd.twist.angular.z)) {
      RCLCPP_ERROR(logger_, "GoalApproachController: NaN/Inf detected, zeroing velocity.");
      cmd.twist.linear.x = 0.0;
      cmd.twist.linear.y = 0.0;
      cmd.twist.angular.z = 0.0;
    }

    return cmd;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    inner_controller_->setSpeedLimit(speed_limit, percentage);
  }

private:
  pluginlib::UniquePtr<nav2_core::Controller> inner_controller_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("goal_approach_controller")};
  rclcpp::Clock::SharedPtr clock_;
  geometry_msgs::msg::PoseStamped goal_;
  double approach_distance_{1.5};
  double approach_velocity_{0.5};
  double approach_kp_{1.0};
};

}  // namespace goal_approach_controller

PLUGINLIB_EXPORT_CLASS(
  goal_approach_controller::GoalApproachController,
  nav2_core::Controller)
