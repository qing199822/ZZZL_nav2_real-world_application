// goal_approach_controller.cpp  // 本文件实现 Nav2 的目标接近控制器包装插件
// Nav2 controller wrapper: limits velocity when approaching goal to prevent overshooting.  // 作用：机器人接近目标点时限制速度，降低高速冲过终点的概率
// Principle: transparent proxy wrapping inner controller (e.g. MPPI),  // 原理：外层插件通过透明代理方式包装一个内部控制器，例如 MPPI
//            clamps resultant speed below approach_velocity when within approach_distance.  // 进入接近区域后，对内部控制器生成的速度做上限限制
//            In direct-approach zone, takes over with direct heading drive but keeps collision checks.  // 进入直达区域后覆盖线速度；注意当前实现没有对覆盖后的速度重新执行碰撞检查

#include <cmath>  // 提供 hypot、isfinite 等数学计算函数
#include <memory>  // 提供 unique_ptr、shared_ptr 和 make_unique 等智能指针工具
#include <string>  // 提供 std::string 字符串类型

#include "nav2_core/controller.hpp"  // 引入 Nav2 控制器插件必须实现的抽象接口
#include "nav2_costmap_2d/costmap_2d_ros.hpp"  // 引入 Nav2 代价地图 ROS 包装类，用于获取机器人坐标系等信息
#include "nav2_costmap_2d/footprint_collision_checker.hpp"  // 引入机器人轮廓碰撞检查器；当前文件虽包含该头文件但没有实际调用
#include "nav2_util/node_utils.hpp"  // 引入 Nav2 参数声明辅助函数
#include "pluginlib/class_list_macros.hpp"  // 引入将本类导出为 ROS 插件所需的宏
#include "pluginlib/class_loader.hpp"  // 引入运行时动态加载内部控制器插件的 ClassLoader
#include "rclcpp/rclcpp.hpp"  // 引入 ROS 2 C++ 节点、参数、时间和日志接口

namespace goal_approach_controller  // 使用独立命名空间，避免控制器类名与其他功能包冲突
{  // goal_approach_controller 命名空间开始

class GoalApproachController : public nav2_core::Controller  // 定义目标接近控制器，并实现 Nav2 Controller 插件接口
{  // GoalApproachController 类定义开始
public:  // 以下构造、生命周期和控制接口允许 Nav2 Controller Server 调用
  GoalApproachController() = default;  // 使用编译器生成的默认构造函数
  ~GoalApproachController() override = default;  // 使用默认析构函数，并明确覆盖基类虚析构函数

  void configure(  // Nav2 配置阶段调用，用于读取参数并创建内部控制器
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,  // 接收 Controller Server 生命周期节点的弱指针，避免循环引用
    std::string name,  // 接收本插件实例名，通常为参数文件中的 FollowPath
    std::shared_ptr<tf2_ros::Buffer> tf,  // 接收 Nav2 共享 TF 缓冲区，用于坐标变换
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override  // 接收局部代价地图对象，并覆盖基类配置接口
  {  // configure 函数体开始
    auto node = parent.lock();  // 将生命周期节点弱指针提升为共享指针，以便安全访问节点接口
    logger_ = node->get_logger();  // 保存节点日志器，供后续输出运行信息和错误信息
    clock_ = node->get_clock();  // 保存节点时钟，供节流日志判断时间间隔
    tf_ = tf;  // 保存 TF 缓冲区共享指针
    costmap_ros_ = costmap_ros;  // 保存代价地图 ROS 对象共享指针

    // Declare wrapper parameters  // 声明包装器自身使用的参数，已经由 YAML 声明时不会重复声明
    nav2_util::declare_parameter_if_not_declared(  // 声明内部控制器类型参数
      node, name + ".inner_plugin",  // 参数完整名称由插件实例名与 inner_plugin 组成
      rclcpp::ParameterValue("nav2_mppi_controller::MPPIController"));  // 默认把 Nav2 MPPI 控制器作为内部控制器
    nav2_util::declare_parameter_if_not_declared(  // 声明开始接近限速的距离参数
      node, name + ".approach_distance",  // 参数完整名称通常为 FollowPath.approach_distance
      rclcpp::ParameterValue(1.5));  // 未在 YAML 配置时默认距离为 1.5 米
    nav2_util::declare_parameter_if_not_declared(  // 声明接近目标时的线速度上限参数
      node, name + ".approach_velocity",  // 参数完整名称通常为 FollowPath.approach_velocity
      rclcpp::ParameterValue(0.5));  // 未在 YAML 配置时默认速度上限为 0.5 米每秒
    nav2_util::declare_parameter_if_not_declared(  // 声明切换到直达目标模式的距离参数
      node, name + ".direct_approach_distance",  // 参数完整名称通常为 FollowPath.direct_approach_distance
      rclcpp::ParameterValue(0.5));  // 未在 YAML 配置时默认在距离目标 0.5 米内进入直达模式
    nav2_util::declare_parameter_if_not_declared(  // 声明直达模式距离比例控制增益参数
      node, name + ".direct_approach_kp",  // 参数完整名称通常为 FollowPath.direct_approach_kp
      rclcpp::ParameterValue(1.0));  // 未在 YAML 配置时默认比例增益为 1.0

    std::string inner_plugin_type;  // 创建字符串变量，用于保存内部控制器的插件类型名称
    node->get_parameter(name + ".inner_plugin", inner_plugin_type);  // 从参数服务器读取内部控制器类型
    node->get_parameter(name + ".approach_distance", approach_distance_);  // 读取接近限速区域的距离阈值
    node->get_parameter(name + ".approach_velocity", approach_velocity_);  // 读取接近目标时允许的最大速度
    node->get_parameter(name + ".direct_approach_distance", direct_approach_distance_);  // 读取直达目标模式的距离阈值
    node->get_parameter(name + ".direct_approach_kp", direct_approach_kp_);  // 读取直达目标比例速度增益

    // Load inner controller via pluginlib  // 使用 pluginlib 根据配置名称在运行时创建内部控制器
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(  // 创建以 nav2_core::Controller 为基类的插件加载器
      "nav2_core", "nav2_core::Controller");  // 指定插件接口所在包以及插件基类的完整类型名
    inner_controller_ = loader_->createUniqueInstance(inner_plugin_type);  // 创建参数指定的内部控制器唯一实例
    inner_controller_->configure(parent, name, tf, costmap_ros);  // 把相同节点、插件名、TF 和代价地图传给内部控制器完成配置

    RCLCPP_INFO(  // 输出包装器和内部控制器的最终配置，方便启动时核对参数
      logger_,  // 使用 configure 阶段保存的 ROS 2 日志器
      "GoalApproachController: wrapping [%s], approach_distance=%.2f m, "  // 日志第一段打印内部插件和接近距离
      "approach_velocity=%.2f m/s, direct_approach_distance=%.2f m, direct_approach_kp=%.2f",  // 日志第二段打印限速和直达参数
      inner_plugin_type.c_str(), approach_distance_, approach_velocity_,  // 依次提供内部插件名、接近距离和接近速度格式化参数
      direct_approach_distance_, direct_approach_kp_);  // 提供直达距离和比例增益格式化参数并结束日志调用
  }  // configure 函数结束

  void cleanup() override  // Nav2 清理阶段调用，用于释放控制器配置资源
  {  // cleanup 函数体开始
    inner_controller_->cleanup();  // 将清理操作继续转发给内部控制器
  }  // cleanup 函数结束

  void activate() override  // Nav2 激活阶段调用，使控制器进入可运行状态
  {  // activate 函数体开始
    inner_controller_->activate();  // 将激活操作继续转发给内部控制器
  }  // activate 函数结束

  void deactivate() override  // Nav2 停用阶段调用，使控制器停止工作
  {  // deactivate 函数体开始
    inner_controller_->deactivate();  // 将停用操作继续转发给内部控制器
  }  // deactivate 函数结束

  void setPlan(const nav_msgs::msg::Path & path) override  // 接收规划器生成的新路径，并覆盖 Nav2 Controller 接口
  {  // setPlan 函数体开始
    if (!path.poses.empty()) {  // 仅在路径至少包含一个位姿时读取终点，避免访问空容器
      goal_ = path.poses.back();  // 保存路径最后一个位姿，作为后续距离计算使用的目标点
    }  // 路径非空判断结束
    inner_controller_->setPlan(path);  // 把完整路径继续传给 MPPI 等内部控制器进行跟踪
  }  // setPlan 函数结束

  geometry_msgs::msg::TwistStamped computeVelocityCommands(  // 计算本控制周期最终速度指令，是控制器的核心函数
    const geometry_msgs::msg::PoseStamped & pose,  // 接收机器人当前位姿
    const geometry_msgs::msg::Twist & velocity,  // 接收机器人当前测量速度
    nav2_core::GoalChecker * goal_checker) override  // 接收目标到达检查器，并覆盖 Nav2 Controller 计算接口
  {  // computeVelocityCommands 函数体开始
    auto cmd = inner_controller_->computeVelocityCommands(pose, velocity, goal_checker);  // 先让内部控制器根据路径、代价地图和当前状态生成原始速度

    // Transform goal to robot base frame for consistent distance calculation  // 把目标转换到机器人坐标系，统一计算相对方向和距离
    geometry_msgs::msg::PoseStamped goal_in_base;  // 创建用于保存 base frame 下目标位姿的消息
    double dist = std::numeric_limits<double>::max();  // 将目标距离初始化为 double 最大值，防止转换前误进入接近模式
    double dx = 0.0, dy = 0.0;  // 初始化目标相对机器人在 x、y 方向上的位置差

    if (tf_ && !goal_.header.frame_id.empty()) {  // 确认 TF 缓冲区存在且保存的目标带有有效坐标系名称
      try {  // 开始捕获目标坐标变换可能抛出的异常
        goal_in_base = tf_->transform(  // 请求 TF 将保存的目标位姿转换到机器人本体坐标系
          goal_, costmap_ros_->getBaseFrameID(), tf2::durationFromSec(0.1));  // 使用代价地图的 base frame，并最多等待 0.1 秒
        dx = goal_in_base.pose.position.x;  // 读取目标在机器人前后方向上的相对距离
        dy = goal_in_base.pose.position.y;  // 读取目标在机器人左右方向上的相对距离
        dist = std::hypot(dx, dy);  // 计算机器人到目标点的二维欧氏距离
      } catch (const tf2::TransformException & ex) {  // 捕获 TF 不存在、超时或时间不同步等变换异常
        // TF failure: skip approach logic, return inner controller output unchanged  // TF 失败时跳过外层限速与直达逻辑，保留内部控制器结果
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,  // 每 1000 毫秒最多输出一次 TF 失败警告，避免刷屏
          "GoalApproachController: TF failed for goal (%s). "  // 日志第一段说明目标坐标变换失败并预留异常字符串
          "Skipping approach control, passing through inner controller output.", ex.what());  // 日志第二段说明直接返回内部控制器结果，并填入异常原因
        return cmd;  // 返回 MPPI 等内部控制器原始速度，不再执行后续接近逻辑
      }  // TF 异常捕获块结束
    } else {  // TF 缓冲区无效或目标坐标系名称为空时执行降级处理
      // No TF or empty frame_id: skip approach logic  // 缺少变换条件时不能可靠计算目标相对距离，因此跳过外层控制
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,  // 每 5000 毫秒最多输出一次缺少 TF 条件的警告
        "GoalApproachController: TF unavailable or goal frame_id empty. "  // 日志第一段说明 TF 或目标坐标系不可用
        "Skipping approach control.");  // 日志第二段说明已跳过目标接近控制
      return cmd;  // 直接返回内部控制器计算出的速度
    }  // TF 可用性判断结束

    if (dist < direct_approach_distance_) {  // 距离小于直达阈值时进入直接朝目标运动的控制分支
      // Direct drive mode: bypass MPPI arc output, drive straight to goal  // 直达模式会覆盖 MPPI 计算的线速度方向，消除接近终点时可能出现的弧线
      // BUT retain collision checking  // 原注释声称保留碰撞检查；但当前代码未对下方新生成的直达速度重新进行碰撞预测
      double target_speed = std::min(approach_velocity_, dist * direct_approach_kp_);  // 按距离比例计算速度，并用 approach_velocity_ 限制最大值

      if (dist > 0.01) {  // 目标距离大于 1 厘米时才做单位方向向量除法，避免除零或数值放大
        cmd.twist.linear.x = target_speed * (dx / dist);  // 用目标单位方向的 x 分量生成机器人前后线速度
        cmd.twist.linear.y = target_speed * (dy / dist);  // 用目标单位方向的 y 分量生成机器人横移线速度
      } else {  // 已非常接近目标位置时停止平移
        cmd.twist.linear.x = 0.0;  // 将机器人前后线速度清零
        cmd.twist.linear.y = 0.0;  // 将机器人横移线速度清零
      }  // 目标距离防除零判断结束
      // Keep internal controller's angular.z for orientation alignment  // 不覆盖内部控制器计算的角速度，让其继续负责最终朝向调整
      // (don't force to 0.0 - goal checker may require final yaw)  // 若目标检查器要求最终 yaw，保留角速度可继续旋转对准目标方向
    } else if (dist < approach_distance_) {  // 未进入直达区但已进入接近区时，仅限制内部控制器输出速度
      // Speed limiting mode: upper-bound only (don't amplify slow speeds)  // 限速只降低超过上限的速度，不会放大本来较慢的指令
      double speed = std::hypot(cmd.twist.linear.x, cmd.twist.linear.y);  // 计算内部控制器输出的二维合线速度大小
      if (speed > approach_velocity_) {  // 仅当合线速度超过配置上限时执行缩放
        double scale = approach_velocity_ / speed;  // 计算把当前速度压到上限所需的等比例缩放系数
        cmd.twist.linear.x *= scale;  // 按相同比例缩小 x 方向线速度，保持原运动方向
        cmd.twist.linear.y *= scale;  // 按相同比例缩小 y 方向线速度，保持原运动方向
        // Scale angular proportionally to avoid spinning-in-place  // 角速度同时缩放，避免线速度降低后机器人相对旋转过快
        cmd.twist.angular.z *= scale;  // 使用相同系数缩小绕 z 轴旋转速度
      }  // 速度是否超过上限的判断结束
    }  // 直达模式与普通接近限速模式的选择结束

    // Safety: reject NaN/Inf  // 发布前检查速度中是否出现非法浮点数，防止异常指令进入底盘
    if (!std::isfinite(cmd.twist.linear.x) || !std::isfinite(cmd.twist.linear.y) ||  // 检查 x、y 线速度是否都是有限数值
        !std::isfinite(cmd.twist.angular.z)) {  // 检查 z 轴角速度是否为有限数值
      RCLCPP_ERROR(logger_, "GoalApproachController: NaN/Inf detected, zeroing velocity.");  // 发现 NaN 或无穷大时输出错误日志
      cmd.twist.linear.x = 0.0;  // 将异常情况下的 x 线速度清零
      cmd.twist.linear.y = 0.0;  // 将异常情况下的 y 线速度清零
      cmd.twist.angular.z = 0.0;  // 将异常情况下的角速度清零
    }  // 非法浮点速度检查结束

    return cmd;  // 返回经过直达处理、接近限速和数值安全检查后的最终速度指令
  }  // computeVelocityCommands 函数结束

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override  // 接收 Nav2 外部速度限制请求
  {  // setSpeedLimit 函数体开始
    inner_controller_->setSpeedLimit(speed_limit, percentage);  // 将绝对值或百分比速度限制转发给内部控制器处理
  }  // setSpeedLimit 函数结束

private:  // 以下运行状态和资源仅供本控制器类内部使用
  pluginlib::UniquePtr<nav2_core::Controller> inner_controller_;  // 保存通过 pluginlib 创建的内部控制器唯一实例
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;  // 保存控制器插件加载器，必须比所创建插件实例存活得更久
  std::shared_ptr<tf2_ros::Buffer> tf_;  // 保存共享 TF 缓冲区，用于把目标转换到机器人坐标系
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  // 保存局部代价地图 ROS 对象，用于取得机器人基础坐标系名称
  rclcpp::Logger logger_{rclcpp::get_logger("goal_approach_controller")};  // 创建默认日志器，configure 后会替换为父节点日志器
  rclcpp::Clock::SharedPtr clock_;  // 保存 ROS 2 时钟，供节流日志宏使用
  geometry_msgs::msg::PoseStamped goal_;  // 保存当前路径最后一个位姿，也就是本轮控制使用的最终目标
  double approach_distance_{1.5};  // 保存进入普通接近限速模式的距离阈值，默认 1.5 米
  double approach_velocity_{0.5};  // 保存接近目标时允许的最大线速度，默认 0.5 米每秒
  double direct_approach_distance_{0.5};  // 保存进入直接朝目标模式的距离阈值，默认 0.5 米
  double direct_approach_kp_{1.0};  // 保存直达模式的距离比例控制增益，默认 1.0
};  // GoalApproachController 类定义结束

}  // namespace goal_approach_controller  // goal_approach_controller 命名空间结束

PLUGINLIB_EXPORT_CLASS(  // 将本类注册到 pluginlib，使 Nav2 能根据 XML/YAML 类型名动态加载
  goal_approach_controller::GoalApproachController,  // 指定要导出的具体插件类完整名称
  nav2_core::Controller)  // 指定该插件实现的基类接口类型
