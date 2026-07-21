#include <memory>  // 提供 std::shared_ptr、std::make_shared 等智能指针工具
#include "rclcpp/rclcpp.hpp"  // 引入 ROS 2 C++ 客户端库的节点、发布订阅和日志接口
#include <chrono>  // 提供定时器使用的时间单位和时间长度类型
#include "sensor_msgs/msg/point_cloud2.hpp"  // 引入 ROS 2 标准 PointCloud2 点云消息类型
#include <pcl_conversions/pcl_conversions.h>  // 提供 ROS PointCloud2 与 PCL 点云格式之间的转换函数
#include <pcl/filters/crop_box.h>  // 引入 PCL CropBox 裁剪框滤波器
#include <pcl/filters/voxel_grid.h>  // 引入 PCL VoxelGrid 体素降采样滤波器
#include "visualization_msgs/msg/marker.hpp"  // 引入 RViz 可视化 Marker 消息类型

class LidarFilterNode : public rclcpp::Node  // 定义雷达点云滤波节点类，并继承 ROS 2 基础节点类
{  // LidarFilterNode 类定义开始
public:  // 以下构造函数对类外公开
  LidarFilterNode() : Node("lidar_filter_node")  // 构造节点并把 ROS 2 节点名称设置为 lidar_filter_node
  {  // 构造函数函数体开始
    // Declare and cache parameters  // 声明本节点使用的 ROS 2 参数，随后会读取并缓存参数值
    this->declare_parameter("input_topic", "/livox/lidar");  // 声明输入点云话题参数，默认订阅 Livox 原始点云
    this->declare_parameter("output_topic", "/livox/lidar_filtered");  // 声明输出话题参数，默认发布过滤后的点云
    this->declare_parameter("min_x", -0.4);  // 声明裁剪框 x 轴最小边界，单位为米
    this->declare_parameter("max_x", 0.4);  // 声明裁剪框 x 轴最大边界，单位为米
    this->declare_parameter("min_y", -0.3);  // 声明裁剪框 y 轴最小边界，单位为米
    this->declare_parameter("max_y", 0.3);  // 声明裁剪框 y 轴最大边界，单位为米
    this->declare_parameter("min_z", -0.1);  // 声明裁剪框 z 轴最小边界，单位为米
    this->declare_parameter("max_z", 0.6);  // 声明裁剪框 z 轴最大边界，单位为米
    this->declare_parameter("negative", true);  // 声明是否反选裁剪框，true 表示保留框外点并删除框内车身点
    this->declare_parameter("leaf_size", 0.05);  // 声明体素边长参数，默认每个体素边长为 0.05 米
    this->declare_parameter("enable_voxel_filter", false);  // 声明体素降采样开关，默认关闭

    // Cache parameters at init  // 节点初始化时读取一次所有参数并保存到成员变量
    updateCachedParams();  // 调用参数读取函数更新本地参数缓存

    // Dynamic parameter callback to update cache  // 注册运行时参数修改回调，用于响应 ros2 param set
    dyn_params_handler_ = this->add_on_set_parameters_callback(  // 保存回调句柄，保证动态参数回调在节点运行期间有效
      std::bind(&LidarFilterNode::dynamicParamsCallback, this, std::placeholders::_1));  // 将参数事件绑定到当前对象的处理函数

    RCLCPP_INFO(this->get_logger(), "Listening on: %s", input_topic_.c_str());  // 在日志中输出当前订阅的点云话题
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_.c_str());  // 在日志中输出过滤点云的发布话题

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(  // 创建 PointCloud2 点云订阅器
      input_topic_, rclcpp::SensorDataQoS(),  // 使用传感器数据 QoS 订阅配置的话题，以适应高频且允许少量丢帧的点云
      std::bind(&LidarFilterNode::cloud_callback, this, std::placeholders::_1));  // 每收到一帧点云就调用 cloud_callback

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);  // 创建过滤点云发布器，消息队列深度为 10
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("crop_box_marker", 10);  // 创建裁剪框可视化标记发布器

    timer_ = this->create_wall_timer(  // 创建使用系统时间的周期定时器
      std::chrono::milliseconds(1000),  // 将定时器周期设置为 1000 毫秒，即每秒触发一次
      std::bind(&LidarFilterNode::publish_marker, this));  // 定时调用 publish_marker 发布裁剪框
  }  // 构造函数结束

private:  // 以下函数和成员变量只允许类内部访问
  void updateCachedParams()  // 定义从 ROS 2 参数服务器读取参数并缓存到成员变量的函数
  {  // updateCachedParams 函数体开始
    input_topic_ = this->get_parameter("input_topic").as_string();  // 读取输入点云话题名称
    output_topic_ = this->get_parameter("output_topic").as_string();  // 读取输出点云话题名称
    min_x_ = this->get_parameter("min_x").as_double();  // 读取裁剪框 x 轴最小边界
    max_x_ = this->get_parameter("max_x").as_double();  // 读取裁剪框 x 轴最大边界
    min_y_ = this->get_parameter("min_y").as_double();  // 读取裁剪框 y 轴最小边界
    max_y_ = this->get_parameter("max_y").as_double();  // 读取裁剪框 y 轴最大边界
    min_z_ = this->get_parameter("min_z").as_double();  // 读取裁剪框 z 轴最小边界
    max_z_ = this->get_parameter("max_z").as_double();  // 读取裁剪框 z 轴最大边界
    negative_ = this->get_parameter("negative").as_bool();  // 读取裁剪框保留内部还是外部点的开关
    leaf_size_ = this->get_parameter("leaf_size").as_double();  // 读取体素降采样的体素边长
    enable_voxel_filter_ = this->get_parameter("enable_voxel_filter").as_bool();  // 读取体素降采样启用状态
  }  // updateCachedParams 函数结束

  rcl_interfaces::msg::SetParametersResult dynamicParamsCallback(  // 定义动态参数修改请求的处理函数，并返回是否接受修改
    std::vector<rclcpp::Parameter> parameters)  // 接收本次准备修改的参数列表，当前代码未直接遍历该列表
  {  // dynamicParamsCallback 函数体开始
    updateCachedParams();  // 重新读取参数并尝试更新本地缓存
    rcl_interfaces::msg::SetParametersResult result;  // 创建参数修改结果消息
    result.successful = true;  // 将本次参数修改标记为允许执行
    return result;  // 把参数修改结果返回给 ROS 2 参数系统
  }  // dynamicParamsCallback 函数结束

  void publish_marker()  // 定义向 RViz 发布裁剪框可视化标记的函数
  {  // publish_marker 函数体开始
    visualization_msgs::msg::Marker marker;  // 创建一个 Marker 消息对象
    marker.header.frame_id = "base_link";  // 指定裁剪框以机器人 base_link 坐标系为参考
    marker.header.stamp = this->now();  // 使用节点当前时间作为标记时间戳
    marker.ns = "vehicle_body";  // 设置 Marker 命名空间，便于 RViz 分类和更新
    marker.id = 0;  // 设置命名空间内的标记编号为 0
    marker.type = visualization_msgs::msg::Marker::CUBE;  // 将标记形状设置为立方体，用来表示裁剪框
    marker.action = visualization_msgs::msg::Marker::ADD;  // 指示 RViz 添加新标记或更新同编号标记

    marker.pose.position.x = (max_x_ + min_x_) / 2.0;  // 根据 x 轴上下界计算裁剪框中心的 x 坐标
    marker.pose.position.y = (max_y_ + min_y_) / 2.0;  // 根据 y 轴上下界计算裁剪框中心的 y 坐标
    marker.pose.position.z = (max_z_ + min_z_) / 2.0;  // 根据 z 轴上下界计算裁剪框中心的 z 坐标
    marker.pose.orientation.w = 1.0;  // 设置单位四元数，表示裁剪框不发生旋转

    marker.scale.x = max_x_ - min_x_;  // 根据 x 轴边界差计算裁剪框长度
    marker.scale.y = max_y_ - min_y_;  // 根据 y 轴边界差计算裁剪框宽度
    marker.scale.z = max_z_ - min_z_;  // 根据 z 轴边界差计算裁剪框高度

    marker.color.r = 1.0;  // 将标记颜色的红色通道设置为最大值
    marker.color.g = 0.0;  // 将标记颜色的绿色通道设置为零
    marker.color.b = 0.0;  // 将标记颜色的蓝色通道设置为零
    marker.color.a = 0.4;  // 设置标记透明度为 0.4，使点云仍可透过裁剪框观察

    marker_pub_->publish(marker);  // 发布裁剪框 Marker 消息供 RViz 显示
  }  // publish_marker 函数结束

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)  // 定义点云订阅回调，并接收一帧 ROS PointCloud2 消息
  {  // cloud_callback 函数体开始
    pcl::PCLPointCloud2::Ptr cloud_in(new pcl::PCLPointCloud2);  // 创建用于保存输入数据的 PCL 点云智能指针
    pcl_conversions::toPCL(*msg, *cloud_in);  // 将 ROS PointCloud2 消息转换成 PCLPointCloud2 格式

    // CropBox filter (remove robot body points)  // 使用裁剪框滤波器剔除机器人车身范围内的自反射点
    pcl::CropBox<pcl::PCLPointCloud2> crop;  // 创建处理 PCLPointCloud2 数据的 CropBox 滤波器
    crop.setInputCloud(cloud_in);  // 把本次收到的原始点云设置为裁剪框输入

    Eigen::Vector4f min_pt, max_pt;  // 创建裁剪框最小点和最大点的四维齐次坐标向量
    min_pt << min_x_, min_y_, min_z_, 1.0;  // 使用缓存参数组成裁剪框的最小边界点
    max_pt << max_x_, max_y_, max_z_, 1.0;  // 使用缓存参数组成裁剪框的最大边界点

    crop.setMin(min_pt);  // 把最小边界点传给 CropBox 滤波器
    crop.setMax(max_pt);  // 把最大边界点传给 CropBox 滤波器
    crop.setNegative(negative_);  // 设置是否反选；true 删除框内点，false 保留框内点

    pcl::PCLPointCloud2::Ptr cloud_cropped(new pcl::PCLPointCloud2);  // 创建用于保存裁剪结果的 PCL 点云
    crop.filter(*cloud_cropped);  // 执行 CropBox 过滤并把结果写入 cloud_cropped

    // Optional voxel grid downsampling  // 根据参数决定是否对裁剪后的点云执行体素降采样
    pcl::PCLPointCloud2::Ptr cloud_filtered = cloud_cropped;  // 默认直接把裁剪结果作为最终过滤结果
    if (enable_voxel_filter_) {  // 仅在体素滤波开关为 true 时执行以下降采样操作
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  // 创建处理 PCLPointCloud2 的体素网格滤波器
      sor.setInputCloud(cloud_cropped);  // 将裁剪后的点云设置为体素滤波器输入
      sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);  // 将 x、y、z 三个方向的体素边长设置为 leaf_size_
      cloud_filtered = std::make_shared<pcl::PCLPointCloud2>();  // 为体素降采样结果分配新的点云对象
      sor.filter(*cloud_filtered);  // 执行体素降采样并保存最终点云
    }  // 可选体素降采样条件结束

    sensor_msgs::msg::PointCloud2 output;  // 创建准备发布的 ROS PointCloud2 输出消息
    pcl_conversions::fromPCL(*cloud_filtered, output);  // 将最终 PCL 点云转换回 ROS PointCloud2 格式
    output.header = msg->header;  // 复制原始消息的时间戳和坐标系，保持数据时空信息一致

    pub_->publish(output);  // 将处理完成的点云发布到输出话题
  }  // cloud_callback 函数结束

  // Cached parameters  // 以下成员变量缓存参数值，避免每处理一帧点云都查询参数服务器
  std::string input_topic_;  // 保存输入点云话题名称
  std::string output_topic_;  // 保存输出点云话题名称
  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;  // 保存裁剪框三个坐标轴的最小和最大边界
  bool negative_;  // 保存 CropBox 是否反选的配置
  double leaf_size_;  // 保存体素滤波器的体素边长
  bool enable_voxel_filter_;  // 保存是否启用体素降采样的配置

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;  // 保存输入点云订阅器对象
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;  // 保存过滤点云发布器对象
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;  // 保存裁剪框 Marker 发布器对象
  rclcpp::TimerBase::SharedPtr timer_;  // 保存周期发布裁剪框的定时器对象
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;  // 保存动态参数回调句柄
};  // LidarFilterNode 类定义结束

int main(int argc, char ** argv)  // 程序入口，接收命令行参数并返回进程退出状态
{  // main 函数体开始
  rclcpp::init(argc, argv);  // 初始化 ROS 2 通信环境并解析 ROS 命令行参数
  rclcpp::spin(std::make_shared<LidarFilterNode>());  // 创建滤波节点并持续处理订阅、定时器和参数回调
  rclcpp::shutdown();  // 节点退出后关闭 ROS 2 通信环境并释放相关资源
  return 0;  // 返回 0 表示程序正常结束
}  // main 函数结束
