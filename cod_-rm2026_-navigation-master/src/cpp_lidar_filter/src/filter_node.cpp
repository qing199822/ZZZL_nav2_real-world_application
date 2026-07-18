#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include "visualization_msgs/msg/marker.hpp"

class LidarFilterNode : public rclcpp::Node
{
public:
  LidarFilterNode() : Node("lidar_filter_node")
  {
    // Declare and cache parameters
    this->declare_parameter("input_topic", "/livox/lidar");
    this->declare_parameter("output_topic", "/livox/lidar_filtered");
    this->declare_parameter("min_x", -0.4);
    this->declare_parameter("max_x", 0.4);
    this->declare_parameter("min_y", -0.3);
    this->declare_parameter("max_y", 0.3);
    this->declare_parameter("min_z", -0.1);
    this->declare_parameter("max_z", 0.6);
    this->declare_parameter("negative", true);
    this->declare_parameter("leaf_size", 0.05);
    this->declare_parameter("enable_voxel_filter", false);

    // Cache parameters at init
    updateCachedParams();

    // Dynamic parameter callback to update cache
    dyn_params_handler_ = this->add_on_set_parameters_callback(
      std::bind(&LidarFilterNode::dynamicParamsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Listening on: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_.c_str());

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarFilterNode::cloud_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("crop_box_marker", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&LidarFilterNode::publish_marker, this));
  }

private:
  void updateCachedParams()
  {
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    min_x_ = this->get_parameter("min_x").as_double();
    max_x_ = this->get_parameter("max_x").as_double();
    min_y_ = this->get_parameter("min_y").as_double();
    max_y_ = this->get_parameter("max_y").as_double();
    min_z_ = this->get_parameter("min_z").as_double();
    max_z_ = this->get_parameter("max_z").as_double();
    negative_ = this->get_parameter("negative").as_bool();
    leaf_size_ = this->get_parameter("leaf_size").as_double();
    enable_voxel_filter_ = this->get_parameter("enable_voxel_filter").as_bool();
  }

  rcl_interfaces::msg::SetParametersResult dynamicParamsCallback(
    std::vector<rclcpp::Parameter> parameters)
  {
    updateCachedParams();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void publish_marker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "vehicle_body";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = (max_x_ + min_x_) / 2.0;
    marker.pose.position.y = (max_y_ + min_y_) / 2.0;
    marker.pose.position.z = (max_z_ + min_z_) / 2.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = max_x_ - min_x_;
    marker.scale.y = max_y_ - min_y_;
    marker.scale.z = max_z_ - min_z_;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.4;

    marker_pub_->publish(marker);
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PCLPointCloud2::Ptr cloud_in(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*msg, *cloud_in);

    // CropBox filter (remove robot body points)
    pcl::CropBox<pcl::PCLPointCloud2> crop;
    crop.setInputCloud(cloud_in);

    Eigen::Vector4f min_pt, max_pt;
    min_pt << min_x_, min_y_, min_z_, 1.0;
    max_pt << max_x_, max_y_, max_z_, 1.0;

    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.setNegative(negative_);

    pcl::PCLPointCloud2::Ptr cloud_cropped(new pcl::PCLPointCloud2);
    crop.filter(*cloud_cropped);

    // Optional voxel grid downsampling
    pcl::PCLPointCloud2::Ptr cloud_filtered = cloud_cropped;
    if (enable_voxel_filter_) {
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud(cloud_cropped);
      sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
      cloud_filtered = std::make_shared<pcl::PCLPointCloud2>();
      sor.filter(*cloud_filtered);
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_filtered, output);
    output.header = msg->header;

    pub_->publish(output);
  }

  // Cached parameters
  std::string input_topic_;
  std::string output_topic_;
  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
  bool negative_;
  double leaf_size_;
  bool enable_voxel_filter_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilterNode>());
  rclcpp::shutdown();
  return 0;
}
