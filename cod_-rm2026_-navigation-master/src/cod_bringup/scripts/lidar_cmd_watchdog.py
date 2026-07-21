#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class LidarCmdWatchdog(Node):
    def __init__(self):
        super().__init__('lidar_cmd_watchdog')

        self.declare_parameter('cloud_topic', '/livox/lidar_filtered')
        self.declare_parameter('cmd_vel_in_topic', 'cmd_vel_collision_safe')
        self.declare_parameter('cmd_vel_out_topic', 'cmd_vel')
        self.declare_parameter('cloud_timeout', 0.3)
        self.declare_parameter('check_rate', 20.0)

        cloud_topic = self.get_parameter('cloud_topic').value
        cmd_vel_in_topic = self.get_parameter('cmd_vel_in_topic').value
        cmd_vel_out_topic = self.get_parameter('cmd_vel_out_topic').value
        self.cloud_timeout = float(self.get_parameter('cloud_timeout').value)
        check_rate = float(self.get_parameter('check_rate').value)

        if self.cloud_timeout <= 0.0:
            raise ValueError('cloud_timeout must be positive')
        if check_rate <= 0.0:
            raise ValueError('check_rate must be positive')

        self.last_cloud_time = None
        self.was_healthy = False
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_out_topic, 10)
        self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_callback, qos_profile_sensor_data)
        self.create_subscription(Twist, cmd_vel_in_topic, self.cmd_callback, 10)
        self.create_timer(1.0 / check_rate, self.watchdog_callback)

        self.get_logger().warn(
            f'LiDAR command gate starts closed; waiting for {cloud_topic}')

    def cloud_callback(self, _msg):
        self.last_cloud_time = self.get_clock().now()

    def lidar_is_healthy(self):
        if self.last_cloud_time is None:
            return False
        age = (self.get_clock().now() - self.last_cloud_time).nanoseconds / 1.0e9
        return age <= self.cloud_timeout

    def cmd_callback(self, msg):
        if self.lidar_is_healthy():
            self.cmd_pub.publish(msg)
        else:
            self.cmd_pub.publish(Twist())

    def watchdog_callback(self):
        healthy = self.lidar_is_healthy()
        if not healthy:
            self.cmd_pub.publish(Twist())
        if healthy != self.was_healthy:
            if healthy:
                self.get_logger().info('LiDAR data is fresh; command gate opened')
            elif self.last_cloud_time is not None:
                self.get_logger().error(
                    'LiDAR data timed out; command gate closed and zero velocity enforced')
            self.was_healthy = healthy


def main(args=None):
    rclpy.init(args=args)
    node = LidarCmdWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
