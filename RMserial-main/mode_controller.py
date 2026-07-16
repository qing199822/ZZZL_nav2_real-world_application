#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from example_interfaces.msg import Float32

# TF2 相关的库，用于获取实时位姿
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

# 请导入你实际的工作空间中的自定义消息和服务类型
from def_msg.msg import GobalInformation 
from std_srvs.srv import SetBool 

import threading
import time
import json
import os

class GameLogicNode(Node):
    def __init__(self):
        super().__init__('game_logic_node')
        
        # ================= 1. 发布者 =================
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.spin_pub = self.create_publisher(Float32, '/cmd_spin', 10)
        
        # ================= 2. 订阅者 =================
        self.global_info_sub = self.create_subscription(
            GobalInformation, 
            '/gobal_information', 
            self.global_info_callback, 
            10
        )
        
        # ================= 3. 服务与TF客户端 =================
        self.control_client = self.create_client(SetBool, '/request_chassis_control')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ================= 4. 状态变量 =================
        self.fault_flag = 1      
        self.life_extra = 1000   
        self.health_threshold = 200 
        
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_spin = 0.0
        
        self.history_stack = []

        # ================= 5. 读取 JSON 配置文件 =================
        # 注意：这里默认读取当前目录下的 json 文件。
        # 如果你使用 ROS2 launch，建议改为绝对路径，或使用 ament_index_python 获取包路径
        self.config_path = "occupy_stages.json" 
        self.occupy_stages = self.load_stages_config()
        
        # ================= 6. 定时器 =================
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.log_timer = self.create_timer(2.0, self.log_position_callback)
        
        # ================= 7. 开启逻辑控制子线程 =================
        self.logic_thread = threading.Thread(target=self.main_logic_loop)
        self.logic_thread.start()

    def load_stages_config(self):
        """读取占点模式阶段配置的 JSON 文件"""
        if not os.path.exists(self.config_path):
            self.get_logger().error(f"配置文件 {self.config_path} 不存在! 将使用默认的硬编码阶段。")
            return [
                {"vx": 0.0, "vy": 0.5, "monitor_axis": "y", "target_delta": 3.0},
                {"vx": -0.5, "vy": 0.0, "monitor_axis": "x", "target_delta": -3.5},
                {"vx": 0.0, "vy": -0.5, "monitor_axis": "y", "target_delta": -3.0}
            ]
        
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                self.get_logger().info(f"成功加载配置文件: {self.config_path}")
                return data.get("stages", [])
        except Exception as e:
            self.get_logger().error(f"读取配置文件失败: {e}，将返回空列表。")
            return []

    def log_position_callback(self):
        x, y = self.get_odom_position()
        if x is not None and y is not None:
            self.get_logger().info(f"[实时坐标] odom -> base_link: x={x:.3f}, y={y:.3f}")
        else:
            self.get_logger().warn("[实时坐标] 正在等待 TF 坐标转换 (odom -> base_link)...")

    def global_info_callback(self, msg):
        self.fault_flag = 1
        self.life_extra = 1000

    def timer_callback(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.target_vx
        twist_msg.linear.y = self.target_vy
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        
        spin_msg = Float32()
        spin_msg.data = self.target_spin
        self.spin_pub.publish(spin_msg)

    def request_control_service(self, enable: bool):
        self.get_logger().info(f"正在调用服务，试图 {'获取' if enable else '归还'} 控制权...")
        if not self.control_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("服务不可用！跳过服务请求。")
            return
        
        req = SetBool.Request()
        req.data = enable
        try:
            self.control_client.call_async(req) 
            self.get_logger().info("服务调用已发出。")
        except Exception as e:
            self.get_logger().error(f"服务调用失败: {e}")

    def get_odom_position(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            return trans.transform.translation.x, trans.transform.translation.y
        except TransformException:
            return None, None

    def move_until(self, vx, vy, monitor_axis, target_delta, check_health=True):
        start_x, start_y = None, None
        while start_x is None and rclpy.ok():
            if self.fault_flag != 1: return "STOPPED", 0.0
            start_x, start_y = self.get_odom_position()
            time.sleep(0.05)
            
        start_val = start_x if monitor_axis == 'x' else start_y
        self.get_logger().info(f"开始位移: 轴 {monitor_axis}, 目标差值 {target_delta}m")

        while rclpy.ok():
            if self.fault_flag != 1:
                self.target_vx, self.target_vy = 0.0, 0.0
                return "STOPPED", 0.0

            curr_x, curr_y = self.get_odom_position()
            if curr_x is None:
                time.sleep(0.05)
                continue
            
            curr_val = curr_x if monitor_axis == 'x' else curr_y
            actual_delta = curr_val - start_val

            if check_health and self.life_extra < self.health_threshold:
                self.target_vx, self.target_vy = 0.0, 0.0
                return "INTERRUPTED", actual_delta

            reached = False
            if target_delta > 0:
                if actual_delta >= target_delta - 0.25: reached = True
            else:
                if actual_delta <= target_delta + 0.25: reached = True

            if reached:
                self.target_vx, self.target_vy = 0.0, 0.0
                return "REACHED", actual_delta

            self.target_vx = float(vx)
            self.target_vy = float(vy)
            time.sleep(0.05)

    def execute_retreat(self, current_vx=None, current_vy=None, current_axis=None, current_dist=None):
        self.get_logger().info(f">>> 触发撤退模式！")
        self.request_control_service(True)
        
        if current_vx is not None and current_dist is not None:
            self.move_until(vx=-current_vx, vy=-current_vy, 
                            monitor_axis=current_axis, target_delta=-current_dist, check_health=False)

        while len(self.history_stack) > 0:
            if self.fault_flag != 1: break
            step = self.history_stack.pop()
            self.move_until(vx=-step['vx'], vy=-step['vy'], 
                            monitor_axis=step['axis'], target_delta=-step['dist'], check_health=False)

        self.target_vx, self.target_vy = 0.0, 0.0
        self.get_logger().info("撤退完成。")
        
    def main_logic_loop(self):
        self.get_logger().info("比赛逻辑控制节点已启动，坐标报告已开启（2s/次）")
        
        while rclpy.ok():
            if self.fault_flag != 1:
                time.sleep(0.5)
                continue

            # 占点逻辑
            self.get_logger().info(">>> 触发: 占点模式")
            self.request_control_service(True)
            self.history_stack.clear()
            
            occupy_success = True
            is_stopped = False
            
            # --- 动态读取配置文件中的阶段 ---
            for i, stage in enumerate(self.occupy_stages):
                if not occupy_success or self.fault_flag != 1:
                    break

                vx = stage.get('vx', 0.0)
                vy = stage.get('vy', 0.0)
                axis = stage.get('monitor_axis', 'x')
                target_delta = stage.get('target_delta', 0.0)

                self.get_logger().info(f"--- 正在执行第 {i+1} 阶段 ---")
                st, dist = self.move_until(vx=vx, vy=vy, monitor_axis=axis, target_delta=target_delta)
                
                if st == "STOPPED":
                    is_stopped = True
                    break  # 终止当前占点，退到外层 while 循环
                
                if st == "INTERRUPTED":
                    self.execute_retreat(vx, vy, axis, dist)
                    occupy_success = False
                    break
                
                elif st == "REACHED":
                    self.history_stack.append({'vx': vx, 'vy': vy, 'axis': axis, 'dist': dist})

            # 如果中途比赛停止，跳过后面的逻辑直接重开大循环
            if is_stopped:
                continue

            self.request_control_service(False)
            
            if occupy_success:
                self.get_logger().info(">>> 占点成功: 等待血量降低...")
                while rclpy.ok() and self.life_extra >= self.health_threshold and self.fault_flag == 1:
                    time.sleep(0.1)
                self.execute_retreat()

            self.get_logger().info(">>> 循环结束，静止 15 秒...")
            for _ in range(150):
                if self.fault_flag != 1: break
                time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = GameLogicNode()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
