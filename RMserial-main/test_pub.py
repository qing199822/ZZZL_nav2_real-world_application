#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from example_interfaces.msg import Float32
import math
import time
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext

# 导入自定义消息
try:
    from def_msg.msg import GimbleControl
except ImportError:
    print("错误：无法导入 def_msg。请确保 source install/setup.bash")
    exit(1)

class MultiThreadPublisher(Node):
    def __init__(self):
        super().__init__('gui_control_publisher')

        # === 资源句柄 (初始为 None) ===
        self.nav_pub = None
        self.nav_timer = None
        self.gimbal_pub = None
        self.gimbal_timer = None
        self.spin_pub = None
        self.spin_timer = None

        # === 记忆频率 (默认 10Hz) ===
        self.nav_freq = 10
        self.gimbal_freq = 10
        self.spin_freq = 10

        # === 日志回调 ===
        self.log_callback = None

        # === 初始化：默认全部开启 ===
        # 这里调用 toggle 函数来创建资源
        self.toggle_resource('nav', True)
        self.toggle_resource('gimbal', True)
        self.toggle_resource('spin', True)
        
        self.get_logger().info("ROS 2 节点已初始化 (支持话题动态销毁)")

    def log(self, msg):
        if self.log_callback:
            self.log_callback(msg)

    def toggle_resource(self, key, enable):
        """核心函数：根据 enable 状态创建或销毁 Publisher 和 Timer"""
        if enable:
            self.create_resources(key)
        else:
            self.destroy_resources(key)

    def create_resources(self, key):
        """创建 Publisher 和 Timer"""
        if key == 'nav':
            if self.nav_pub is None:
                self.nav_pub = self.create_publisher(Twist, 'cmd_vel', 10)
                self.update_timer('nav', self.nav_freq) # 使用记忆的频率启动定时器
                self.log(">>> [系统] 导航话题已创建")

        elif key == 'gimbal':
            if self.gimbal_pub is None:
                self.gimbal_pub = self.create_publisher(GimbleControl, 'vision/gimble_control', 10)
                self.update_timer('gimbal', self.gimbal_freq)
                self.log(">>> [系统] 云台话题已创建")

        elif key == 'spin':
            if self.spin_pub is None:
                self.spin_pub = self.create_publisher(Float32, 'cmd_spin', 10)
                self.update_timer('spin', self.spin_freq)
                self.log(">>> [系统] 小陀螺话题已创建")

    def destroy_resources(self, key):
        """销毁 Publisher 和 Timer"""
        if key == 'nav':
            if self.nav_timer:
                self.destroy_timer(self.nav_timer)
                self.nav_timer = None
            if self.nav_pub:
                self.destroy_publisher(self.nav_pub)
                self.nav_pub = None
            self.log("<<< [系统] 导航话题已销毁")

        elif key == 'gimbal':
            if self.gimbal_timer:
                self.destroy_timer(self.gimbal_timer)
                self.gimbal_timer = None
            if self.gimbal_pub:
                self.destroy_publisher(self.gimbal_pub)
                self.gimbal_pub = None
            self.log("<<< [系统] 云台话题已销毁")

        elif key == 'spin':
            if self.spin_timer:
                self.destroy_timer(self.spin_timer)
                self.spin_timer = None
            if self.spin_pub:
                self.destroy_publisher(self.spin_pub)
                self.spin_pub = None
            self.log("<<< [系统] 小陀螺话题已销毁")

    def update_freq_val(self, key, freq):
        """只更新频率数值，如果当前是开启状态，则重启定时器"""
        # 1. 更新记忆变量
        if key == 'nav': self.nav_freq = freq
        elif key == 'gimbal': self.gimbal_freq = freq
        elif key == 'spin': self.spin_freq = freq

        # 2. 如果资源存在，立即应用新频率
        self.update_timer(key, freq)

    def update_timer(self, key, freq):
        """内部函数：更新定时器频率"""
        period = 1.0 / max(0.1, float(freq))
        
        # 只有当资源(Publisher/Timer)已经存在时，才去更新 Timer
        # 如果资源被销毁了，这里什么都不做，只等下次 toggle_resource(True) 时使用新频率
        if key == 'nav' and self.nav_pub:
            if self.nav_timer: self.destroy_timer(self.nav_timer)
            self.nav_timer = self.create_timer(period, self.nav_callback)
            
        elif key == 'gimbal' and self.gimbal_pub:
            if self.gimbal_timer: self.destroy_timer(self.gimbal_timer)
            self.gimbal_timer = self.create_timer(period, self.gimbal_callback)
            
        elif key == 'spin' and self.spin_pub:
            if self.spin_timer: self.destroy_timer(self.spin_timer)
            self.spin_timer = self.create_timer(period, self.spin_callback)

    # ================= 回调函数 =================
    def nav_callback(self):
        if self.nav_pub: # 双重检查
            msg = Twist()
            msg.linear.x = 0.5
            msg.angular.z = 0.5 
            self.nav_pub.publish(msg)
            self.log(f"[导航] -> vx: {msg.linear.x:.2f}")

    def gimbal_callback(self):
        if self.gimbal_pub:
            msg = GimbleControl()
            t = time.time()
            msg.yaw = math.sin(t * 2.0) * 30.0   
            msg.pitch = math.cos(t * 2.0) * 10.0 
            msg.fire_advise = True if int(t) % 2 == 0 else False
            self.gimbal_pub.publish(msg)
            self.log(f"[云台] -> Y:{msg.yaw:.1f} P:{msg.pitch:.1f}")

    def spin_callback(self):
        if self.spin_pub:
            msg = Float32()
            t = time.time()
            if (t % 4.0) > 2.0:
                msg.data = 3.14
                status = "开启"
            else:
                msg.data = 0.0
                status = "关闭"
            self.spin_pub.publish(msg)
            self.log(f"[旋转] -> {status} ({msg.data:.2f})")


class ControlApp:
    def __init__(self, root, ros_node):
        self.root = root
        self.node = ros_node
        self.root.title("ROS 2 动态话题控制面板")
        self.root.geometry("520x650")

        self.node.log_callback = self.append_log

        # === 1. 导航控制区 ===
        self.create_control_group("底盘导航", "nav", 
                                  default_freq=10, topic_name="hardware/cmd_vel_api")

        # === 2. 云台控制区 ===
        self.create_control_group("云台控制", "gimbal", 
                                  default_freq=10, topic_name="vision/gimble_control")

        # === 3. 旋转控制区 ===
        self.create_control_group("小陀螺", "spin", 
                                  default_freq=10, topic_name="cmd_spin")

        # === 4. 日志显示区 ===
        log_frame = ttk.LabelFrame(root, text="实时日志", padding=10)
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, state='disabled')
        self.log_text.pack(fill="both", expand=True)
        
        ttk.Button(log_frame, text="清除日志", command=self.clear_log).pack(pady=5)

    def create_control_group(self, title, key, default_freq, topic_name):
        frame = ttk.LabelFrame(self.root, text=f"{title} ({topic_name})", padding=10)
        frame.pack(fill="x", padx=10, pady=5)

        # 开关
        var_enable = tk.BooleanVar(value=True)
        chk = ttk.Checkbutton(frame, text="启用话题 (取消则销毁话题)", variable=var_enable, 
                              command=lambda: self.toggle_resource_gui(key, var_enable.get()))
        chk.pack(anchor="w")

        # 频率滑块
        freq_frame = ttk.Frame(frame)
        freq_frame.pack(fill="x", pady=5)
        
        ttk.Label(freq_frame, text="频率 (Hz):").pack(side="left")
        lbl_freq_val = ttk.Label(freq_frame, text=f"{default_freq} Hz")
        lbl_freq_val.pack(side="right")

        scale = ttk.Scale(freq_frame, from_=1, to=100, orient="horizontal", value=default_freq)
        scale.pack(side="left", fill="x", expand=True, padx=10)
        
        scale.bind("<ButtonRelease-1>", 
                   lambda event: self.change_freq(key, scale.get(), lbl_freq_val))

    def toggle_resource_gui(self, key, is_enabled):
        # 调用 Node 的逻辑进行创建/销毁
        self.node.toggle_resource(key, is_enabled)

    def change_freq(self, key, val, label_widget):
        freq = int(val)
        label_widget.config(text=f"{freq} Hz")
        # 更新 Node 内部频率
        self.node.update_freq_val(key, freq)
        self.append_log(f"--- {key} 频率设定为: {freq} Hz ---")

    def append_log(self, msg):
        self.root.after(0, lambda: self._update_text(msg))

    def _update_text(self, msg):
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state='disabled')

    def clear_log(self):
        self.log_text.config(state='normal')
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state='disabled')

def ros_thread_entry(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    ros_node = MultiThreadPublisher()

    t = threading.Thread(target=ros_thread_entry, args=(ros_node,), daemon=True)
    t.start()

    root = tk.Tk()
    app = ControlApp(root, ros_node)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
