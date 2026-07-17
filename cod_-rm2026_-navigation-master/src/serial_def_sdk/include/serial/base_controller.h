#ifndef _BASE_CONTROLLER_H__
#define _BASE_CONTROLLER_H__
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <vector>

#include "def_msg/msg/gobal_information.hpp"
#include "def_msg/msg/gimble_control.hpp"
#include "def_msg/msg/common_control.hpp"
#include "def_msg/srv/heart_beat.hpp"
#include "def_msg/msg/gimble_position.hpp"
#include "def_msg/msg/gimble_control.hpp"
//#include "vision_msg/msg/gimble_position.hpp"
#include <std_srvs/srv/set_bool.hpp>  // <--- 新增标准服务头文件
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>  //used for trans tf
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>//#include <tf/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <example_interfaces/msg/float32.hpp> 
#include <atomic>
#include "uart_hd.h"
#include "DataType.h"
#include "uart.h"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"


using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

#define TIMESTAMP_ASSERT_DELAY 13  //时间戳接收最长延迟，超过该延迟判定为断连，单位s*10

class BaseController:public rclcpp::Node{
private:
    /************* claim timer ***********************/
    /*the timer is used by trans data from serial to serial*/
    rclcpp::TimerBase::SharedPtr gimble_timer;
    rclcpp::TimerBase::SharedPtr odom_timer;
    rclcpp::TimerBase::SharedPtr gobal_information_timer;
    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    rclcpp::TimerBase::SharedPtr imu_timer;
    rclcpp::TimerBase::SharedPtr yaw_timer;
    rclcpp::TimerBase::SharedPtr game_status_timer;
    rclcpp::TimerBase::SharedPtr send_timer_;

    rclcpp::TimerBase::SharedPtr fix_send_timer_;
    rclcpp::TimerBase::SharedPtr t1;
    rclcpp::TimerBase::SharedPtr t2;
    rclcpp::TimerBase::SharedPtr t3;
    rclcpp::TimerBase::SharedPtr t4;
    
    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_gimbal_time_;
    rclcpp::Time last_spin_time_;
  // R3修复: 已移除未使用的 DATA_TIMEOUT_MS, 改用可配置参数 topic_timeout_ms_
    // M2修复: 成员变量替代静态变量的声明
    bool has_ever_received_ = false;
    rclcpp::Time init_cmd_time_;
    rclcpp::Time init_gimbal_time_;
    rclcpp::Time init_spin_time_;
    // M5: 话题超时参数(ms), launch 文件可配置
    int topic_timeout_ms_ = 100; 

    /************  claim publisher *******************/
    rclcpp::Publisher<def_msg::msg::GimbleControl>::SharedPtr gimble_pub;  //publish the gimble recv data
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
    rclcpp::Publisher<def_msg::msg::GobalInformation>::SharedPtr gobal_information_pub;  
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> gimble_broadcaster;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu2_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;   //yaw_timer

    rclcpp::Publisher<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
    rclcpp::Publisher<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;

    /************  chaim sub ***********************/
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
    rclcpp::Subscription<def_msg::msg::GimbleControl>::SharedPtr gimble_control_sub;
    rclcpp::Subscription<def_msg::msg::CommonControl>::SharedPtr common_control_sub;
    rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr spin_sub;
    bool override_gimbal_ = false;  // 默认不接管（正常自瞄模式）
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mux_service_;
    void mux_service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    std::atomic<float> current_spin_speed{0.0f};  // M3修复: 改为原子类型
public:
    BaseController(string name):Node(name)
    {
        /*获取参数*/
        this->declare_parameter("serial_port","/dev/ttyUSB0");
        this->get_parameter_or<std::string>("serial_port", uart_port, "/dev/ttyUSB0");
        uart->setUartName(uart_port);
        /***************** sub *****************/
        /*从combine_control获取控制指令*/
        //订阅底盘控制命令
        cmd_sub=
            this->create_subscription<geometry_msgs::msg::Twist>("hardware/cmd_vel_api", 10,
        std::bind(&BaseController::cmd2serial,this,std::placeholders::_1));
        //gimble control cmd
        gimble_control_sub = 
            this->create_subscription<def_msg::msg::GimbleControl>("vision/gimble_control", rclcpp::SensorDataQoS(),
        std::bind(&BaseController::gimble2serial, this, _1));

        spin_sub = 
            this->create_subscription<example_interfaces::msg::Float32>("cmd_spin", 10,
        std::bind(&BaseController::spin2serial, this, _1));

        //common_control cmd
        common_control_sub = 
            this->create_subscription<def_msg::msg::CommonControl>("hardware/common_control", 10,
        std::bind(&BaseController::control2serial, this, _1));
        
        /************* pub **********************/
        //publish the tf,control cmd，从串口数据到来
        odom_pub=
			this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        gobal_information_pub=
            this->create_publisher<def_msg::msg::GobalInformation>("hardware/gobal_information",10);

            game_status_pub_ = 
            this->create_publisher<pb_rm_interfaces::msg::GameStatus>("/robot/referee/game_status", 10);
        robot_status_pub_ = 
            this->create_publisher<pb_rm_interfaces::msg::RobotStatus>("/robot/referee/robot_status", 10);

        gimble_pub=
            this->create_publisher<def_msg::msg::GimbleControl>("hardware/gimble_current_rpy",10);
        odom_broadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        gimble_broadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        imu2_pub = 
            this->create_publisher<sensor_msgs::msg::Imu>("imu/data_2",10);
        joint_pub = 
            this->create_publisher<sensor_msgs::msg::JointState>("joint_state",10);

        /*================================ time callback =================================*/

        ////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////          FROM SERIAL            /////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////


        //100HZ gimble
        auto gimble_time_callback = [this]() -> void {
			this->serial2gimble();
		};
		gimble_timer = this->create_wall_timer(26ms, gimble_time_callback//,flight_timer_group
		);   
        //100HZ odom
        // auto odom_time_callback = [this]() -> void {
        //     this->speed2odom();
        // };
        // odom_timer = this->create_wall_timer(25ms,odom_time_callback);

        //5HZ gobal
        auto gobal_information_callback = [this]() -> void {
			this->serial2global();
		};
		gobal_information_timer = this->create_wall_timer(200ms, gobal_information_callback
		);   
        
         //joint 100Hz  yaw joint
        auto joint_callabck = [this]()->void{
            serial2joint();
        };
        yaw_timer = this->create_wall_timer(10ms,joint_callabck);
         // //100-200HZ  imu
        // auto imu_callback = [this]() -> void{
        //     serial2imu();
        // };
        // imu_timer = this->create_wall_timer(14ms,imu_callback);


        ////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////          TO SERIAL            ///////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////
        //2HZ heart
        auto heartbeat_callback = [this]() -> void{
            heartbeat2serial();
        };  
        heartbeat_timer = this->create_wall_timer(200ms,heartbeat_callback);
        
        // auto fix_send_callback = [this]() -> void {
        //     this->send_merged_control();
        // };
        // // 注意：需要在 .h 文件中声明 rclcpp::TimerBase::SharedPtr fix_send_timer_;
        // fix_send_timer_ = this->create_wall_timer(100ms, fix_send_callback);
        // [新增] 初始化时间戳
        mux_service_ = this->create_service<std_srvs::srv::SetBool>(
            "gimbal_override",
            std::bind(&BaseController::mux_service_callback, this, _1, _2));

        // 2. 恢复固定频率发送（推荐 50Hz 即 20ms，或者你原本写的 100ms）
        auto fix_send_callback = [this]() -> void {
            this->send_merged_control();
        };
        // 开启定时器，替代话题回调里的事件驱动，保证接管时也能持续发0
        fix_send_timer_ = this->create_wall_timer(20ms, fix_send_callback);
        // M5修复: 话题超时参数化 (默认 100ms, 可 launch 文件配置)
        this->declare_parameter("topic_timeout_ms", 100);
        this->get_parameter_or<int>("topic_timeout_ms", topic_timeout_ms_, 100);
        RCLCPP_INFO(this->get_logger(), "话题超时配置: %d ms", topic_timeout_ms_);

        // M2修复: 先初始化 last_*_time_, 再复制到 init_*_time_
        last_cmd_time_ = this->get_clock()->now();
        last_gimbal_time_ = this->get_clock()->now();
        last_spin_time_ = this->get_clock()->now();

        has_ever_received_ = false;
        init_cmd_time_ = last_cmd_time_;
        init_gimbal_time_ = last_gimbal_time_;
        init_spin_time_ = last_spin_time_;

        // C4修复: 不再手动启动线程, SerialDriver::readLoop 已处理
        // (C4修复: 移除重复线程 — SerialDriver::readLoop 已处理接收)
        //
        // auto t1_tim =  [this]() -> void{
        //     control2serial_t();
        // };  
        //  t1 = this->create_wall_timer(12ms,t1_tim);
        // //
        //  auto t2_tim =  [this]() -> void{
        //     gimble2serial_t();
        // };  
        // t2 = this->create_wall_timer(5ms,t2_tim);

        // //
        //  auto t3_tim =  [this]() -> void{
            
        // };  
        //  t3 = this->create_wall_timer(500ms,t3_tim);
        // //
        //  auto t4_tim =  [this]() -> void{
        //     control2serial
        // };  
        // t4 = this->create_wall_timer(500ms,t4_tim);
        //thread
        // (C4修复: 移除重复线程 — SerialDriver::readLoop 已处理接收)
        RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());

    }
    ~BaseController(){
        flag = false;
    }

    inline void cmd2serial(const geometry_msgs::msg::Twist::UniquePtr twist_aux); //cmd控制命令转串口(ros->serial)
    void gimble2serial(const def_msg::msg::GimbleControl::UniquePtr msg); // 云台控制(ros->serial)
    inline void control2serial(const def_msg::msg::CommonControl::UniquePtr msg);  //机器人电控状态控制(ros->serial)
    void spin2serial(const example_interfaces::msg::Float32::UniquePtr msg);
    void heartbeat2serial(); //心跳包
    // inline void control2serial_t();
    // inline void gimble2serial_t();
    void serial2imu();  //串口获取imu数据
    void speed2odom(); //速度转odom(serial->ros)
    inline void serial2gimble(); //云台回传并发布tf(serial->ros)
    void serial2global();   //更新机器人状态(serial->ros)
    //void shoot_control(const def_msg::msg::Shoot::UniquePtr msg); //机器人射击控制(ros->serial)
    void serial2heartbeat(const def_msg::srv::HeartBeat::Request::SharedPtr request,
        const def_msg::srv::HeartBeat::Response::SharedPtr response);
    void serial2joint();
    void send_merged_control(); 

private:
    string uart_port;
    bool flag = true;

    struct{//回传的底盘速度
        float vx = 0;  //速度x
        float vy = 0;
        float vz = 0;  //角速度z
        double time_stamp;
        rclcpp::Time current_time;
        rclcpp::Time last_time;
    }speed;    //received

    struct{
        double x = 0.0;                   // 初始位置x的坐标
        double y = 0.0;                   // 初始位置y的坐标
        double z = 0.0;                   // 初始位置的角度
        double dt = 0.0;                  // 积分时间
    }odom_raw;  //定义的odom

    // struct{
    //     //received heart beat
    //     clock_t time;
    //     uint8_t battery = 100;
    //     uint8_t life_extra = 100;
    //     uint8_t color = 0;    //  0 undefine   1 blue   2 紫色 3 red
    //     uint8_t bullet_extra = 255; //仅能以unsigned 0-700映射到0-255;
    //     uint8_t fault_flag = 0B11111111;

    //     //additional data
    //     uint8_t launch,        //回传底盘云台是否已经启用
    //             arm,           //回传摩擦轮是否已经启动
    //             base_hp_our,   //我方基地站血量,百分制
    //             base_hp_enemy, //对方基地在血量
    //             judge_warning_level;  //裁判系统警
        
    //     //game data
    //     uint8_t game_progress = 0;
    //     uint16_t stage_remain_time = 0;
    // }global_information;  //从电控传回的全局数据信息

    struct{
        float yaw = 0;
        float pitch = 0;
    }gimble_current,gimble_last,gimble_target; //有绝对和增量形式两种

};
#endif
