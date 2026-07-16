#include "base_controller.h"
#include <thread>
void BaseController::spin2serial(const example_interfaces::msg::Float32::UniquePtr msg) {
  // 1. 更新数据
  current_spin_speed = msg->data;
  
  // 2. [新增] 更新最后接收时间
  last_spin_time_ = this->get_clock()->now();
  // this->send_merged_control();
}

void BaseController::cmd2serial(const geometry_msgs::msg::Twist::UniquePtr twist_aux) {
  // 1. 更新数据
  fix_control_send.vx = twist_aux->linear.x;
  fix_control_send.vy = twist_aux->linear.y;
  fix_control_send.vz = twist_aux->angular.z;
  
  // 2. [新增] 更新最后接收时间
  last_cmd_time_ = this->get_clock()->now();
  // this->send_merged_control();

}


void BaseController::gimble2serial(const def_msg::msg::GimbleControl::UniquePtr msg)
{
    // 1. 更新数据
    fix_control_send.yaw = msg->yaw;
    fix_control_send.pitch = msg->pitch;
    fix_control_send.fire = msg->fire_advise; 
    
    // 2. [新增] 更新最后接收时间
    last_gimbal_time_ = this->get_clock()->now();
    // this->send_merged_control();
}
void BaseController::control2serial(const def_msg::msg::CommonControl::UniquePtr msg){
    RCLCPP_INFO(get_logger(),"common control started");
    control_data.velocity_top = msg->velocity_top;
    control_data.shoot = msg->shoot;
    defUartSend(control);
}

void BaseController::send_merged_control() {
  // M2修复: 使用成员变量替代静态变量，支持生命周期管理
  // has_ever_received 和 init_*_time 在构造时初始化

  // 初始拦截锁：在收到任何第一条消息之前，绝对保持静默，不发数据
  if (!has_ever_received_) {
      if (last_cmd_time_ != init_cmd_time_ || 
          last_gimbal_time_ != init_gimbal_time_ || 
          last_spin_time_ != init_spin_time_) {
          has_ever_received_ = true; // 终于收到消息了，解除拦截锁！
      } else {
          return; // 还没收到过任何消息，静悄悄地退出
      }
  }

  auto current_time = this->get_clock()->now();

  // R1修复: 使用可配置的话题超时参数 (默认100ms)
  double timeout_sec = topic_timeout_ms_ / 1000.0;
  bool cmd_alive = (current_time - last_cmd_time_).seconds() <= timeout_sec;
  bool gimbal_alive = (current_time - last_gimbal_time_).seconds() <= timeout_sec;
  bool spin_alive = (current_time - last_spin_time_).seconds() <= timeout_sec;

  // ====================================================================
  // 【核心满足你的要求】：如果所有被订阅的话题都超时“死掉”了，直接停止发送！
  // ====================================================================
  if (!cmd_alive && !gimbal_alive && !spin_alive) {
      return; 
  }

  // ====================================================================
  // 走到这里，说明【至少有一个】话题还在正常发送，那我们就拼包把它发出去
  // ====================================================================
  
  // (1) 检查底盘是否阵亡，如果底盘挂了而云台还在，底盘数据清零
  if (!cmd_alive) {
      fix_control_send.vx = 0.0;
      fix_control_send.vy = 0.0;
      fix_control_send.vz = 0.0;
  }

  // (2) 检查云台是否接管或阵亡
  if (override_gimbal_) {
      fix_control_send.yaw = 0.0;
      fix_control_send.pitch = 0.0;
      fix_control_send.fire = 0;
  } else if (!gimbal_alive) {
      fix_control_send.yaw = 0.0;
      fix_control_send.pitch = 0.0;
      fix_control_send.fire = 0;
  }

  // (3) 检查小陀螺是否阵亡
  if (!spin_alive) {
      current_spin_speed = 0.0;
  }

  // 互斥逻辑
  if (std::abs(current_spin_speed) > 0.001) {
      fix_control_send.spin = current_spin_speed;
  } else {
      fix_control_send.spin = 0.0;
  }

  // 4. 执行串口发送
  RCLCPP_INFO(this->get_logger(), 
      "发送串口数据 -> [底盘] vx:%.2f vy:%.2f vz:%.2f spin:%.2f | [云台] yaw:%.2f pitch:%.2f fire:%d",
      fix_control_send.vx,
      fix_control_send.vy,
      fix_control_send.vz,
      fix_control_send.spin,
      fix_control_send.yaw,
      fix_control_send.pitch,
      fix_control_send.fire
  );
  
  defUartSend(fix_control);
}

void BaseController::heartbeat2serial(){
  heartbeat_send.timestamp = this->get_clock()->now().seconds();
  //待补充
  //defUartSend(heartbeat);
}
//rclcpp::Clock().now()，get_clock()->now()和this->now()获取到的时间与std::chrono::system_clock::now()是一致的。
//这里需要注意的一点是，rclcpp::Clock().now()，get_clock()->now()和this->now()获取到的时间戳均包含seconds()和nanoseconds()方法。seconds()和nanoseconds()方法都返回了当前的时间，是等价的，只是单位不一样。一个是以秒为单位，一个是纳秒为单位。



void BaseController::speed2odom(){
    ///////////////////////////////////////////////// odom ////////////////////////////////////////////////////////
    speed.last_time = speed.current_time;   //上一次时间
    speed.current_time = get_clock()->now();  //新时间

    //update speed and time stamp
    speed.vx = chassis_receive.vx;
    speed.vy = chassis_receive.vy;
    speed.vz = chassis_receive.vz;

    //compute odometry
    float dt =  speed.current_time.seconds()-speed.last_time.seconds();//time distance
    float delta_x = (speed.vx * cos(odom_raw.z) - speed.vy * sin(odom_raw.z)) * dt;
    float delta_y = (speed.vx * sin(odom_raw.z) + speed.vy * cos(odom_raw.z)) * dt;
    float delta_th = speed.vz * odom_raw.dt;
    //add then odom added
    odom_raw.x += delta_x;
    odom_raw.y += delta_y;
    odom_raw.z += delta_th;

    //pub odom
    nav_msgs::msg::Odometry odom{};
    odom.header.stamp = speed.current_time;
    odom.header.frame_id = "raw/odom";
    odom.child_frame_id = "base_link";   //turtlename_.c_str();
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_raw.z);
    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::msg::Quaternion odom_quat; //tf2_ros::createQuaternionMsgFromRollPitchYaw(0,0,odom_raw.z);
    odom_quat.x=q.x();
    odom_quat.y=q.y();
    odom_quat.z=q.z();
    odom_quat.w=q.w();

    //set the position
    odom.pose.pose.position.x = odom_raw.x;
    odom.pose.pose.position.y = odom_raw.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
 
    // set the velocity
    odom.twist.twist.linear.x = speed.vx; //liner speed
    odom.twist.twist.linear.y = speed.vy;
    odom.twist.twist.angular.z = speed.vz;   // angle speed
    odom_pub->publish(odom);
    
    ///////////////////////////////////////////////// tf trans ////////////////////////////////////////////////////////
    //transform over tf
    
    geometry_msgs::msg::TransformStamped odom_trans{};
    odom_trans.header.stamp = speed.current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
  
    odom_trans.transform.translation.x = 0.0;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0; //this parma has different meaning with odom_raw.z.
    //odom_trans.transform.rotation = tf2::createQuaternionMsgFromYaw(odom_raw.z);    only used in ros1
    //tf2::Quaternion q;
    q.setRPY(0, 0, odom_raw.z);
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();
    //send transform
    odom_broadcaster->sendTransform(odom_trans);
}

void BaseController::serial2global(){
    static unsigned long long current_time;
    current_time = rclcpp::Time().nanoseconds();
    //RCLCPP_INFO(this->get_logger(),"publish gobal information");
    def_msg::msg::GobalInformation status{};  
    //timestamp

    //heartbeat
    if((current_time - heartbeat_receive.timestamp)*10/CLOCKS_PER_SEC > TIMESTAMP_ASSERT_DELAY){
      RCLCPP_WARN(this->get_logger(),"--下位机心跳数据超时，断连警告--");
    }
    status.battery = sentry_state_receive.battery;
    status.life_extra = sentry_state_receive.life;
    status.color = sentry_state_receive.color;
    status.bullet_extra = sentry_state_receive.bullet;
    status.fault_flag = sentry_state_receive.fault_flag;

    //additional
    status.launch = additional_data.launch; //if_launch
    status.arm = additional_data.arm;

    //game_data
    status.stage_remain_time = game_status_data.stage_remain_time;
    status.game_progress = game_status_data.game_progress;
    gobal_information_pub->publish(status);

    // =========================================================================
    // ================== [新增] 裁判系统状态数据打包与发布 ====================
    // =========================================================================

    // 1. 发布 GameStatus
    pb_rm_interfaces::msg::GameStatus game_msg;
    // 当 flag = 1 时，game_progress 填 4 (RUNNING)，其他情况填 0 (NOT_START)
    if (sentry_state_receive.fault_flag == 1) {
        game_msg.game_progress = 4;
    } else {
        game_msg.game_progress = 0;
    }
    game_msg.stage_remain_time = 0; // 其他填0
    game_status_pub_->publish(game_msg);

    // 2. 发布 RobotStatus
    pb_rm_interfaces::msg::RobotStatus robot_msg;
    // 动态提取血量
    robot_msg.current_hp = sentry_state_receive.life;
    
    // 其余数据按照你的要求严格写死
    robot_msg.robot_id = 1;
    robot_msg.robot_level = 1;
    robot_msg.maximum_hp = 500;
    robot_msg.shooter_barrel_cooling_value = 100;
    robot_msg.shooter_barrel_heat_limit = 350;
    robot_msg.shooter_17mm_1_barrel_heat = 0;
    
    // 姿态默认值 (Position 全 0, Orientation: w=1.0)
    robot_msg.robot_pos.position.x = 0.0;
    robot_msg.robot_pos.position.y = 0.0;
    robot_msg.robot_pos.position.z = 0.0;
    robot_msg.robot_pos.orientation.x = 0.0;
    robot_msg.robot_pos.orientation.y = 0.0;
    robot_msg.robot_pos.orientation.z = 0.0;
    robot_msg.robot_pos.orientation.w = 1.0;
    
    robot_msg.armor_id = 0;
    robot_msg.hp_deduction_reason = 0;
    robot_msg.projectile_allowance_17mm = 100;
    robot_msg.remaining_gold_coin = 1000;
    robot_msg.is_hp_deduced = false;

    robot_status_pub_->publish(robot_msg);


}


//receive gimble data and pub
void BaseController::serial2gimble(){
  cntLossConnection();
  auto current_time = get_clock()->now();
  //recvive gimble data and pub data
  def_msg::msg::GimbleControl gimble_recv{};
  gimble_recv.header.stamp = current_time;
  gimble_recv.header.frame_id = "gimbal_link"; 
  gimble_recv.yaw = gimbal_receive.yaw;
  gimble_recv.pitch = gimbal_receive.pitch;
  //gimble_recv.roll = gimbal_receive.roll;
  gimble_pub->publish(gimble_recv);
  // RCLCPP_INFO(get_logger(),"serial gimble pose data received and pub");
  // RCLCPP_WARN(get_logger(),"losing rate:%f",error_lost);
  /*======================================================*/
  /*
  //tf pub yaw
  geometry_msgs::msg::TransformStamped gimble_trans_yaw{}; //transform from chassis/world to gimble
  gimble_trans_yaw.header.stamp = current_time;
  gimble_trans_yaw.header.frame_id = "base_link";
  gimble_trans_yaw.child_frame_id = "gimble_yaw";
 
  gimble_trans_yaw.transform.translation.x = 0;
  gimble_trans_yaw.transform.translation.y = 0;
  gimble_trans_yaw.transform.translation.z = 0; //this parma has different meaning with odom_raw.z.
   
  tf2::Quaternion q;
  q.setRPY(0, 0, gimble_status.yaw);
  gimble_trans_yaw.transform.rotation.x = q.x();
  gimble_trans_yaw.transform.rotation.y = q.y();
  gimble_trans_yaw.transform.rotation.z = q.z();
  gimble_trans_yaw.transform.rotation.w = q.w();
  //send trans yaw
  gimble_broadcaster->sendTransform(gimble_trans_yaw);


  //tf pub pitch
  geometry_msgs::msg::TransformStamped gimble_trans_pitch{}; //transform from chassis/world to gimble
  gimble_trans_pitch.header.stamp = current_time;
  gimble_trans_pitch.header.frame_id = "gimble_yaw";
  gimble_trans_pitch.child_frame_id = "gimble_pitch";
  gimble_trans_pitch.transform.translation.x = 0;
  gimble_trans_pitch.transform.translation.y = 0;
  gimble_trans_pitch.transform.translation.z = 0; //this parma has different meaning with odom_raw.z.
  q.setRPY(0,gimble_status.pitch,0);
  gimble_trans_pitch.transform.rotation.x = q.x();
  gimble_trans_pitch.transform.rotation.y = q.y();
  gimble_trans_pitch.transform.rotation.z = q.z();
  gimble_trans_pitch.transform.rotation.w = q.w();
  //send trans pitch
  gimble_broadcaster->sendTransform(gimble_trans_pitch);
  */
}


void BaseController::serial2imu(){
    sensor_msgs::msg::Imu imu{};
    imu.header.stamp = this->get_clock()->now();
    imu.header.frame_id = "base_footprint";
    imu.angular_velocity.x = imu2_data.vx;
    imu.angular_velocity.y = imu2_data.vy;
    imu.angular_velocity.z = imu2_data.vz;
    imu.linear_acceleration.x = imu2_data.ax;
    imu.linear_acceleration.y = imu2_data.ay;
    imu.linear_acceleration.z = imu2_data.az;
    imu2_pub->publish(imu);
}



void BaseController::serial2joint(){
  static float temp_yaw = 0,last_yaw = 0;
  static int rount = 0;
  temp_yaw = yaw_data.yaw;
  if(temp_yaw < 0){
    temp_yaw  = 360.0-temp_yaw;
  }
  if ((last_yaw - temp_yaw) < -180.0){
     rount --; 
  }
  else if ((last_yaw - temp_yaw) > 180.0){
     rount ++;
  } 
  float yaw_ = rount * 360 + temp_yaw;
  last_yaw = temp_yaw;
  sensor_msgs::msg::JointState joint{};
  joint.header.set__frame_id("");
  joint.header.set__stamp(this->get_clock()->now());
  joint.name = {"pitch_joint","yaw_joint"};
  joint.velocity = {0,0};
  joint.position = {gimble_current.pitch,yaw_};
  joint.effort = {0,0};
  joint_pub->publish(joint);
}
void BaseController::mux_service_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response) 
{
  override_gimbal_ = request->data;
  response->success = true;
  
  if (override_gimbal_) {
      response->message = "Gimbal override ENABLED: Yaw=0, Pitch=0, Fire=0";
  } else {
      response->message = "Gimbal override DISABLED: Auto-aim resumed";
  }
  
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}
int main(int argc, char* argv[]) {
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	// C1修复: 先创建 uart 实例再访问；端口由 launch 参数 setting，不再硬编码
	uart = std::make_shared<UartLinux>();
	uart->startReading();

	auto base_control_node = std::make_shared<BaseController>("hardware_serial");
	rclcpp::spin(base_control_node);
	rclcpp::shutdown();
	return 0;
}

