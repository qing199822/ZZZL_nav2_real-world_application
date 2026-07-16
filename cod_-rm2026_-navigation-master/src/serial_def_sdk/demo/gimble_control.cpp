class ActionClient : public rclcpp::Node {
 public:
  using GimbleCommand = def_msg::action::GimbleCommand;
  using GimbleCommandClient = rclcpp_action::ClientGoalHandle<GimbleCommand>;

  explicit ActionClient(
      std::string name,
      const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
      : Node(name, node_options) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());

    this->client_ptr_ =
        rclcpp_action::create_client<GimbleCommand>(this, "GimbleCommand");

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&ActionClient::send_goal, this));
  }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();    //cancel the timer to avoid frequent

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = GimbleCommand::Goal();
    goal_msg.distance = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<GimbleCommand>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&ActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&ActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

  }

 private:
  rclcpp_action::Client<GimbleCommand>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(GimbleCommandClient::SharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GimbleCommandClient::SharedPtr,
      const std::shared_ptr<const GimbleCommand::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback current pose:%f", feedback->pose);
  }

  void result_callback(const GimbleCommandClient::WrappedResult& result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received: %f", result.result->pose);
    // rclcpp::shutdown();
  }
};  // class ActionClient
