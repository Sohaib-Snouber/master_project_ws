#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_interfaces/action/process_task.hpp"
#include <vector>
#include <chrono>

class DualClientNode : public rclcpp::Node
{
public:
  using ProcessTask = action_interfaces::action::ProcessTask;
  using GoalHandleProcessTask = rclcpp_action::ClientGoalHandle<ProcessTask>;

  explicit DualClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("dual_client_node", options)
  {
    this->client_ = rclcpp_action::create_client<ProcessTask>(this, "process_task");

    // Start the first goal to move the robot
    this->send_move_robot_goal();
  }

private:
  rclcpp_action::Client<ProcessTask>::SharedPtr client_;
  std::vector<int32_t> sequence_;  // Vector to store the sequence result

  void send_move_robot_goal()
  {
    using namespace std::placeholders;

    if (!this->client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = ProcessTask::Goal();
    goal_msg.move_robot = true;
    goal_msg.request_open_gripper = false;
    goal_msg.request_close_gripper = false;
    goal_msg.request_greeting = false;
    goal_msg.check_robot_status = false;

    RCLCPP_INFO(this->get_logger(), "Sending move robot goal");

    auto send_goal_options = rclcpp_action::Client<ProcessTask>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&DualClientNode::move_robot_goal_response_callback, this, _1);
    send_goal_options.result_callback =
      std::bind(&DualClientNode::move_robot_result_callback, this, _1);
    this->client_->async_send_goal(goal_msg, send_goal_options);
  }

  void move_robot_goal_response_callback(GoalHandleProcessTask::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Move robot goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Move robot goal accepted by server, waiting for result");
    }
  }

  void move_robot_result_callback(const GoalHandleProcessTask::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Move robot result received with goal ID %d", result.result->goal_id);
        // After receiving the result, send the open gripper goal
        send_check_robot_status_after_move_goal();
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Move robot goal was aborted");
        rclcpp::shutdown();
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Move robot goal was canceled");
        rclcpp::shutdown();
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code for move robot goal");
        rclcpp::shutdown();
        return;
    }
  }

  void send_check_robot_status_after_move_goal()
  {
    using namespace std::placeholders;

    if (!this->client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = ProcessTask::Goal();
    goal_msg.check_robot_status = true;
    goal_msg.request_open_gripper = false;
    goal_msg.request_close_gripper = false;
    goal_msg.request_greeting = false;
    goal_msg.move_robot = false;

    RCLCPP_INFO(this->get_logger(), "Sending check robot status goal after move");

    auto send_goal_options = rclcpp_action::Client<ProcessTask>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&DualClientNode::check_robot_status_after_move_goal_response_callback, this, _1);
    send_goal_options.result_callback =
      std::bind(&DualClientNode::check_robot_status_after_move_result_callback, this, _1);
    this->client_->async_send_goal(goal_msg, send_goal_options);
  }

  void check_robot_status_after_move_goal_response_callback(GoalHandleProcessTask::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Check robot status goal after move was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Check robot status goal after move accepted by server, waiting for result");
    }
  }

  void check_robot_status_after_move_result_callback(const GoalHandleProcessTask::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Check robot status result after move received with goal ID %d: ", result.result->goal_id);
        // After receiving the result, send the open gripper goal
        send_open_gripper_goal();
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Check robot status goal after move was aborted");
        rclcpp::shutdown();
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Check robot status goal after move was canceled");
        rclcpp::shutdown();
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code for check robot status goal after move");
        rclcpp::shutdown();
        return;
    }
  }

  void send_open_gripper_goal()
  {
    using namespace std::placeholders;

    if (!this->client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = ProcessTask::Goal();
    goal_msg.request_open_gripper = true;  // Request open gripper
    goal_msg.request_close_gripper = false;
    goal_msg.request_greeting = false;
    goal_msg.check_robot_status = false;
    goal_msg.move_robot = false;

    RCLCPP_INFO(this->get_logger(), "Sending open gripper goal");

    auto send_goal_options = rclcpp_action::Client<ProcessTask>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&DualClientNode::open_gripper_goal_response_callback, this, _1);
    send_goal_options.result_callback =
      std::bind(&DualClientNode::open_gripper_result_callback, this, _1);
    this->client_->async_send_goal(goal_msg, send_goal_options);
  }

  void open_gripper_goal_response_callback(GoalHandleProcessTask::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Open gripper goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Open gripper goal accepted by server, waiting for result");
    }
  }

  void open_gripper_result_callback(const GoalHandleProcessTask::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Open gripper result received with goal ID %d: ", result.result->goal_id);
        // After receiving the result, send the check robot status goal
        send_check_robot_status_goal();
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Open gripper goal was aborted");
        rclcpp::shutdown();
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Open gripper goal was canceled");
        rclcpp::shutdown();
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code for open gripper goal");
        rclcpp::shutdown();
        return;
    }
  }

  void send_check_robot_status_goal()
  {
    using namespace std::placeholders;

    if (!this->client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = ProcessTask::Goal();
    goal_msg.check_robot_status = true;
    goal_msg.request_open_gripper = false;
    goal_msg.request_close_gripper = false;
    goal_msg.request_greeting = false;
    goal_msg.move_robot = false;

    RCLCPP_INFO(this->get_logger(), "Sending check robot status goal");

    auto send_goal_options = rclcpp_action::Client<ProcessTask>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&DualClientNode::check_robot_status_goal_response_callback, this, _1);
    send_goal_options.result_callback =
      std::bind(&DualClientNode::check_robot_status_result_callback, this, _1);
    this->client_->async_send_goal(goal_msg, send_goal_options);
  }

  void check_robot_status_goal_response_callback(GoalHandleProcessTask::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Check robot status goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Check robot status goal accepted by server, waiting for result");
    }
  }

  void check_robot_status_result_callback(const GoalHandleProcessTask::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Check robot status result received with goal ID %d: ", result.result->goal_id);
        // After receiving the result, send the close gripper goal
        send_close_gripper_goal();
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Check robot status goal was aborted");
        rclcpp::shutdown();
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Check robot status goal was canceled");
        rclcpp::shutdown();
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code for check robot status goal");
        rclcpp::shutdown();
        return;
    }
  }

  void send_close_gripper_goal()
  {
    using namespace std::placeholders;

    if (!this->client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = ProcessTask::Goal();
    goal_msg.request_open_gripper = false;
    goal_msg.request_close_gripper = true;  // Request close gripper
    goal_msg.request_greeting = false;
    goal_msg.move_robot = false;
    goal_msg.check_robot_status = false;

    RCLCPP_INFO(this->get_logger(), "Sending close gripper goal");

    auto send_goal_options = rclcpp_action::Client<ProcessTask>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&DualClientNode::close_gripper_goal_response_callback, this, _1);
    send_goal_options.result_callback =
      std::bind(&DualClientNode::close_gripper_result_callback, this, _1);
    this->client_->async_send_goal(goal_msg, send_goal_options);
  }

  void close_gripper_goal_response_callback(GoalHandleProcessTask::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Close gripper goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Close gripper goal accepted by server, waiting for result");
    }
  }

  void close_gripper_result_callback(const GoalHandleProcessTask::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Close gripper result received with goal ID %d: ", result.result->goal_id);
        // After receiving the result, send the greeting goal
        send_greeting_goal();
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Close gripper goal was aborted");
        rclcpp::shutdown();
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Close gripper goal was canceled");
        rclcpp::shutdown();
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code for close gripper goal");
        rclcpp::shutdown();
        return;
    }
  }

  void send_greeting_goal()
  {
    using namespace std::placeholders;

    if (!this->client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = ProcessTask::Goal();
    goal_msg.request_open_gripper = false;
    goal_msg.request_close_gripper = false;
    goal_msg.request_greeting = true;  // Request greeting
    goal_msg.move_robot = false;
    goal_msg.check_robot_status = false;

    RCLCPP_INFO(this->get_logger(), "Sending greeting goal");

    auto send_goal_options = rclcpp_action::Client<ProcessTask>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&DualClientNode::greeting_goal_response_callback, this, _1);
    send_goal_options.result_callback =
      std::bind(&DualClientNode::greeting_result_callback, this, _1);
    this->client_->async_send_goal(goal_msg, send_goal_options);
  }

  void greeting_goal_response_callback(GoalHandleProcessTask::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Greeting goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Greeting goal accepted by server, waiting for result");
    }
  }

  void greeting_result_callback(const GoalHandleProcessTask::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Greeting result received with goal ID %d: %s", result.result->goal_id, result.result->greeting_message.c_str());
        // All tasks completed
        RCLCPP_INFO(this->get_logger(), "All tasks completed successfully.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Greeting goal was aborted");
        rclcpp::shutdown();
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Greeting goal was canceled");
        rclcpp::shutdown();
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code for greeting goal");
        rclcpp::shutdown();
        return;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto dual_client_node = std::make_shared<DualClientNode>();
  rclcpp::spin(dual_client_node);
  rclcpp::shutdown();
  return 0;
}
