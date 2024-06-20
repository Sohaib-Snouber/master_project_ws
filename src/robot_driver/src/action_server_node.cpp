#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_interfaces/action/process_task.hpp"
#include <master_project_msgs/msg/task.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/robotiq_gripper.h>
#include <thread>
#include <random>
using namespace std::chrono_literals;

using namespace ur_rtde;

class ProcessTaskActionServer : public rclcpp::Node
{
public:
  using ProcessTask = action_interfaces::action::ProcessTask;
  using GoalHandleProcessTask = rclcpp_action::ServerGoalHandle<ProcessTask>;


  explicit ProcessTaskActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("process_task_action_server", options), ip_("10.130.1.100")
  {
    this->action_server_ = rclcpp_action::create_server<ProcessTask>(
      this,
      "process_task",
      std::bind(&ProcessTaskActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ProcessTaskActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ProcessTaskActionServer::handle_accepted, this, std::placeholders::_1));

    rtde_control_ = std::make_unique<RTDEControlInterface>(ip_);
    rtde_receive_ = std::make_unique<RTDEReceiveInterface>(ip_);
    gripper_ = std::make_unique<RobotiqGripper>(ip_);

    task_details_subscriber_ = this->create_subscription<master_project_msgs::msg::Task>(
        "task_details", 10, std::bind(&ProcessTaskActionServer::taskDetailsCallback, this, std::placeholders::_1));

    // Debug: Print statements for gripper connection and activation
    RCLCPP_INFO(this->get_logger(), "Connecting to gripper...");
    gripper_->connect();
    RCLCPP_INFO(this->get_logger(), "Gripper connected.");
    gripper_->activate();
    RCLCPP_INFO(this->get_logger(), "Gripper activated.");
  }

private:
  rclcpp::Subscription<master_project_msgs::msg::Task>::SharedPtr task_details_subscriber_;
  rclcpp_action::Server<ProcessTask>::SharedPtr action_server_;
  std::string ip_;
  std::unique_ptr<RTDEControlInterface> rtde_control_;
  std::unique_ptr<RTDEReceiveInterface> rtde_receive_;
  std::unique_ptr<RobotiqGripper> gripper_;
  master_project_msgs::msg::Task task; 

  void taskDetailsCallback(const master_project_msgs::msg::Task::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received task details: %s", msg->name.c_str());
    task = *msg;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ProcessTask::Goal> goal)
  {
    if (goal->move_robot) {
      RCLCPP_INFO(this->get_logger(), "Received goal request to move the robot");
    } else if (goal->request_open_gripper) {
      RCLCPP_INFO(this->get_logger(), "Received goal request to open gripper");
    } else if (goal->request_close_gripper) {
      RCLCPP_INFO(this->get_logger(), "Received goal request to close gripper");
    } else if (goal->request_greeting) {
      RCLCPP_INFO(this->get_logger(), "Received goal request for greeting message");
    } else if (goal->check_robot_status) {
      RCLCPP_INFO(this->get_logger(), "Received goal request to check robot status");
    }
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleProcessTask> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleProcessTask> goal_handle)
  {
    std::thread{std::bind(&ProcessTaskActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleProcessTask> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ProcessTask::Result>();

    // Generate a unique ID for this goal
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, 10000);
    int goal_id = dis(gen);
    result->goal_id = goal_id;

    if (goal->move_robot) {
      // Implement the logic to move the robot based on the task details
      RCLCPP_INFO(this->get_logger(), "Moving the robot based on task details...");
      master_project_msgs::msg::Task task_msg;
      processTask(task); // Example usage of task processing
    } else if (goal->request_open_gripper) {
      gripperOpen();
    } else if (goal->request_close_gripper) {
      gripperClose();
    } else if (goal->check_robot_status) {
      checkRobotStatus(goal_handle, result);
    } else if (goal->request_greeting) {
      // Greeting message
      result->greeting_message = "Successfully finished all tasks!";
    }

    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded with ID %d", goal_id);
    }
  }

  void checkRobotStatus(const std::shared_ptr<GoalHandleProcessTask> goal_handle, std::shared_ptr<ProcessTask::Result> result)
  {
    RCLCPP_INFO(this->get_logger(), "Checking robot status...");
    while (isRobotMoving()) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled while checking robot status");
        return;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    RCLCPP_INFO(this->get_logger(), "Robot has stopped moving.");
    std::this_thread::sleep_for(std::chrono::seconds(3));
  }

  bool isRobotMoving() 
  {
    int checks = 5;
    double tolerance = 1e-4; // Adjust tolerance 0.1 mm

    for (int i = 0; i < checks; ++i) {
      auto target_q = rtde_receive_->getTargetQ();
      auto actual_q = rtde_receive_->getActualQ();
      
      RCLCPP_INFO(this->get_logger(), "Check %d - Target Q: ", i+1);
      for (const auto &q : target_q) {
        RCLCPP_INFO(this->get_logger(), "%f ", q);
      }
      RCLCPP_INFO(this->get_logger(), "Actual Q: ");
      for (const auto &q : actual_q) {
        RCLCPP_INFO(this->get_logger(), "%f ", q);
      }

      bool moving = false;
      for (size_t j = 0; j < target_q.size(); ++j) {
        double diff = target_q[j] - actual_q[j];
        if (std::fabs(diff) > tolerance) {
          RCLCPP_INFO(this->get_logger(), "difference is %f ", diff);
          RCLCPP_INFO(this->get_logger(), "the defined tolerance is %f ", tolerance);
          RCLCPP_INFO(this->get_logger(), "Robot is moving");
          moving = true;
          break;
        }
      }

      if (!moving) {
        RCLCPP_INFO(this->get_logger(), "Robot is not moving");
      } else {
        return true; // If any check finds the robot moving, return true
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for 100ms between checks
    }

    // If all checks return false, the robot is not moving
    return false;
  }

  /* bool isRobotMoving() {
    auto target_q = rtde_receive_->getTargetQ();
    auto actual_q = rtde_receive_->getActualQ();
    
    RCLCPP_INFO(this->get_logger(), "Target Q: ");
    for (const auto &q : target_q) {
      RCLCPP_INFO(this->get_logger(), "%f ", q);
    }
    RCLCPP_INFO(this->get_logger(), "Actual Q: ");
    for (const auto &q : actual_q) {
      RCLCPP_INFO(this->get_logger(), "%f ", q);
    }

    double tolerance = 4e-5; // Adjust tolerance
    for (size_t i = 0; i < target_q.size(); ++i) {
      if (std::fabs(target_q[i] - actual_q[i]) > tolerance) {
        RCLCPP_INFO(this->get_logger(), "Robot is moving");
        return true;
      }
    }
    RCLCPP_INFO(this->get_logger(), "Robot is not moving");
    return false;
  } */

  void processTask(const master_project_msgs::msg::Task& task)
  {
    for (const auto& stage : task.stages)
    {
      RCLCPP_INFO(this->get_logger(), "Executing stage: %s", stage.name.c_str());
      
      RCLCPP_INFO(this->get_logger(), "Number of waypoints in stage %s: %zu", stage.name.c_str(), stage.waypoints.size());
      if (stage.waypoints.size() == 0) {
        RCLCPP_INFO(this->get_logger(), "Stage %s has no waypoints. Skipping...", stage.name.c_str());
        std::this_thread::sleep_for(50ms);
        continue;
      }

      std::vector<std::vector<double>> robot_path;
      double initial_gripper_position = -1;
      double final_gripper_position = -1;
      double finger_change = 0.1;

      for (size_t i = 0; i < stage.waypoints.size(); ++i)
      {
        const auto& waypoint = stage.waypoints[i];
        std::vector<double> joint_positions(6, 0.0);  // Ensure this only has 6 elements for the robot arm joints

        size_t joint_index = 0;
        for (const auto& joint : waypoint.joints)
        {
          if (joint_index < 6) {  // Only handle the first 6 joints for the robot arm
            joint_positions[joint_index] = joint.position;
          } else if (joint_index == 6) {  // Handle the gripper position
            if (i == stage.waypoints.size() - 1) {
              final_gripper_position = map_finger_joint_to_gripper(joint.position);
            }
            if (i == 0) {
              initial_gripper_position = map_finger_joint_to_gripper(joint.position);
            }
          }
          joint_index++;
        }

        std::vector<double> one_pos(joint_positions.begin(), joint_positions.end());
        one_pos.push_back(1.0);  // velocity
        one_pos.push_back(1.2);  // acceleration
        one_pos.push_back(0.1);  // blend (5mm tolerance)
        robot_path.push_back(one_pos);
      }

      // Debug: Print gripper positions
      RCLCPP_INFO(this->get_logger(), "Initial gripper position: %f", initial_gripper_position);
      RCLCPP_INFO(this->get_logger(), "Final gripper position: %f", final_gripper_position);

      if (abs(abs(initial_gripper_position) - abs(final_gripper_position)) > finger_change) {
        // Debug: Before moving the gripper
        RCLCPP_INFO(this->get_logger(), "Moving gripper to position: %f", final_gripper_position);
        final_gripper_position = std::clamp(final_gripper_position, 0.0, 1.0);
        RCLCPP_INFO(this->get_logger(), "Moving gripper to position: %f", final_gripper_position);
        gripper_->move(final_gripper_position);
        // Debug: After moving the gripper
        RCLCPP_INFO(this->get_logger(), "Gripper moved to position: %f", final_gripper_position);
      } else {
        rtde_control_->moveJ(robot_path, true);
      }

      std::this_thread::sleep_for(50ms);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Delay to simulate gripper motion time
  }

  double map_value(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
  }

  double map_finger_joint_to_gripper(double finger_joint_position) {
    double close_finger_joint = 0.69991196175;
    double open_finger_joint = 0.07397215645645;
    return map_value(finger_joint_position, open_finger_joint, close_finger_joint, 0.988235, 0.105882);
  }

  void gripperOpen() {
    RCLCPP_INFO(this->get_logger(), "Opening gripper");
    gripper_->move(1.0); // Assuming 1.0 is the open position
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Delay to simulate gripper motion time
  }

  void gripperClose() {
    RCLCPP_INFO(this->get_logger(), "Closing gripper");
    gripper_->move(0.0); // Assuming 0.0 is the closed position
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Delay to simulate gripper motion time
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProcessTaskActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
