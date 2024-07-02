#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_interfaces/action/full_drive.hpp"
#include "full_drive/update_planning_scene.h"
#include "full_drive/move.h"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/robotiq_gripper.h>
#include <thread>
#include <chrono>
#include <random>
#include <unordered_map>
#include <string>
#include <vector>
#include <algorithm>
#include "master_project_msgs/msg/move_group_action_details.hpp"  // Add this line


#if __cplusplus < 201703L  // If the C++ version is less than C++17, define clamp
template <typename T>
const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}
#endif

using namespace ur_rtde;
using FullDrive = action_interfaces::action::FullDrive;
using GoalHandleFullDrive = rclcpp_action::ServerGoalHandle<FullDrive>;

// Structure to hold object properties
struct ObjectProperties {
    bool exists;
    bool collision_allowed;
    std::vector<std::string> allowed_collision_objects;
};

class FullDriveActionServer : public rclcpp::Node {
public:
    FullDriveActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("full_drive_action_server", options), ip_("10.130.1.100") {
        this->action_server_ = rclcpp_action::create_server<FullDrive>(
            this,
            "full_drive",
            std::bind(&FullDriveActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FullDriveActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&FullDriveActionServer::handle_accepted, this, std::placeholders::_1));

        rtde_control_ = std::make_unique<RTDEControlInterface>(ip_);
        rtde_receive_ = std::make_unique<RTDEReceiveInterface>(ip_);
        gripper_ = std::make_unique<RobotiqGripper>(ip_);

        task_details_subscriber_ = this->create_subscription<master_project_msgs::msg::MoveGroupActionDetails>(
            "task_details", 10, std::bind(&FullDriveActionServer::taskDetailsCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Connecting to gripper...");
        gripper_->connect();
        RCLCPP_INFO(this->get_logger(), "Gripper connected.");
        gripper_->activate();
        RCLCPP_INFO(this->get_logger(), "Gripper activated.");

        // Defer the initialization of shared_from_this()
        auto timer_callback = [this]() {
            planning_scene_updater_ = std::make_shared<PlanningSceneUpdater>(this->shared_from_this());
            RCLCPP_INFO(this->get_logger(), "Initialized the PlanninSceneUpdater class");
            move_group_helper_ = std::make_shared<MoveGroupHelper>(this->shared_from_this());
            RCLCPP_INFO(this->get_logger(), "Initialized the MoveGroupHelper class");
            loadExistingCollisionObjects();
            initialization_timer_->cancel(); // Cancel the timer after initialization
        };
        // Use a timer to defer the callback
        initialization_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), timer_callback);
    }

private:
    rclcpp::Subscription<master_project_msgs::msg::MoveGroupActionDetails>::SharedPtr task_details_subscriber_;
    rclcpp_action::Server<FullDrive>::SharedPtr action_server_;
    std::string ip_;
    std::unique_ptr<RTDEControlInterface> rtde_control_;
    std::unique_ptr<RTDEReceiveInterface> rtde_receive_;
    std::unique_ptr<RobotiqGripper> gripper_;
    std::shared_ptr<PlanningSceneUpdater> planning_scene_updater_;
    // Map to store objects and their properties
    std::unordered_map<std::string, ObjectProperties> object_map_;
    rclcpp::TimerBase::SharedPtr initialization_timer_;  // Store the timer to prevent it from being destroyed
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;  // Add this line
    std::shared_ptr<MoveGroupHelper> move_group_helper_;  // Add this line

    /* // example of Adding an object to the map
    object_map_["object1"] = {true, false, {"object2", "object3"}}; */

    void loadExistingCollisionObjects() {
        auto collision_objects = planning_scene_interface_.getObjects();

        for (const auto& object_pair : collision_objects) {
            const std::string& object_id = object_pair.first;
            RCLCPP_INFO(this->get_logger(), "Found existing collision object: %s", object_id.c_str());
            object_map_[object_id] = {true, false, {}};
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu existing collision objects into object_map_", object_map_.size());
    }

    void taskDetailsCallback(const master_project_msgs::msg::MoveGroupActionDetails::SharedPtr msg) {  // Change the message type
        RCLCPP_INFO(this->get_logger(), "Received task details: %s", msg->action_name.c_str());
        processTask(*msg);
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FullDrive::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        // Print the goal details
        std::cout << "Goal Details:" << std::endl;
        std::cout << "add_collision_object: " << goal->add_collision_object << std::endl;
        std::cout << "delete_collision_object: " << goal->delete_collision_object << std::endl;
        std::cout << "attach_object: " << goal->attach_object << std::endl;
        std::cout << "detach_object: " << goal->detach_object << std::endl;
        std::cout << "move_to: " << goal->move_to << std::endl;
        std::cout << "move_linear: " << goal->move_linear << std::endl;
        std::cout << "check_robot_status: " << goal->check_robot_status << std::endl;
        std::cout << "allow_collision: " << goal->allow_collision << std::endl;
        std::cout << "reenable_collision: " << goal->reenable_collision << std::endl;
        std::cout << "current_state: " << goal->current_state << std::endl;
        std::cout << "set_gripper_position: " << goal->set_gripper_position << std::endl;
        std::cout << "gripper_position: " << goal->gripper_position << std::endl;
        std::cout << "object_name: " << goal->object_name << std::endl;
        std::cout << "target_name: " << goal->target_name << std::endl;
        std::cout << "task: " << goal->task << std::endl;
        std::cout << "id: " << goal->id << std::endl;
        std::cout << "link: " << goal->link << std::endl;
        std::cout << "constrain: " << goal->constrain << std::endl;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFullDrive> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFullDrive> goal_handle) {
        std::thread{std::bind(&FullDriveActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    bool execute(const std::shared_ptr<GoalHandleFullDrive> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<FullDrive::Result>();
        bool success = false;

        MoveGroupHelper move_group_helper(this->shared_from_this());

        if (goal->add_collision_object) {
            if (object_map_.find(goal->target_name) != object_map_.end()) {
                RCLCPP_ERROR(this->get_logger(), "Collision object already exists: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Collision object already exists.";
                goal_handle->abort(result);
                return true;
            }
            RCLCPP_INFO(this->get_logger(), "Adding collision object");
            planning_scene_updater_->addCollisionObject(goal->target_name, goal->object_primitive, goal->object_pose, goal->color);
            object_map_[goal->target_name] = {true, false, {}};
            success = true;

        } else if (goal->delete_collision_object) {
            if (object_map_.find(goal->target_name) == object_map_.end()) {
                RCLCPP_ERROR(this->get_logger(), "Collision object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Collision object does not exist.";
                goal_handle->abort(result);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Deleting collision object");
            planning_scene_updater_->removeCollisionObject(goal->target_name);
            object_map_.erase(goal->target_name);
            success = true;

        } else if (goal->attach_object) {
            if (object_map_.find(goal->target_name) == object_map_.end() || !object_map_[goal->target_name].exists) {
                RCLCPP_ERROR(this->get_logger(), "Object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Object does not exist.";
                goal_handle->abort(result);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Attaching object");
            success = move_group_helper_->attachObject(goal->target_name, goal->link);
            result->success = success;

        } else if (goal->detach_object) {
            if (object_map_.find(goal->target_name) == object_map_.end() || !object_map_[goal->target_name].exists) {
                RCLCPP_ERROR(this->get_logger(), "Object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Object does not exist.";
                goal_handle->abort(result);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Detaching object");
            success = move_group_helper_->detachObject(goal->target_name, goal->link);
            result->success = success;

        } else if (goal->move_to) {
            RCLCPP_INFO(this->get_logger(), "Moving to pose");
            success = move_group_helper_->moveToPose(goal->target_pose, goal->constrain);
            result->success = success;

        } else if (goal->move_linear) {
            RCLCPP_INFO(this->get_logger(), "Moving linearly to pose");
            success = move_group_helper_->moveLinear(goal->target_pose);
            result->success = success;

        } else if (goal->check_robot_status) {
            success = checkRobotStatus(goal_handle, result);

        } else if (goal->allow_collision) {
            if (goal->target_name != "hand" && object_map_.find(goal->target_name) == object_map_.end()) {
                RCLCPP_ERROR(this->get_logger(), "Object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Object does not exist.";
                goal_handle->abort(result);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Allowing collision");
            success = move_group_helper_->allowCollision(goal->target_name, goal->object_name, true);
            result->success = success;

        } else if (goal->reenable_collision) {
            if (goal->target_name != "hand" && goal->target_name != "base_link_inertia" && object_map_.find(goal->target_name) == object_map_.end()) {
                RCLCPP_ERROR(this->get_logger(), "Object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Object does not exist.";
                goal_handle->abort(result);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Re-enabling collision");
            success = move_group_helper_->allowCollision(goal->target_name, goal->object_name, false);
            result->success = success;

        } else if (goal->set_gripper_position) {
            RCLCPP_INFO(this->get_logger(), "Setting gripper to position: %f", goal->gripper_position);
            success = move_group_helper_->setGripperPosition(goal->gripper_position);
            result->success = success;
        }
        
        // Set the result and return
        if (success) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        } else {
            goal_handle->abort(result);
        }

        return success;
    }

    bool checkRobotStatus(const std::shared_ptr<GoalHandleFullDrive> goal_handle, std::shared_ptr<FullDrive::Result> result) {
        RCLCPP_INFO(this->get_logger(), "Checking robot status...");
        while (isRobotMoving()) {
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled while checking robot status");
                return false;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        RCLCPP_INFO(this->get_logger(), "Robot has stopped moving.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return true;
    }

    bool isRobotMoving() {
        int checks = 5;
        double tolerance = 0.4e-4;

        for (int i = 0; i < checks; ++i) {
            auto target_q = rtde_receive_->getTargetQ();
            auto actual_q = rtde_receive_->getActualQ();

            RCLCPP_INFO(this->get_logger(), "Check %d - Target Q: ", i + 1);
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
                return true;
            }
        }
        return false;
    }

    void processTask(const master_project_msgs::msg::MoveGroupActionDetails& task_details) {
        RCLCPP_INFO(this->get_logger(), "Processing task for group: %s, action: %s", task_details.group_name.c_str(), task_details.action_name.c_str());

        std::vector<std::vector<double>> robot_path;
        double initial_gripper_position = -1;
        double final_gripper_position = -1;
        double finger_change = 0.1;

        // Check if waypoints are empty
        if (task_details.waypoints.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No waypoints to process.");
            return;
        }

        for (const auto& waypoint : task_details.waypoints) {
            std::vector<double> joint_positions;

            // Log the number of joints in the waypoint
            RCLCPP_INFO(this->get_logger(), "Processing waypoint with %zu joints", waypoint.joints.size());

            for (const auto& joint : waypoint.joints) {
                RCLCPP_INFO(this->get_logger(), "Joint name: %s, position: %f", joint.name.c_str(), joint.position);
                joint_positions.push_back(joint.position);
            }

            joint_positions.push_back(1.0);
            joint_positions.push_back(1.0);
            joint_positions.push_back(0.05); // Tolerance or some other parameter

            robot_path.push_back(joint_positions);
        }

        if (task_details.group_name == "gripper") {
            if (!task_details.waypoints.front().joints.empty()) {
                initial_gripper_position = task_details.waypoints.front().joints.back().position;
            }
            if (!task_details.waypoints.back().joints.empty()) {
                final_gripper_position = task_details.waypoints.back().joints.back().position;
            }

            RCLCPP_INFO(this->get_logger(), "Initial gripper position: %f, Final gripper position: %f", initial_gripper_position, final_gripper_position);

            if (abs(abs(initial_gripper_position) - abs(final_gripper_position)) > finger_change) {
                RCLCPP_INFO(this->get_logger(), "Moving gripper to position: %f", final_gripper_position);
                final_gripper_position = std::clamp(final_gripper_position, 0.0, 1.0);
                RCLCPP_INFO(this->get_logger(), "Moving gripper to position: %f", final_gripper_position);
                gripper_->move(final_gripper_position);
                RCLCPP_INFO(this->get_logger(), "Gripper moved to position: %f", final_gripper_position);
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Executing robot path with %zu waypoints", robot_path.size());
            rtde_control_->moveJ(robot_path, true);

            // Wait until the robot has stopped moving
            while (isRobotMoving()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                RCLCPP_INFO(this->get_logger(), "Robot is moving, sleeping 1 second");
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        RCLCPP_INFO(this->get_logger(), "Sleeping 50 milliseconds");

        // Keep checking if the robot is busy
        RCLCPP_INFO(this->get_logger(), "Checking if the robot is busy...");
        while (isRobotBusy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(this->get_logger(), "Robot is still busy, sleeping 1 second");
        }
        RCLCPP_INFO(this->get_logger(), "Robot is now idle. Task processing completed.");
    }



    bool isRobotBusy() {
        // Check if the robot is steady
        bool steady = rtde_control_->isSteady();
        RCLCPP_INFO(this->get_logger(), "Is robot steady: %d", steady);
        return !steady; // Robot is busy if not steady
    }

    double map_value(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
        return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
    }

    double map_finger_joint_to_gripper(double finger_joint_position) {
        double close_finger_joint = 0.69991196175;
        double open_finger_joint = 0.07397215645645;
        return map_value(finger_joint_position, open_finger_joint, close_finger_joint, 0.988235, 0.105882);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FullDriveActionServer>();
    
    // Create a multithreaded executor
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4); // 4 threads
    
    executor.add_node(node);
    executor.spin();    
    rclcpp::shutdown();
    return 0;
}
