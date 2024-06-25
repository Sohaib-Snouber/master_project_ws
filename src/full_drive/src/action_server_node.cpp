#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_interfaces/action/full_drive.hpp"
#include "full_drive/mtc.h"
#include "full_drive/update_planning_scene.h"
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

        task_details_subscriber_ = this->create_subscription<master_project_msgs::msg::Task>(
            "task_details", 10, std::bind(&FullDriveActionServer::taskDetailsCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Connecting to gripper...");
        gripper_->connect();
        RCLCPP_INFO(this->get_logger(), "Gripper connected.");
        gripper_->activate();
        RCLCPP_INFO(this->get_logger(), "Gripper activated.");

        planning_scene_updater_ = std::make_shared<PlanningSceneUpdater>(this->shared_from_this());
        mtc_ = std::make_shared<MTCStages>(this->shared_from_this());
    }

private:
    rclcpp::Subscription<master_project_msgs::msg::Task>::SharedPtr task_details_subscriber_;
    rclcpp_action::Server<FullDrive>::SharedPtr action_server_;
    std::string ip_;
    std::unique_ptr<RTDEControlInterface> rtde_control_;
    std::unique_ptr<RTDEReceiveInterface> rtde_receive_;
    std::unique_ptr<RobotiqGripper> gripper_;
    master_project_msgs::msg::Task task_;
    std::shared_ptr<PlanningSceneUpdater> planning_scene_updater_;
    std::shared_ptr<MTCStages> mtc_;
    // Map to store objects and their properties
    std::unordered_map<std::string, ObjectProperties> object_map_;

    /* // example of Adding an object to the map
    object_map_["object1"] = {true, false, {"object2", "object3"}}; */

    void taskDetailsCallback(const master_project_msgs::msg::Task::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received task details: %s", msg->name.c_str());
        task_ = *msg;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FullDrive::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request");
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

    void execute(const std::shared_ptr<GoalHandleFullDrive> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<FullDrive::Result>();

        if (goal->add_collision_object) {
            if (object_map_.find(goal->target_name) != object_map_.end()) {
                RCLCPP_ERROR(this->get_logger(), "Collision object already exists: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Collision object already exists.";
                goal_handle->abort(result);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Adding collision object");
            planning_scene_updater_->addCollisionObject(goal->target_name, goal->object_primitive, goal->object_pose, goal->color);
            object_map_[goal->target_name] = {true, false, {}};
        } else if (goal->delete_collision_object) {
            if (object_map_.find(goal->target_name) == object_map_.end()) {
                RCLCPP_ERROR(this->get_logger(), "Collision object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Collision object does not exist.";
                goal_handle->abort(result);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Deleting collision object");
            planning_scene_updater_->removeCollisionObject(goal->target_name);
            object_map_.erase(goal->target_name);
        } else if (goal->attach_object) {
            if (object_map_.find(goal->target_name) == object_map_.end() || !object_map_[goal->target_name].exists) {
                RCLCPP_ERROR(this->get_logger(), "Object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Object does not exist.";
                goal_handle->abort(result);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Attaching object");
            mtc_->createAttachObjectStage("attach_object", goal->target_name, goal->link);
            if (!mtc_->planAndExecute()) {
                RCLCPP_ERROR(this->get_logger(), "Attach object failed");
                result->success = false;
                result->message = "Attach object failed.";
                goal_handle->abort(result);
                return;
            }
            object_map_[goal->target_name].exists = true;
        } else if (goal->detach_object) {
            if (object_map_.find(goal->target_name) == object_map_.end() || !object_map_[goal->target_name].exists) {
                RCLCPP_ERROR(this->get_logger(), "Object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Object does not exist.";
                goal_handle->abort(result);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Detaching object");
            mtc_->createDetachObjectStage("detach_object", goal->target_name, goal->link);
            if (!mtc_->planAndExecute()) {
                RCLCPP_ERROR(this->get_logger(), "Detach object failed");
                result->success = false;
                result->message = "Detach object failed.";
                goal_handle->abort(result);
                return;
            }
            object_map_[goal->target_name].exists = false;
        } else if (goal->move_to) {
            RCLCPP_INFO(this->get_logger(), "Moving to pose");
            mtc_->createMoveToStage("move_to", "arm", goal->target_pose);
            if (!mtc_->planAndExecute()) {
                RCLCPP_ERROR(this->get_logger(), "Move to pose failed");
                result->success = false;
                result->message = "Move to pose failed.";
                goal_handle->abort(result);
                return;
            } else {
                processTask(task_);
            }
        } else if (goal->move_linear) {
            RCLCPP_INFO(this->get_logger(), "Moving linearly to pose");
            mtc_->createMoveLinearStage("move_linear", "arm", goal->target_pose);
            if (!mtc_->planAndExecute()) {
                RCLCPP_ERROR(this->get_logger(), "Move linear failed");
                result->success = false;
                result->message = "Move linear failed.";
                goal_handle->abort(result);
                return;
            } else {
                processTask(task_);
            }
        } else if (goal->check_robot_status) {
            checkRobotStatus(goal_handle, result);
        } else if (goal->allow_collision) {
            if (object_map_.find(goal->target_name) == object_map_.end()) {
                RCLCPP_ERROR(this->get_logger(), "Object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Object does not exist.";
                goal_handle->abort(result);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Allowing collision");
            mtc_->createAllowCollisionStage("allow_collision", goal->target_name, {goal->object_name}, true);
            if (!mtc_->planAndExecute()) {
                RCLCPP_ERROR(this->get_logger(), "Allow collision failed");
                result->success = false;
                result->message = "Allow collision failed.";
                goal_handle->abort(result);
                return;
            }
            object_map_[goal->target_name].collision_allowed = true;
        } else if (goal->reenable_collision) {
            if (object_map_.find(goal->target_name) == object_map_.end()) {
                RCLCPP_ERROR(this->get_logger(), "Object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Object does not exist.";
                goal_handle->abort(result);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Re-enabling collision");
            mtc_->createAllowCollisionStage("reenable_collision", goal->target_name, {goal->object_name}, false);
            if (!mtc_->planAndExecute()) {
                RCLCPP_ERROR(this->get_logger(), "Re-enable collision failed");
                result->success = false;
                result->message = "Re-enable collision failed.";
                goal_handle->abort(result);
                return;
            }
            object_map_[goal->target_name].collision_allowed = false;
        } else if (goal->current_state) {
            RCLCPP_INFO(this->get_logger(), "getting current state");
            mtc_->createCurrentStateStage("current_state");
            if (!mtc_->planAndExecute()) {
                RCLCPP_ERROR(this->get_logger(), "getting current state failed");
                result->success = false;
                result->message = "getting current state failed.";
                goal_handle->abort(result);
                return;
            } else {
                processTask(task_);
            }
        } else if (goal->set_gripper_position) {
            RCLCPP_INFO(this->get_logger(), "Setting gripper to position: %f", goal->gripper_position);
            mtc_->createGripperStage("set_gripper", goal->gripper_position);
            if (!mtc_->planAndExecute()) {
                RCLCPP_ERROR(this->get_logger(), "Set gripper position failed");
                result->success = false;
                result->message = "Set gripper position failed.";
                goal_handle->abort(result);
                return;
            } else {
                processTask(task_);
            }
        }
        
        if (rclcpp::ok()) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

    void checkRobotStatus(const std::shared_ptr<GoalHandleFullDrive> goal_handle, std::shared_ptr<FullDrive::Result> result) {
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

    bool isRobotMoving() {
        int checks = 5;
        double tolerance = 1e-4;

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

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        return false;
    }

    void processTask(const master_project_msgs::msg::Task& task) {
        for (const auto& stage : task.stages) {
            RCLCPP_INFO(this->get_logger(), "Executing stage: %s", stage.name.c_str());

            RCLCPP_INFO(this->get_logger(), "Number of waypoints in stage %s: %zu", stage.name.c_str(), stage.waypoints.size());
            if (stage.waypoints.size() == 0) {
                RCLCPP_INFO(this->get_logger(), "Stage %s has no waypoints. Skipping...", stage.name.c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            std::vector<std::vector<double>> robot_path;
            double initial_gripper_position = -1;
            double final_gripper_position = -1;
            double finger_change = 0.1;

            for (size_t i = 0; i < stage.waypoints.size(); ++i) {
                const auto& waypoint = stage.waypoints[i];
                std::vector<double> joint_positions(6, 0.0);

                size_t joint_index = 0;
                for (const auto& joint : waypoint.joints) {
                    if (joint_index < 6) {
                        joint_positions[joint_index] = joint.position;
                    } else if (joint_index == 6) {
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
                one_pos.push_back(1.0);
                one_pos.push_back(1.2);
                one_pos.push_back(0.1);
                robot_path.push_back(one_pos);
            }

            RCLCPP_INFO(this->get_logger(), "Initial gripper position: %f", initial_gripper_position);
            RCLCPP_INFO(this->get_logger(), "Final gripper position: %f", final_gripper_position);

            if (abs(abs(initial_gripper_position) - abs(final_gripper_position)) > finger_change) {
                RCLCPP_INFO(this->get_logger(), "Moving gripper to position: %f", final_gripper_position);
                final_gripper_position = std::clamp(final_gripper_position, 0.0, 1.0);
                RCLCPP_INFO(this->get_logger(), "Moving gripper to position: %f", final_gripper_position);
                gripper_->move(final_gripper_position);
                RCLCPP_INFO(this->get_logger(), "Gripper moved to position: %f", final_gripper_position);
            } else {
                rtde_control_->moveJ(robot_path, true);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
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
        gripper_->move(1.0);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    void gripperClose() {
        RCLCPP_INFO(this->get_logger(), "Closing gripper");
        gripper_->move(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FullDriveActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
