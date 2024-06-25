#ifndef MTC_H
#define MTC_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/container.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <master_project_msgs/msg/task.hpp>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace mtc = moveit::task_constructor;

class MTCStages {
public:
    MTCStages(const rclcpp::Node::SharedPtr& node)
        : node_(node),
          task_("task"),
          sampling_planner_(std::make_shared<mtc::solvers::PipelinePlanner>(node)),
          cartesian_planner_(std::make_shared<mtc::solvers::CartesianPath>()) {

        cartesian_planner_->setMaxVelocityScalingFactor(1.0);
        cartesian_planner_->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner_->setStepSize(.005);

        task_details_publisher_ = node_->create_publisher<master_project_msgs::msg::Task>("task_details_published", 10);
        
        // Additional initialization
        task_.reset();
        task_.stages()->clear();
        task_.stages()->setName("planned_task");
        task_.loadRobotModel(node_);

        const auto& arm_group_name = "ur5e_arm";
        const auto& hand_group_name = "gripper";
        const auto& hand_frame = "flange";

        // Set task properties
        task_.setProperty("group", arm_group_name);
        task_.setProperty("eef", hand_group_name);
        task_.setProperty("ik_frame", hand_frame);
    }

    void createCurrentStateStage(const std::string& name) {
        // note: maybe in the test phase, when the mtc will not work fine as expected then the additional initialization lines should come here
        // and the crateCurrentStateStage should called before any moving stage everytime the mtc needed, to make clear that the satges planned perfectly
        // or maybe each stage should use these things before continuing 
        auto current_state_stage = std::make_unique<mtc::stages::CurrentState>(name);
        task_.add(std::move(current_state_stage));
    }

    void createMoveToStage(const std::string& name, const std::string& group, const geometry_msgs::msg::PoseStamped& target) {
        auto move_to_stage = std::make_unique<mtc::stages::MoveTo>(name, sampling_planner_);
        move_to_stage->setGroup(group); // when this group name did not work, try to use the above variable name -> arm_group_name
        move_to_stage->setGoal(target);
        task_.add(std::move(move_to_stage));
    }

    void createMoveLinearStage(const std::string& name, const std::string& group, const geometry_msgs::msg::PoseStamped& target) {
        auto move_linear_stage = std::make_unique<mtc::stages::MoveTo>(name, cartesian_planner_);
        move_linear_stage->setGroup(group);
        move_linear_stage->setGoal(target);
        task_.add(std::move(move_linear_stage));
    }

    void createGripperStage(const std::string& name, double gripper_position) {
        auto gripper_stage = std::make_unique<mtc::stages::MoveTo>(name, sampling_planner_);
        gripper_stage->setGroup("gripper");

        // Define the target joint values for the gripper
        std::map<std::string, double> target_joint_values;
        target_joint_values["finger_joint"] = gripper_position;

        gripper_stage->setGoal(target_joint_values);
        task_.add(std::move(gripper_stage));
    }


    void createAttachObjectStage(const std::string& name, const std::string& target_object, const std::string& link) {
        auto attach_stage = std::make_unique<mtc::stages::ModifyPlanningScene>(name);
        attach_stage->attachObject(target_object, link);
        task_.add(std::move(attach_stage));
    }

    void createDetachObjectStage(const std::string& name, const std::string& target_object, const std::string& link) {
        auto detach_stage = std::make_unique<mtc::stages::ModifyPlanningScene>(name);
        detach_stage->detachObject(target_object, link);
        task_.add(std::move(detach_stage));
    }

    void createAllowCollisionStage(const std::string& name, const std::string& target_object, const std::vector<std::string>& objects, bool allow) {
        auto allow_collision_stage = std::make_unique<mtc::stages::ModifyPlanningScene>(name);
        allow_collision_stage->allowCollisions(target_object, objects, allow);
        task_.add(std::move(allow_collision_stage));
    }

    bool planAndExecute() {
        try {
            task_.init();
        } catch (const mtc::InitStageException& e) {
            RCLCPP_ERROR(node_->get_logger(), "Task initialization failed: %s", e.what());
            return false;
        }

        if (!task_.plan(10)) {
            RCLCPP_ERROR(node_->get_logger(), "Task planning failed");
            return false;
        }

        task_.introspection().publishSolution(*task_.solutions().front());
        publishTaskDetails(task_);

        auto result = task_.execute(*task_.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Task execution failed");
            return false;
        }

        return true;
    }

    void publishTaskDetails(const mtc::Task& task) {
        master_project_msgs::msg::Task task_msg;
        task_msg.name = task.name();

        const auto* container = task.stages();
        if (container) {
            container->traverseChildren([&](const mtc::Stage& stage, unsigned int /*depth*/) -> bool {
                master_project_msgs::msg::Stage stage_msg;
                stage_msg.name = stage.name();

                for (const auto& solution : stage.solutions()) {
                    const auto* sub_trajectory = dynamic_cast<const mtc::SubTrajectory*>(solution.get());
                    if (sub_trajectory && sub_trajectory->trajectory()) {
                        const auto& trajectory = *sub_trajectory->trajectory();
                        for (size_t j = 0; j < trajectory.getWayPointCount(); ++j) {
                            master_project_msgs::msg::Waypoint waypoint_msg;
                            const auto& waypoint = trajectory.getWayPoint(j);

                            for (const auto& joint : waypoint.getVariableNames()) {
                                master_project_msgs::msg::JointState joint_state_msg;
                                joint_state_msg.name = joint;
                                joint_state_msg.position = waypoint.getVariablePosition(joint);
                                waypoint_msg.joints.push_back(joint_state_msg);
                            }

                            stage_msg.waypoints.push_back(waypoint_msg);
                        }
                    }
                }

                task_msg.stages.push_back(stage_msg);
                return true; // Continue traversal
            });
        }

        task_msg.number_of_stages = task_msg.stages.size();
        RCLCPP_INFO(node_->get_logger(), "Publishing TaskDetails message: %s", task_msg.name.c_str());
        task_details_publisher_->publish(task_msg);
    }

private:
    rclcpp::Node::SharedPtr node_;
    mtc::Task task_;
    std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner_;
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;
    rclcpp::Publisher<master_project_msgs::msg::Task>::SharedPtr task_details_publisher_;
};

#endif // MTC_H
    
    /* void createExactHandStage(const std::string& name, const std::string& group, const geometry_msgs::msg::PoseStamped& target) {
        auto exact_hand_stage = std::make_unique<mtc::stages::MoveTo>(name, sampling_planner_);
        exact_hand_stage->setGroup(group);
        exact_hand_stage->setGoal(value);
        task_.add(std::move(exact_hand_stage));
    } */


    /* void createCloseHandStage(const std::string& name, const std::string& group, const std::string& state) {
        auto close_hand_stage = std::make_unique<mtc::stages::MoveTo>(name, sampling_planner_);
        close_hand_stage->setGroup(group);
        close_hand_stage->setGoal(state);
        task_.add(std::move(close_hand_stage));
    }

    void createOpenHandStage(const std::string& name, const std::string& group, const std::string& state) {
        auto open_hand_stage = std::make_unique<mtc::stages::MoveTo>(name, sampling_planner_);
        open_hand_stage->setGroup(group);
        open_hand_stage->setGoal("open");
        task_.add(std::move(open_hand_stage));*/