#ifndef MOVE_H
#define MOVE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include "master_project_msgs/msg/move_group_action_details.hpp"  // Replace with actual package name
#include "master_project_msgs/msg/move_group_waypoint.hpp"
#include "master_project_msgs/msg/move_group_joint_state.hpp"

class MoveGroupHelper {
public:
    MoveGroupHelper(const rclcpp::Node::SharedPtr& node)
        : node_(node),
          arm_move_group_(node_, "ur5e_arm"), 
          gripper_move_group_(node_, "gripper"), 
          planning_scene_interface_() {
        task_details_publisher_ = node_->create_publisher<master_project_msgs::msg::MoveGroupActionDetails>("/task_details", 10);
    }

    bool moveToPose(const geometry_msgs::msg::PoseStamped& target_pose, bool constrain) {
        arm_move_group_.setPoseTarget(target_pose);
        arm_move_group_.setMaxVelocityScalingFactor(0.1);
        arm_move_group_.setMaxAccelerationScalingFactor(0.1);

        // Create path constraints
        moveit_msgs::msg::Constraints path_constraints;
        if (constrain) {
            moveit_msgs::msg::JointConstraint shoulder_constraint;
            shoulder_constraint.joint_name = "shoulder_lift_joint"; // Replace with the actual joint name if different
            shoulder_constraint.position = -M_PI_2; // -90 degrees in radians
            shoulder_constraint.tolerance_above = 0.5; // Allowable deviation above the target
            shoulder_constraint.tolerance_below = 0.5; // Allowable deviation below the target
            shoulder_constraint.weight = 1.0; // Priority of this constraint
            path_constraints.joint_constraints.push_back(shoulder_constraint);

            // Apply path constraints to the move group
            arm_move_group_.setPathConstraints(path_constraints);
        }

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (arm_move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Planning successful, executing...");
            arm_move_group_.execute(my_plan);
            publishTaskDetails("ur5e_arm", "move_to_pose", my_plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed");
        }
        // Clear path constraints after planning
        arm_move_group_.clearPathConstraints();

        return success;
    }

    bool moveLinear(const geometry_msgs::msg::PoseStamped& target_pose) {
        // Use Cartesian Paths for linear movement
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose.pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit_msgs::msg::RobotTrajectory trajectory;
        
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = arm_move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 0.9) {
            RCLCPP_ERROR(node_->get_logger(), "Cartesian path planning failed. Only %f of the path was computed", fraction);
            return false;
        }

        my_plan.trajectory_ = trajectory;

        bool success = (arm_move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Linear path planning successful, executing...");
            publishTaskDetails("ur5e_arm", "move_linear", my_plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Linear path planning failed");
        }
        return success;
    }

    bool attachObject(const std::string& object_id, const std::string& link) {
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.link_name = link;
        attached_object.object.id = object_id;
        attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        planning_scene_interface_.applyAttachedCollisionObject(attached_object);
        RCLCPP_INFO(node_->get_logger(), "Object %s attached to %s", object_id.c_str(), link.c_str());
        return true;
    }

    bool detachObject(const std::string& object_id, const std::string& link) {
        moveit_msgs::msg::AttachedCollisionObject detached_object;
        detached_object.link_name = link;
        detached_object.object.id = object_id;
        detached_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

        planning_scene_interface_.applyAttachedCollisionObject(detached_object);
        RCLCPP_INFO(node_->get_logger(), "Object %s detached from %s", object_id.c_str(), link.c_str());
        return true;
    }

    bool setGripperPosition(double gripper_position) {
        gripper_move_group_.setJointValueTarget("finger_joint", gripper_position);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (gripper_move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Planning successful, executing...");
            gripper_move_group_.execute(my_plan);
            publishTaskDetails("gripper", "set_gripper_position", my_plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed");
        }
        return success;
    }

    bool allowCollision(const std::string& object1, const std::string& object2, bool allow) {
        moveit_msgs::msg::PlanningScene planning_scene;
        planning_scene.is_diff = true;

        planning_scene.allowed_collision_matrix.entry_names.push_back(object1);
        planning_scene.allowed_collision_matrix.entry_names.push_back(object2);

        moveit_msgs::msg::AllowedCollisionEntry allowed_collision_entry;
        allowed_collision_entry.enabled.push_back(allow);
        planning_scene.allowed_collision_matrix.entry_values.push_back(allowed_collision_entry);

        planning_scene_interface_.applyPlanningScene(planning_scene);
        RCLCPP_INFO(node_->get_logger(), "Allowed collision between %s and %s: %d", object1.c_str(), object2.c_str(), allow);
        return true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface arm_move_group_;
    moveit::planning_interface::MoveGroupInterface gripper_move_group_;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Publisher<master_project_msgs::msg::MoveGroupActionDetails>::SharedPtr task_details_publisher_;

    void publishTaskDetails(const std::string& group_name, const std::string& action_name, const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        // Create and publish task details message
        master_project_msgs::msg::MoveGroupActionDetails action_details_msg;
        master_project_msgs::msg::MoveGroupWaypoint waypoint_msg;
        master_project_msgs::msg::MoveGroupJointState joint_state_msg;

        action_details_msg.group_name = group_name;
        action_details_msg.action_name = action_name;

        for (const auto& trajectory_point : plan.trajectory_.joint_trajectory.points) {
            waypoint_msg.joints.clear();  // Clear the joints vector for each new trajectory point

            for (size_t i = 0; i < trajectory_point.positions.size(); ++i) {
                joint_state_msg.name = plan.trajectory_.joint_trajectory.joint_names[i];
                joint_state_msg.position = trajectory_point.positions[i];
                waypoint_msg.joints.push_back(joint_state_msg);
            }

            waypoint_msg.speed = trajectory_point.velocities.empty() ? 0.0 : trajectory_point.velocities[0];
            waypoint_msg.acceleration = trajectory_point.accelerations.empty() ? 0.0 : trajectory_point.accelerations[0];
            action_details_msg.waypoints.push_back(waypoint_msg);

        }

        RCLCPP_INFO(node_->get_logger(), "Publishing MoveGroupActionDetails message: %s", action_name.c_str());
        task_details_publisher_->publish(action_details_msg);
    }
};

#endif // MOVE_H
