#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_interfaces/action/full_drive.hpp"
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
#include "master_project_msgs/msg/move_group_action_details.hpp"
#include <mutex>

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
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/introspection.h>
#include <moveit/task_constructor/stages/current_state.h>


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
        : Node("full_drive_action_server", options), ip_("10.130.1.100"),
        planning_scene_interface_(std::make_shared<moveit::planning_interface::PlanningSceneInterface>()){
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
        task_details_publisher_ = this->create_publisher<master_project_msgs::msg::MoveGroupActionDetails>("/task_details", 10);

        RCLCPP_INFO(this->get_logger(), "Connecting to gripper...");
        gripper_->connect();
        RCLCPP_INFO(this->get_logger(), "Gripper connected.");
        gripper_->activate();
        RCLCPP_INFO(this->get_logger(), "Gripper activated.");

        // Defer the initialization of shared_from_this()
        auto timer_callback = [this]() {
            planning_scene_updater_ = std::make_shared<PlanningSceneUpdater>(this->shared_from_this());
            RCLCPP_INFO(this->get_logger(), "Initialized the PlanninSceneUpdater class");
            //move_group_helper_ = std::make_shared<MoveGroupHelper>(this->shared_from_this());
            RCLCPP_INFO(this->get_logger(), "Initialized the MoveGroupHelper class");
            // Initialize MoveGroupInterface instances
            arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur5e_arm");
            gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
            
            robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this(), "robot_description");
            moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();
            planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model);

            // Initialize planning scene
            setupPlanningScene();
            loadExistingCollisionObjects();

            initialization_timer_->cancel(); // Cancel the timer after initialization
        };
        // Use a timer to defer the callback
        initialization_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), timer_callback);
    }

private:
    rclcpp::Subscription<master_project_msgs::msg::MoveGroupActionDetails>::SharedPtr task_details_subscriber_;
    rclcpp::Publisher<master_project_msgs::msg::MoveGroupActionDetails>::SharedPtr task_details_publisher_;
    rclcpp_action::Server<FullDrive>::SharedPtr action_server_;
    std::string ip_;
    std::unique_ptr<RTDEControlInterface> rtde_control_;
    std::unique_ptr<RTDEReceiveInterface> rtde_receive_;
    std::unique_ptr<RobotiqGripper> gripper_;
    std::shared_ptr<PlanningSceneUpdater> planning_scene_updater_;

    // Map to store objects and their properties
    std::unordered_map<std::string, ObjectProperties> object_map_;
    rclcpp::TimerBase::SharedPtr initialization_timer_;  // Store the timer to prevent it from being destroyed
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::mutex task_mutex_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene_;


    void setupPlanningScene(){
        // Create collision objects
        shape_msgs::msg::SolidPrimitive primitive_wall1;
        primitive_wall1.type = primitive_wall1.BOX;
        primitive_wall1.dimensions = {0.1, 1.4, 0.5};
        geometry_msgs::msg::Pose wall1_pose;
        wall1_pose.orientation.w = 1.0;
        wall1_pose.position.x = 0.8;
        wall1_pose.position.y = 0.2;
        wall1_pose.position.z = 0.25;
        moveit_msgs::msg::CollisionObject collision_object_wall1 = createCollisionObject("wall1", primitive_wall1, wall1_pose);
        
        shape_msgs::msg::SolidPrimitive primitive_wall2;
        primitive_wall2.type = primitive_wall2.BOX;
        primitive_wall2.dimensions = {1.4, 0.1, 0.5};
        geometry_msgs::msg::Pose wall2_pose;
        wall2_pose.orientation.w = 1.0;
        wall2_pose.position.x = 0.2;
        wall2_pose.position.y = 0.8;
        wall2_pose.position.z = 0.25;
        moveit_msgs::msg::CollisionObject collision_object_wall2 = createCollisionObject("wall2", primitive_wall2, wall2_pose);

        shape_msgs::msg::SolidPrimitive primitive_wall3;
        primitive_wall3.type = primitive_wall3.BOX;
        primitive_wall3.dimensions = {0.1, 1.4, 0.5};
        geometry_msgs::msg::Pose wall3_pose;
        wall3_pose.orientation.w = 1.0;
        wall3_pose.position.x = -0.6;
        wall3_pose.position.y = 0.2;
        wall3_pose.position.z = 0.25;
        moveit_msgs::msg::CollisionObject collision_object_wall3 = createCollisionObject("wall3", primitive_wall3, wall3_pose);
        
        shape_msgs::msg::SolidPrimitive primitive_wall4;
        primitive_wall4.type = primitive_wall4.BOX;
        primitive_wall4.dimensions = {1.4, 0.1, 0.5};
        geometry_msgs::msg::Pose wall4_pose;
        wall4_pose.orientation.w = 1.0;
        wall4_pose.position.x = 0.2;
        wall4_pose.position.y = -0.6;
        wall4_pose.position.z = 0.25;
        moveit_msgs::msg::CollisionObject collision_object_wall4 = createCollisionObject("wall4", primitive_wall4, wall4_pose);

        /* shape_msgs::msg::SolidPrimitive primitive_table2;
        primitive_table2.type = primitive_table2.BOX;
        primitive_table2.dimensions = {0.6, 0.9, 0.2};
        geometry_msgs::msg::Pose table2_pose;
        table2_pose.orientation.w = 1.0;
        table2_pose.position.x = -0.60;
        table2_pose.position.y = 0.0;
        table2_pose.position.z = 0.1;
        moveit_msgs::msg::CollisionObject collision_object_table2 = createCollisionObject("table2", primitive_table2, table2_pose); */


        /* shape_msgs::msg::SolidPrimitive primitive_target1;
        primitive_target1.type = primitive_target1.CYLINDER;
        primitive_target1.dimensions = {0.1, 0.02}; // height, radius
        geometry_msgs::msg::Pose target1_pose;
        target1_pose.orientation.w = 1.0;
        target1_pose.position.x = 0.45;
        target1_pose.position.y = -0.35;
        target1_pose.position.z = 0.25;
        moveit_msgs::msg::CollisionObject collision_object_target1 = createCollisionObject("target1", primitive_target1, target1_pose);
        */
        shape_msgs::msg::SolidPrimitive primitive_surface;
        primitive_surface.type = primitive_surface.BOX;
        primitive_surface.dimensions = {0.75, 0.8, 0.1};
        geometry_msgs::msg::Pose surface_pose;
        surface_pose.orientation.w = 1.0;
        surface_pose.position.x = 0.16;
        surface_pose.position.y = 0.25;
        surface_pose.position.z = -0.05;
        moveit_msgs::msg::CollisionObject collision_object_surface = createCollisionObject("surface", primitive_surface, surface_pose);

        /* // Create object colors
        moveit_msgs::msg::ObjectColor table1_color = createObjectColor("table1", 1.0, 0.5, 0.5, 1.0);
        moveit_msgs::msg::ObjectColor table2_color = createObjectColor("table2", 1.0, 0.5, 0.5, 1.0);
        moveit_msgs::msg::ObjectColor target1_color = createObjectColor("target1", 0.5, 0.0, 1.0, 1.0); */
        moveit_msgs::msg::ObjectColor surface_color = createObjectColor("surface", 0.5, 0.5, 0.5, 1.0);
        moveit_msgs::msg::ObjectColor wall1_color = createObjectColor("wall1", 0.1, 0.2, 0.3, 1.0); // -y axis
        moveit_msgs::msg::ObjectColor wall2_color = createObjectColor("wall2", 1.0, 1.0, 1.0, 1.0); // +x axis
        moveit_msgs::msg::ObjectColor wall3_color = createObjectColor("wall3", 1.0, 1.0, 1.0, 1.0); // +y axis
        moveit_msgs::msg::ObjectColor wall4_color = createObjectColor("wall4", 1.0, 1.0, 1.0, 1.0); //- x axis


        // Create a PlanningScene message and add the collision objects and their colors
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        planning_scene_msg.world.collision_objects = {collision_object_surface, collision_object_wall1,collision_object_wall2,collision_object_wall3,collision_object_wall4};
        planning_scene_msg.object_colors = {surface_color, wall1_color, wall2_color, wall3_color, wall4_color};
        /* planning_scene_msg.world.collision_objects = {collision_object_table1, collision_object_table2, collision_object_target1, collision_object_surface};
        planning_scene_msg.object_colors = {table1_color, table2_color, target1_color, surface_color}; */

        // Apply the planning scene
        planning_scene_interface_->applyPlanningScene(planning_scene_msg);

        // Modify the ACM to allow collision between base_link_inertia and surface
        collision_detection::AllowedCollisionMatrix& acm = planning_scene_->getAllowedCollisionMatrixNonConst();
        acm.setEntry("base_link_inertia", "surface", true);

        // Update the planning scene with the modified ACM
        moveit_msgs::msg::AllowedCollisionMatrix acm_msg;
        acm.getMessage(acm_msg);
        planning_scene_msg.allowed_collision_matrix = acm_msg;
        planning_scene_interface_->applyPlanningScene(planning_scene_msg);

        RCLCPP_INFO(this->get_logger(), "Allowed collision between base_link_inertia and surface.");

        RCLCPP_INFO(this->get_logger(), "Planning scene setup complete");
    }

    moveit_msgs::msg::CollisionObject createCollisionObject(
        const std::string& id, const shape_msgs::msg::SolidPrimitive& primitive, const geometry_msgs::msg::Pose& pose)
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "world";
        collision_object.id = id;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;
        return collision_object;
    }

    moveit_msgs::msg::ObjectColor createObjectColor(
        const std::string& id, float r, float g, float b, float a)
    {
        moveit_msgs::msg::ObjectColor color;
        color.id = id;
        color.color.r = r;
        color.color.g = g;
        color.color.b = b;
        color.color.a = a;
        return color;
    }

    /* // example of Adding an object to the map
    object_map_["object1"] = {true, false, {"object2", "object3"}}; */

    void loadExistingCollisionObjects() {
        auto collision_objects = planning_scene_interface_->getObjects();

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
        std::cout << "precise_motion: " << goal->precise_motion << std::endl;


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
            success = attachObject(goal->target_name, goal->link);
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
            success = detachObject(goal->target_name, goal->link);
            result->success = success;

        } else if (goal->move_to) {
            RCLCPP_INFO(this->get_logger(), "Moving to pose");
            success = moveToPose(goal->target_pose, goal->constrain, goal->precise_motion);
            result->success = success;

        } else if (goal->move_linear) {
            RCLCPP_INFO(this->get_logger(), "Moving linearly to pose");
            success = moveLinear(goal->target_pose);
            result->success = success;

        } else if (goal->set_gripper_position) {
            RCLCPP_INFO(this->get_logger(), "Setting gripper to position: %f", goal->gripper_position);
            success = setGripperPosition(goal->gripper_position);
            result->success = success;

        }else if (goal->check_robot_status) {
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
            success = allowCollision(goal->target_name, goal->object_name, true);
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
            success = allowCollision(goal->target_name, goal->object_name, false);
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

    bool attachObject(const std::string& object_id, const std::string& link) {
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.link_name = link;
        attached_object.object.id = object_id;
        attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        planning_scene_interface_->applyAttachedCollisionObject(attached_object);
        RCLCPP_INFO(this->get_logger(), "Object %s attached to %s", object_id.c_str(), link.c_str());
        return true;
    }

    bool allowCollision(const std::string& object1, const std::string& object2, bool allow) {
        using namespace moveit::task_constructor;

        // Create a Task to hold the stage
        Task task;
        task.stages()->setName("demo task");

        try {
            task.loadRobotModel(shared_from_this());  // Use the current node to load the robot model
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading robot model: %s", e.what());
            return false;
        }

        const auto& arm_group_name = "ur5e_arm";
        const auto& hand_group_name = "gripper";
        const auto& hand_frame = "flange";

        // Set task properties
        task.setProperty("group", arm_group_name);
        task.setProperty("eef", hand_group_name);
        task.setProperty("ik_frame", hand_frame);

        // Current state
        auto stage_state_current = std::make_unique<stages::CurrentState>("current");
        task.add(std::move(stage_state_current));


        try {
            if (object1 == "hand") {
                // Handle the special case where object1 is "hand"
                auto allow_collision_stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand, target1)");
                allow_collision_stage->allowCollisions(object2,
                                                    task.getRobotModel()
                                                        ->getJointModelGroup(hand_group_name)
                                                        ->getLinkModelNamesWithCollisionGeometry(),
                                                    allow);
                task.add(std::move(allow_collision_stage));
            } else {
                // General case
                auto allow_collision_stage = std::make_unique<stages::ModifyPlanningScene>("allow collision");
                allow_collision_stage->allowCollisions(object1, {object2}, allow);
                task.add(std::move(allow_collision_stage));
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error creating modify planning scene stage: %s", e.what());
            return false;
        }

        // Execute the task
        try {
            task.plan(0);  // Plan with the first solution
            if (!task.solutions().empty()) {
                const auto& solution = task.solutions().front();
                if (task.execute(*solution) == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "Allowed collision between %s and %s: %d", object1.c_str(), object2.c_str(), allow);
                    return true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to execute task to allow collision between %s and %s", object1.c_str(), object2.c_str());
                    return false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "No valid solutions found for allowing collision between %s and %s", object1.c_str(), object2.c_str());
                return false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception caught while modifying planning scene: %s", e.what());
            return false;
        }
    }




    bool detachObject(const std::string& object_id, const std::string& link) {
        moveit_msgs::msg::AttachedCollisionObject detached_object;
        detached_object.link_name = link;
        detached_object.object.id = object_id;
        detached_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

        planning_scene_interface_->applyAttachedCollisionObject(detached_object);
        RCLCPP_INFO(this->get_logger(), "Object %s detached from %s", object_id.c_str(), link.c_str());
        return true;
    }

    bool moveToPose(const geometry_msgs::msg::PoseStamped& target_pose, bool constrain, bool precise_motion) {
        arm_move_group_->setPoseTarget(target_pose);
        arm_move_group_->setMaxVelocityScalingFactor(0.1);
        arm_move_group_->setMaxAccelerationScalingFactor(0.1);

        moveit_msgs::msg::Constraints path_constraints;
        if (constrain) {
            moveit_msgs::msg::JointConstraint shoulder_constraint;
            shoulder_constraint.joint_name = "shoulder_lift_joint";
            shoulder_constraint.position = -M_PI_2;
            shoulder_constraint.tolerance_above = 0.5;
            shoulder_constraint.tolerance_below = 0.5;
            shoulder_constraint.weight = 1.0;
            path_constraints.joint_constraints.push_back(shoulder_constraint);

            arm_move_group_->setPathConstraints(path_constraints);
        }

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (arm_move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
            arm_move_group_->execute(my_plan);
            publishTaskDetails("ur5e_arm", "move_to_pose", my_plan, precise_motion);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
        }
        arm_move_group_->clearPathConstraints();

        return success;
    }

    bool moveLinear(const geometry_msgs::msg::PoseStamped& target_pose) {

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose.pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = arm_move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 0.9) {
            RCLCPP_ERROR(this->get_logger(), "Cartesian path planning failed. Only %f of the path was computed", fraction);
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_ = trajectory;

        bool success = (arm_move_group_->execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Linear path planning and execution successful");
            publishTaskDetails("ur5e_arm", "move_linear", my_plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Linear path execution failed");
        }
        return success;
    }

    bool setGripperPosition(double gripper_position) {
        printGripperJointLimits();
        gripper_move_group_->setJointValueTarget("finger_joint", gripper_position);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        gripper_move_group_->setPlanningTime(10.0);

        bool success = (gripper_move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
            gripper_move_group_->execute(my_plan);
            publishTaskDetails("gripper", "set_gripper_position", my_plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
        }
        return success;
    }

    void printGripperJointLimits() {
        const moveit::core::JointModel* joint_model = gripper_move_group_->getRobotModel()->getJointModel("finger_joint");
        const moveit::core::VariableBounds& bounds = joint_model->getVariableBounds()[0];
        RCLCPP_INFO(this->get_logger(), "Finger joint limits: min_position = %f, max_position = %f", bounds.min_position_, bounds.max_position_);
    }

    void publishTaskDetails(const std::string& group_name, const std::string& action_name, const moveit::planning_interface::MoveGroupInterface::Plan& plan, bool precise_motion = true) {
        master_project_msgs::msg::MoveGroupActionDetails action_details_msg;
        master_project_msgs::msg::MoveGroupWaypoint waypoint_msg;
        master_project_msgs::msg::MoveGroupJointState joint_state_msg;

        action_details_msg.group_name = group_name;
        action_details_msg.action_name = action_name;
        action_details_msg.precise_motion = precise_motion;

        for (const auto& trajectory_point : plan.trajectory_.joint_trajectory.points) {
            waypoint_msg.joints.clear();

            for (size_t i = 0; i < trajectory_point.positions.size(); ++i) {
                joint_state_msg.name = plan.trajectory_.joint_trajectory.joint_names[i];
                joint_state_msg.position = trajectory_point.positions[i];
                waypoint_msg.joints.push_back(joint_state_msg);
            }

            waypoint_msg.speed = trajectory_point.velocities.empty() ? 0.0 : trajectory_point.velocities[0];
            waypoint_msg.acceleration = trajectory_point.accelerations.empty() ? 0.0 : trajectory_point.accelerations[0];
            action_details_msg.waypoints.push_back(waypoint_msg);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing MoveGroupActionDetails message: %s", action_name.c_str());
        task_details_publisher_->publish(action_details_msg);
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
        std::lock_guard<std::mutex> lock(task_mutex_);  // Lock the mutex at the beginning
        RCLCPP_INFO(this->get_logger(), "Processing task for group: %s, action: %s", task_details.group_name.c_str(), task_details.action_name.c_str());

        std::vector<std::vector<double>> robot_path;
        double initial_gripper_position = -1;
        double final_gripper_position = -1;
        double finger_change = 0.005;

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

            if(task_details.precise_motion == true){
                if (task_details.group_name == "ur5e_arm"){               
                    rtde_control_->moveJ(joint_positions, 1.0, 1.0, false);

                } else if (task_details.group_name == "gripper") {
                    if (!task_details.waypoints.front().joints.empty()) {
                        initial_gripper_position = task_details.waypoints.front().joints.back().position;
                    }
                    if (!task_details.waypoints.back().joints.empty()) {
                        final_gripper_position = task_details.waypoints.back().joints.back().position;
                    }

                    RCLCPP_INFO(this->get_logger(), "Initial gripper position: %f, Final gripper position: %f", initial_gripper_position, final_gripper_position);

                    if (abs(abs(initial_gripper_position) - abs(final_gripper_position)) > finger_change) {
                        RCLCPP_INFO(this->get_logger(), "Moving gripper to position: %f", final_gripper_position);
                        final_gripper_position = map_finger_joint_to_gripper(final_gripper_position);
                        final_gripper_position = std::clamp(final_gripper_position, 0.0, 1.0);
                        gripper_->move(final_gripper_position);
                        RCLCPP_INFO(this->get_logger(), "Gripper moved to position: %f", final_gripper_position);
                        break;
                    }
                }
            }
            joint_positions.push_back(1.0);
            joint_positions.push_back(1.0);
            joint_positions.push_back(0.05); // Tolerance or some other parameter

            robot_path.push_back(joint_positions);

        }
        if(task_details.precise_motion == false){
            if (task_details.group_name == "ur5e_arm"){               
                rtde_control_->moveJ(robot_path, false);

            } else if (task_details.group_name == "gripper") {
                if (!task_details.waypoints.front().joints.empty()) {
                    initial_gripper_position = task_details.waypoints.front().joints.back().position;
                }
                if (!task_details.waypoints.back().joints.empty()) {
                    final_gripper_position = task_details.waypoints.back().joints.back().position;
                }

                RCLCPP_INFO(this->get_logger(), "Initial gripper position: %f, Final gripper position: %f", initial_gripper_position, final_gripper_position);

                if (abs(abs(initial_gripper_position) - abs(final_gripper_position)) > finger_change) {
                    RCLCPP_INFO(this->get_logger(), "Moving gripper to position: %f", final_gripper_position);
                    final_gripper_position = map_finger_joint_to_gripper(final_gripper_position);
                    final_gripper_position = std::clamp(final_gripper_position, 0.0, 1.0);
                    gripper_->move(final_gripper_position);
                    RCLCPP_INFO(this->get_logger(), "Gripper moved to position: %f", final_gripper_position);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
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
