#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <master_project_msgs/msg/task.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "service_interfaces/srv/moveit_service.hpp"
#include <moveit/planning_scene/planning_scene.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <moveit_msgs/msg/allowed_collision_matrix.h>
#include <moveit_msgs/msg/allowed_collision_entry.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_logger");

namespace mtc = moveit::task_constructor;

class MTCNode : public rclcpp::Node {
public:
    
    MTCNode(const rclcpp::NodeOptions& options);
    void setup();

private:
    void handle_service(const std::shared_ptr<service_interfaces::srv::MoveitService::Request> request,
                    std::shared_ptr<service_interfaces::srv::MoveitService::Response> response);
    void createMoveToStage(const std::string& name, const std::string& group, const geometry_msgs::msg::PoseStamped& target, bool constrain);
    void createMoveLinearStage(const std::string& name, const std::string& group, const geometry_msgs::msg::PoseStamped& target);
    void createGripperStage(const std::string& name, const std::string& group, double gripper_position);
    void createAttachObjectStage(const std::string& name, const std::string& target_object, const std::string& link);
    void createDetachObjectStage(const std::string& name, const std::string& target_object, const std::string& link);
    void createAllowCollisionStage(const std::string& name, const std::string& target_object, const std::vector<std::string>& objects, bool allow);
    
    bool planAndExecute(mtc::Task& task_);
    void publishTaskDetails(const mtc::Task& task);

    std::shared_ptr<mtc::Task> task_;
    rclcpp::Service<service_interfaces::srv::MoveitService>::SharedPtr moveit_service_;
    rclcpp::Publisher<master_project_msgs::msg::Task>::SharedPtr task_details_publisher_;
    moveit::core::RobotModelPtr robot_model_;  // Add this line
    planning_scene::PlanningScenePtr planning_scene_; // Add this line
    rclcpp::TimerBase::SharedPtr initialization_timer_;  
};

MTCNode::MTCNode(const rclcpp::NodeOptions& options) 
  : Node("mtc_node", options)
{
    // Initialize the publisher
    task_details_publisher_ = this->create_publisher<master_project_msgs::msg::Task>("/task_details", 10);
    moveit_service_ = this->create_service<service_interfaces::srv::MoveitService>(
        "moveit_service", std::bind(&MTCNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));        
    
    // Defer the initialization of shared_from_this()
    auto timer_callback = [this]() {
        setup();
        initialization_timer_->cancel(); // Cancel the timer after initialization
    };
    // Use a timer to defer the callback
    initialization_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), timer_callback);
}

void MTCNode::setup()
{
    // Load the robot model
    robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description");
    robot_model_ = robot_model_loader.getModel();
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

    RCLCPP_INFO(this->get_logger(), "Robot model loaded.");
    // Create a PlanningSceneInterface to ensure it can access the planning scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Optionally wait for the planning scene to be initialized
    rclcpp::sleep_for(std::chrono::seconds(1));
    // Create a PlanningScene message 
    moveit_msgs::msg::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = true;

    // Apply the planning scene
    planning_scene_interface.applyPlanningScene(planning_scene_msg);

    // Modify the ACM to allow collision between base_link_inertia and surface
    collision_detection::AllowedCollisionMatrix& acm = planning_scene_->getAllowedCollisionMatrixNonConst();
    acm.setEntry("base_link_inertia", "surface", true);

    // Update the planning scene with the modified ACM
    moveit_msgs::msg::AllowedCollisionMatrix acm_msg;
    acm.getMessage(acm_msg);
    planning_scene_msg.allowed_collision_matrix = acm_msg;
    planning_scene_interface.applyPlanningScene(planning_scene_msg);

    RCLCPP_INFO(LOGGER, "allow collision between base_link_inertia and surface.");
}
    

void MTCNode::handle_service(const std::shared_ptr<service_interfaces::srv::MoveitService::Request> request,
                    std::shared_ptr<service_interfaces::srv::MoveitService::Response> response) {
    // Reset the task and robot model before processing
    task_ = std::make_shared<mtc::Task>("task");
    // Ensure the robot model is set before adding any stages
    if (!robot_model_) {
        RCLCPP_ERROR(this->get_logger(), "Robot model is not loaded.");
        response->success = false;
        response->message = "Robot model is not loaded.";
        return;
    }
    task_->setRobotModel(robot_model_);  // Set the robot model for the task

    if (request->function_name == "createMoveToStage") {
        createMoveToStage(request->name, request->group, request->target, request->constrain);
    } else if (request->function_name == "createMoveLinearStage") {
        createMoveLinearStage(request->name, request->group, request->target);
    } else if (request->function_name == "createGripperStage") {
        createGripperStage(request->name, request->group, request->gripper_position);
    } else if (request->function_name == "createAttachObjectStage") {
        createAttachObjectStage(request->name, request->target_object, request->link);
    } else if (request->function_name == "createDetachObjectStage") {
        createDetachObjectStage(request->name, request->target_object, request->link);
    } else if (request->function_name == "createAllowCollisionStage") {
        createAllowCollisionStage(request->name, request->target_object, request->objects, request->allow);
    } else {
        response->success = false;
        response->message = "Unknown function name";
        return;
    }
    RCLCPP_INFO(this->get_logger(), "service message stage done");

    response->success = planAndExecute(*task_);
    RCLCPP_INFO(this->get_logger(), "service message stage planned and executed");
    response->message = response->success ? "Success" : "Failed to execute stage";

    // clear the task after execution
    task_.reset();
}

void MTCNode::createMoveToStage(const std::string& name, const std::string& group, const geometry_msgs::msg::PoseStamped& target, bool constrain) {
    RCLCPP_INFO(this->get_logger(), "Service message is processing");

    task_->loadRobotModel(shared_from_this());
    task_->stages()->setName("demo task");

    const auto& arm_group_name = "ur5e_arm";
    const auto& hand_group_name = "gripper";
    const auto& hand_frame = "flange";

    // Set task properties
    task_->setProperty("group", arm_group_name);
    task_->setProperty("eef", hand_group_name);
    task_->setProperty("ik_frame", hand_frame);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    // Current state
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    task_->add(std::move(stage_state_current));        
    
    moveit_msgs::msg::Constraints path_constraints;

    // Optionally define and add joint constraint for the shoulder joint
    if (constrain) {
        moveit_msgs::msg::JointConstraint shoulder_constraint;
        shoulder_constraint.joint_name = "shoulder_lift_joint"; // shoulder_lift_joint, shoulder_pan_joint Replace with the actual joint name
        shoulder_constraint.position = -M_PI_2; // -90 degrees in radians
        shoulder_constraint.tolerance_above = 0.5; // Allowable deviation above the target
        shoulder_constraint.tolerance_below = 0.5; // Allowable deviation below the target
        shoulder_constraint.weight = 1.0; // Priority of this constraint
        path_constraints.joint_constraints.push_back(shoulder_constraint);
    }

    auto move_to_stage = std::make_unique<mtc::stages::MoveTo>(name, sampling_planner);
    move_to_stage->setGroup(group);
    move_to_stage->setGoal(target);

    if (constrain) {
        move_to_stage->setPathConstraints(path_constraints); // Set path constraints if applicable
    }

    task_->add(std::move(move_to_stage));
}
/* void createMoveToStage(const std::string& name, const std::string& group, const geometry_msgs::msg::PoseStamped& target) {
    RCLCPP_INFO(this->get_logger(), "service message is processesing");
    auto [sampling_planner, cartesian_planner, task_] = initializeTask();
    
    auto move_to_stage = std::make_unique<mtc::stages::MoveTo>(name, sampling_planner);
    move_to_stage->setGroup(group);
    move_to_stage->setGoal(target);
    task_->add(std::move(move_to_stage));
} */

void MTCNode::createMoveLinearStage(const std::string& name, const std::string& group, const geometry_msgs::msg::PoseStamped& target) {
    RCLCPP_INFO(this->get_logger(), "Service message is processing");

    task_->loadRobotModel(shared_from_this());
    task_->stages()->setName("demo task");


    const auto& arm_group_name = "ur5e_arm";
    const auto& hand_group_name = "gripper";
    const auto& hand_frame = "flange";

    // Set task properties
    task_->setProperty("group", arm_group_name);
    task_->setProperty("eef", hand_group_name);
    task_->setProperty("ik_frame", hand_frame);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    //cartesian_planner->setProperty("min_fraction", 0.9); // Set to 90% or any value you need

    // Current state
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    task_->add(std::move(stage_state_current));   

    auto move_linear_stage = std::make_unique<mtc::stages::MoveTo>(name, cartesian_planner);
    move_linear_stage->setGroup(group);
    move_linear_stage->setTimeout(5.0); // Set a timeout if needed
    move_linear_stage->setGoal(target);
    task_->add(std::move(move_linear_stage));
}

void MTCNode::createGripperStage(const std::string& name, const std::string& group, double gripper_position) {
    RCLCPP_INFO(this->get_logger(), "Service message is processing");

    task_->loadRobotModel(shared_from_this());
    task_->stages()->setName("demo task");

    const auto& arm_group_name = "ur5e_arm";
    const auto& hand_group_name = "gripper";
    const auto& hand_frame = "flange";

    // Set task properties
    task_->setProperty("group", arm_group_name);
    task_->setProperty("eef", hand_group_name);
    task_->setProperty("ik_frame", hand_frame);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.001);

    // Current state
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    task_->add(std::move(stage_state_current));        
    auto gripper_stage = std::make_unique<mtc::stages::MoveTo>(name, sampling_planner);
    gripper_stage->setGroup(group);

    // Define the target joint values for the gripper
    std::map<std::string, double> target_joint_values;
    target_joint_values["finger_joint"] = gripper_position;

    gripper_stage->setGoal(target_joint_values);
    task_->add(std::move(gripper_stage));
}

void MTCNode::createAttachObjectStage(const std::string& name, const std::string& target_object, const std::string& link) {
    RCLCPP_INFO(this->get_logger(), "Service message is processing");

    task_->loadRobotModel(shared_from_this());
    task_->stages()->setName("demo task");

    const auto& arm_group_name = "ur5e_arm";
    const auto& hand_group_name = "gripper";
    const auto& hand_frame = "flange";

    // Set task properties
    task_->setProperty("group", arm_group_name);
    task_->setProperty("eef", hand_group_name);
    task_->setProperty("ik_frame", hand_frame);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.001);

    // Current state
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    task_->add(std::move(stage_state_current)); 

    auto attach_stage = std::make_unique<mtc::stages::ModifyPlanningScene>(name);
    attach_stage->attachObject(target_object, hand_frame);
    task_->add(std::move(attach_stage));
}

void MTCNode::createDetachObjectStage(const std::string& name, const std::string& target_object, const std::string& link) {
    RCLCPP_INFO(this->get_logger(), "Service message is processing");

    task_->loadRobotModel(shared_from_this());
    task_->stages()->setName("demo task");

    const auto& arm_group_name = "ur5e_arm";
    const auto& hand_group_name = "gripper";
    const auto& hand_frame = "flange";

    // Set task properties
    task_->setProperty("group", arm_group_name);
    task_->setProperty("eef", hand_group_name);
    task_->setProperty("ik_frame", hand_frame);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.001);

    // Current state
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    task_->add(std::move(stage_state_current)); 

    auto detach_stage = std::make_unique<mtc::stages::ModifyPlanningScene>(name);
    detach_stage->detachObject(target_object, hand_frame);
    task_->add(std::move(detach_stage));
}

void MTCNode::createAllowCollisionStage(const std::string& name, const std::string& target_object, const std::vector<std::string>& objects, bool allow) {
    RCLCPP_INFO(this->get_logger(), "Service message is processing");

    task_->loadRobotModel(shared_from_this());
    task_->stages()->setName("demo task");

    const auto& arm_group_name = "ur5e_arm";
    const auto& hand_group_name = "gripper";
    const auto& hand_frame = "flange";

    // Set task properties
    task_->setProperty("group", arm_group_name);
    task_->setProperty("eef", hand_group_name);
    task_->setProperty("ik_frame", hand_frame);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.001);

    // Current state
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    task_->add(std::move(stage_state_current)); 

    auto allow_collision_stage = std::make_unique<mtc::stages::ModifyPlanningScene>(name);
    if (target_object == "hand"){
        allow_collision_stage->allowCollisions(objects,
                                   task_->getRobotModel()
                                       ->getJointModelGroup(hand_group_name)
                                       ->getLinkModelNamesWithCollisionGeometry(),
                                   allow);
    } else {
        allow_collision_stage->allowCollisions(target_object, objects, allow);
    }
    task_->add(std::move(allow_collision_stage));
}

bool MTCNode::planAndExecute(mtc::Task& task) {
    try {
        RCLCPP_INFO(this->get_logger(), "Initializing task...");
        task.init();
        RCLCPP_INFO(this->get_logger(), "Task initialized successfully.");
    } catch (const mtc::InitStageException& e) {
        RCLCPP_ERROR(this->get_logger(), "Task initialization failed: %s", e.what());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Planning task...");
    if (!task.plan(10)) {
        RCLCPP_ERROR(this->get_logger(), "Task planning failed");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Task planning successful.");

    task.introspection().publishSolution(*task.solutions().front());

    RCLCPP_INFO(this->get_logger(), "Executing task...");
    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Task execution failed with error code: %d", result.val);
        return false;
    }
    publishTaskDetails(task);

    RCLCPP_INFO(this->get_logger(), "Task execution succeeded.");
    return true;
}

void MTCNode::publishTaskDetails(const mtc::Task& task) {
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
    RCLCPP_INFO(this->get_logger(), "Publishing TaskDetails message: %s", task_msg.name.c_str());
    task_details_publisher_->publish(task_msg);
}
/* std::tuple<std::shared_ptr<mtc::solvers::PipelinePlanner>, 
        std::shared_ptr<mtc::solvers::CartesianPath>, 
        mtc::Task> MTCNode::initializeTask()
{
    mtc::Task task;
    task.stages()->setName("demo task");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "ur5e_arm";
    const auto& hand_group_name = "gripper";
    const auto& hand_frame = "flange";

    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    // Current state
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    task.add(std::move(stage_state_current));

    return std::make_tuple(sampling_planner, cartesian_planner, task);
} */


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<MTCNode>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
        executor.add_node(node);
        executor.spin();
        executor.remove_node(node);
    });  
    spin_thread->join();    
    rclcpp::shutdown();
    return 0;
}
