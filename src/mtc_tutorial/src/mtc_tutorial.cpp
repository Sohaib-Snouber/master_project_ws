#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
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
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
// for allowing the base link inertia to collide with the surface.
#include <moveit_msgs/msg/allowed_collision_matrix.h>
#include <moveit_msgs/msg/allowed_collision_entry.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject collision_object_table1;
  collision_object_table1.header.frame_id = "world";
  collision_object_table1.id = "table1";
  shape_msgs::msg::SolidPrimitive primitive_table1;
  primitive_table1.type = primitive_table1.BOX;
  primitive_table1.dimensions.resize(3);
  primitive_table1.dimensions[primitive_table1.BOX_X] = 0.9;
  primitive_table1.dimensions[primitive_table1.BOX_Y] = 0.6;
  primitive_table1.dimensions[primitive_table1.BOX_Z] = 0.2;
  geometry_msgs::msg::Pose table1_pose;
  table1_pose.orientation.w = 1.0;
  table1_pose.position.x = 0.45;
  table1_pose.position.y = -0.6;
  table1_pose.position.z = 0.1;
  collision_object_table1.primitives.push_back(primitive_table1);
  collision_object_table1.primitive_poses.push_back(table1_pose);
  collision_object_table1.operation = collision_object_table1.ADD;

  moveit_msgs::msg::CollisionObject collision_object_table2;
  collision_object_table2.header.frame_id = "world";
  collision_object_table2.id = "table2";
  shape_msgs::msg::SolidPrimitive primitive_table2;
  primitive_table2.type = primitive_table2.BOX;
  primitive_table2.dimensions.resize(3);
  primitive_table2.dimensions[primitive_table2.BOX_X] = 0.6;
  primitive_table2.dimensions[primitive_table2.BOX_Y] = 0.9;
  primitive_table2.dimensions[primitive_table2.BOX_Z] = 0.2;
  geometry_msgs::msg::Pose table2_pose;
  table2_pose.orientation.w = 1.0;
  table2_pose.position.x = -0.60;
  table2_pose.position.y = 0.0;
  table2_pose.position.z = 0.1;
  collision_object_table2.primitives.push_back(primitive_table2);
  collision_object_table2.primitive_poses.push_back(table2_pose);
  collision_object_table2.operation = collision_object_table2.ADD;

  moveit_msgs::msg::CollisionObject collision_object_target1;
  collision_object_target1.header.frame_id = "world";
  collision_object_target1.id = "target1";
  shape_msgs::msg::SolidPrimitive primitive_target1;
  primitive_target1.type = primitive_target1.CYLINDER;
  primitive_target1.dimensions.resize(2);
  primitive_target1.dimensions[0] = 0.1; // height
  primitive_target1.dimensions[1] = 0.02; // radius
  geometry_msgs::msg::Pose target1_pose;
  target1_pose.orientation.w = 1.0;
  target1_pose.position.x = 0.45;
  target1_pose.position.y = -0.35;
  target1_pose.position.z = 0.25;
  collision_object_target1.primitives.push_back(primitive_target1);
  collision_object_target1.primitive_poses.push_back(target1_pose);
  collision_object_target1.operation = collision_object_target1.ADD;

  moveit_msgs::msg::CollisionObject collision_object_surface;
  collision_object_surface.header.frame_id = "world";
  collision_object_surface.id = "surface";
  shape_msgs::msg::SolidPrimitive primitive_surface;
  primitive_surface.type = primitive_surface.BOX;
  primitive_surface.dimensions.resize(3);
  primitive_surface.dimensions[primitive_surface.BOX_X] = 0.75;
  primitive_surface.dimensions[primitive_surface.BOX_Y] = 0.8;
  primitive_surface.dimensions[primitive_surface.BOX_Z] = 0.1;
  geometry_msgs::msg::Pose surface_pose;
  surface_pose.orientation.w = 1.0;
  surface_pose.position.x = 0.16;
  surface_pose.position.y = 0.25;
  surface_pose.position.z = -0.05;
  collision_object_surface.primitives.push_back(primitive_surface);
  collision_object_surface.primitive_poses.push_back(surface_pose);
  collision_object_surface.operation = collision_object_surface.ADD;

  RCLCPP_INFO(LOGGER, "collision objects created");

  // Define colors
  moveit_msgs::msg::ObjectColor table1_color;
  table1_color.id = collision_object_table1.id;
  table1_color.color.r = 1.0; // Red
  table1_color.color.g = 0.5; // Green
  table1_color.color.b = 0.5; // Blue
  table1_color.color.a = 1.0; // Alpha

  moveit_msgs::msg::ObjectColor table2_color;
  table2_color.id = collision_object_table2.id;
  table2_color.color.r = 1.0; // Red
  table2_color.color.g = 0.5; // Green
  table2_color.color.b = 0.5; // Blue
  table2_color.color.a = 1.0; // Alpha

  moveit_msgs::msg::ObjectColor target1_color;
  target1_color.id = collision_object_target1.id;
  target1_color.color.r = 0.5; // Red
  target1_color.color.g = 0.0; // Green
  target1_color.color.b = 1.0; // Blue
  target1_color.color.a = 1.0; // Alpha

  moveit_msgs::msg::ObjectColor surface_color;
  surface_color.id = collision_object_surface.id;
  surface_color.color.r = 0.5; // Gray
  surface_color.color.g = 0.5;
  surface_color.color.b = 0.5;
  surface_color.color.a = 1.0; // Alpha

  // Create a PlanningScene message and add the collision object and its color
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;
  planning_scene_msg.world.collision_objects.push_back(collision_object_table1);
  planning_scene_msg.world.collision_objects.push_back(collision_object_table2);
  planning_scene_msg.world.collision_objects.push_back(collision_object_target1);
  planning_scene_msg.world.collision_objects.push_back(collision_object_surface);
  planning_scene_msg.object_colors.push_back(table1_color);
  planning_scene_msg.object_colors.push_back(table2_color);
  planning_scene_msg.object_colors.push_back(target1_color);
  planning_scene_msg.object_colors.push_back(surface_color);

  // Apply the planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyPlanningScene(planning_scene_msg);

  // Create a RobotModelLoader
  robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
  moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(robot_model);

  // Modify the ACM to allow collision between base_link_inertia and surface
  collision_detection::AllowedCollisionMatrix& acm = planning_scene.getAllowedCollisionMatrixNonConst();
  acm.setEntry("base_link_inertia", "surface", true);

  // Update the planning scene with the modified ACM
  moveit_msgs::msg::AllowedCollisionMatrix acm_msg;
  acm.getMessage(acm_msg);
  planning_scene_msg.allowed_collision_matrix = acm_msg;
  planning_scene_interface.applyPlanningScene(planning_scene_msg);

  RCLCPP_INFO(LOGGER, "allow collision between base_link_inertia and surface.");
}


void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(20))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
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

  // Open hand
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  // Define the target pose
  auto const target_pose = []{
	  geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
	  
    // Define roll, pitch, and yaw angles in radians
	  double roll = M_PI/2.0;  // Rotation around the x-axis
	  double pitch = 0.0;  // Rotation around the y-axis (90 degrees)
	  double yaw = 0.0;  // Rotation around the z-axis
	  // Convert roll, pitch, yaw to quaternion
	  tf2::Quaternion q;
	  q.setRPY(roll, pitch, yaw);

	  msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
	  msg.pose.position.x = 0.45;
	  msg.pose.position.y = -0.10;  //to 10mm slide move to got to grasp the object
	  msg.pose.position.z = 0.25;
	  return msg;
	}();
  
  // Move to target pose
  auto move_to_target = std::make_unique<mtc::stages::MoveTo>("move to target", sampling_planner);
  move_to_target->setGroup(arm_group_name);
  move_to_target->setGoal(target_pose);
  task.add(std::move(move_to_target));

  // Define the slide pose
  auto const slide_pose = []{
	  geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
	  
    // Define roll, pitch, and yaw angles in radians
	  double roll = M_PI/2.0;  // Rotation around the x-axis
	  double pitch = 0.0;  // Rotation around the y-axis (90 degrees)
	  double yaw = 0.0;  // Rotation around the z-axis
	  // Convert roll, pitch, yaw to quaternion
	  tf2::Quaternion q;
	  q.setRPY(roll, pitch, yaw);

	  msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
	  msg.pose.position.x = 0.45;
	  msg.pose.position.y = -0.18;  //to 8cm slide move
	  msg.pose.position.z = 0.25;
	  return msg;
	}();
  
  // Move to slide pose
  auto move_to_slide = std::make_unique<mtc::stages::MoveTo>("move to slide", cartesian_planner);
  move_to_slide->setGroup(arm_group_name);
  move_to_slide->setGoal(slide_pose);
  task.add(std::move(move_to_slide));

  // Allow collision (hand, target1) temporarily
  auto allow_collision_target1 = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand, target1)");
  allow_collision_target1->allowCollisions("target1",
                                   task.getRobotModel()
                                       ->getJointModelGroup(hand_group_name)
                                       ->getLinkModelNamesWithCollisionGeometry(),
                                   true);
  task.add(std::move(allow_collision_target1));

  // Allow collision (target1, table1) temporarily
  auto allow_collision_target1_table1 = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (target1, table1)");
  allow_collision_target1_table1->allowCollisions("target1", {"table1"}, true);
  task.add(std::move(allow_collision_target1_table1));

  // Close hand
  auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
  stage_close_hand->setGroup(hand_group_name);
  stage_close_hand->setGoal("close");
  task.add(std::move(stage_close_hand));

  // Attach target1 to the gripper
  auto attach_target1 = std::make_unique<mtc::stages::ModifyPlanningScene>("attach target1");
  attach_target1->attachObject("target1", hand_frame);
  task.add(std::move(attach_target1));

  // Lift target1 slightly to avoid collision with the table
  auto lift_target1 = std::make_unique<mtc::stages::MoveRelative>("lift target1", cartesian_planner);
  lift_target1->properties().set("marker_ns", "lift_target1");
  lift_target1->properties().set("link", hand_frame);
  lift_target1->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  lift_target1->setMinMaxDistance(0.1, 0.15); // Increase lift height

  // Set lift direction
  geometry_msgs::msg::Vector3Stamped lift_direction;
  lift_direction.header.frame_id = "world";
  lift_direction.vector.z = 1.0; // Lift upwards
  lift_target1->setDirection(lift_direction);
  task.add(std::move(lift_target1));

  

  // Re-enable collision (target1, table1) after lifting
  auto reenable_collision_target1_table1 = std::make_unique<mtc::stages::ModifyPlanningScene>("reenable collision (target1, table1)");
  reenable_collision_target1_table1->allowCollisions("target1", {"table1"}, false);
  task.add(std::move(reenable_collision_target1_table1));

  
  // Define the target2 pose
  auto const target2_pose = []{
	  geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
	  
    // Define roll, pitch, and yaw angles in radians
	  double roll1 = 0.0;  // Rotation around the x-axis
	  double pitch1 = -M_PI/2.0;  // Rotation around the y-axis (90 degrees)
	  double yaw1 = 0.0;  // Rotation around the z-axis
	  // Convert roll, pitch, yaw to quaternion
	  tf2::Quaternion q1;
	  q1.setRPY(roll1, pitch1, yaw1);
    // Define roll, pitch, and yaw angles in radians
	  double roll2 = 0.0;  // Rotation around the x-axis
	  double pitch2 = 0.0;  // Rotation around the y-axis (90 degrees)
	  double yaw2 = M_PI/2.0;  // Rotation around the z-axis
	  // Convert roll, pitch, yaw to quaternion
	  tf2::Quaternion q2;
	  q2.setRPY(roll2, pitch2, yaw2);

    tf2::Quaternion q_combined = q1 * q2;
    q_combined.normalize();

	  msg.pose.orientation.x = q_combined.x();
    msg.pose.orientation.y = q_combined.y();
    msg.pose.orientation.z = q_combined.z();
    msg.pose.orientation.w = q_combined.w();
	  msg.pose.position.x = -0.35;
	  msg.pose.position.y = 0.0;
	  msg.pose.position.z = 0.40;
	  return msg;
	}();
  
  // Move to target pose
  auto move_to_target2 = std::make_unique<mtc::stages::MoveTo>("move to target2", sampling_planner);
  move_to_target2->setGroup(arm_group_name);
  move_to_target2->setGoal(target2_pose);
  task.add(std::move(move_to_target2));

  // Allow collision (target1, table2) temporarily
  auto allow_collision_target1_table2 = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (target1, table2)");
  allow_collision_target1_table2->allowCollisions("target1", {"table2"}, true);
  task.add(std::move(allow_collision_target1_table2));

  // Define the slide pose
  auto const slide2_pose = []{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
	  // Define roll, pitch, and yaw angles in radians
	  double roll1 = 0.0;  // Rotation around the x-axis
	  double pitch1 = -M_PI/2.0;  // Rotation around the y-axis (90 degrees)
	  double yaw1 = 0.0;  // Rotation around the z-axis
	  // Convert roll, pitch, yaw to quaternion
	  tf2::Quaternion q1;
	  q1.setRPY(roll1, pitch1, yaw1);
    // Define roll, pitch, and yaw angles in radians
	  double roll2 = 0.0;  // Rotation around the x-axis
	  double pitch2 = 0.0;  // Rotation around the y-axis (90 degrees)
	  double yaw2 = M_PI/2.0;  // Rotation around the z-axis
	  // Convert roll, pitch, yaw to quaternion
	  tf2::Quaternion q2;
	  q2.setRPY(roll2, pitch2, yaw2);

    tf2::Quaternion q_combined = q1 * q2;
    q_combined.normalize();

	  msg.pose.orientation.x = q_combined.x();
    msg.pose.orientation.y = q_combined.y();
    msg.pose.orientation.z = q_combined.z();
    msg.pose.orientation.w = q_combined.w();
	  msg.pose.position.x = -0.35;
	  msg.pose.position.y = 0.0;
	  msg.pose.position.z = 0.25;
	  return msg;
	}();
  
  // Move to slide pose
  auto move_to_slide2 = std::make_unique<mtc::stages::MoveTo>("move to slide", cartesian_planner);
  move_to_slide2->setGroup(arm_group_name);
  move_to_slide2->setGoal(slide2_pose);
  task.add(std::move(move_to_slide2));

  // Open hand
  auto stage2_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
  stage2_open_hand->setGroup(hand_group_name);
  stage2_open_hand->setGoal("open");
  task.add(std::move(stage2_open_hand));

  // Detach target1 from the gripper
  auto detach_target1 = std::make_unique<mtc::stages::ModifyPlanningScene>("detach target1");
  detach_target1->detachObject("target1", hand_frame);
  task.add(std::move(detach_target1));

  // Lift gripper slightly to avoid collision with the table
  auto lift_gripper = std::make_unique<mtc::stages::MoveRelative>("lift gripper", cartesian_planner);
  lift_gripper->properties().set("marker_ns", "lift_gripper");
  lift_gripper->properties().set("link", hand_frame);
  lift_gripper->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  lift_gripper->setMinMaxDistance(0.1, 0.15); // Increase lift height

  // this may cause segmentation error
  // Set lift direction
  /* geometry_msgs::msg::Vector3Stamped lift_direction;
  lift_direction.header.frame_id = "world";
  lift_direction.vector.z = 1.0; // Lift upwards */
  lift_gripper->setDirection(lift_direction);
  task.add(std::move(lift_gripper));

  // Re-enable collision (target1, table2) after lifting
  auto reenable_collision_target1_table2 = std::make_unique<mtc::stages::ModifyPlanningScene>("reenable collision (target1, table2)");
  reenable_collision_target1_table2->allowCollisions("target1", {"table2"}, false);
  task.add(std::move(reenable_collision_target1_table2));


  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
  /* // Move wrist joint back to its initial position
  std::map<std::string, double> initial_joint_positions;
  initial_joint_positions["wrist_3_joint"] = 3.14; // Adjust this value based on the initial position of wrist_3_joint
  auto move_to_initial = std::make_unique<mtc::stages::MoveTo>("reset wrist joint", sampling_planner);
  move_to_initial->setGroup(arm_group_name);
  move_to_initial->setGoal(initial_joint_positions);
  task.add(std::move(move_to_initial));
 */
