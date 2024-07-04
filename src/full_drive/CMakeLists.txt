cmake_minimum_required(VERSION 3.8)
project(full_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_interfaces REQUIRED)
find_package(master_project_msgs REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)

include_directories(
  include
)

add_executable(sync_rviz_pose src/sync_rviz_pose.cpp)
ament_target_dependencies(sync_rviz_pose
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
  geometry_msgs
  std_msgs
  moveit_msgs
  moveit_core
  ur_rtde
)

add_executable(action_server_node src/action_server_node.cpp)
ament_target_dependencies(action_server_node 
  rclcpp 
  rclcpp_action 
  master_project_msgs 
  moveit_core 
  moveit_task_constructor_core 
  moveit_ros_planning_interface
  action_interfaces
  ur_rtde
)
add_executable(usage_node src/usage_node.cpp)
ament_target_dependencies(usage_node rclcpp rclcpp_action moveit_core moveit_ros_planning_interface ur_rtde action_interfaces)

install(TARGETS
  sync_rviz_pose
  action_server_node
  usage_node
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(action_server_node
  ur_rtde::rtde
)
target_link_libraries(sync_rviz_pose
  ur_rtde::rtde
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
