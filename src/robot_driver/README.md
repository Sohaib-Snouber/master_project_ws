# robot_driver Package

## Overview
The `robot_driver` package is a core component of the Master Project Workspace, responsible for controlling the UR5e robotic arm and Robotiq 2F140 gripper. This package includes nodes for syncing robot poses, setting up the planning scene, executing tasks, and managing long-running actions.

## Table of Contents
- [Overview](#overview)
- [Nodes](#nodes)
  - [sync_rviz_pose](#sync_rviz_pose)
  - [planning_scene_node](#planning_scene_node)
  - [target_pose_node](#target_pose_node)
  - [task_execution_node](#task_execution_node)
  - [action_server_node](#action_server_node)
  - [usage_node](#usage_node)
- [Launch Files](#launch-files)
- [Running the Nodes](#running-the-nodes)
- [Dependencies](#dependencies)
- [License](#license)

## Nodes

### sync_rviz_pose
**Description:** This node contacts the UR5e robot over remote control with a static IP address using the `ur_rtde` library to get the status of robot joints and synchronize them with RViz for visualization.

### planning_scene_node
**Description:** This node sets up the planning scene by placing collision objects to represent the environment around the robot.

### target_pose_node
**Description:** This node is intended for future use with the ROS1 bridge to facilitate communication with ROS1. Currently, it is not in use.

### task_execution_node
**Description:** This node uses the MoveIt Task Constructor to plan a series of tasks according to the subscribed task poses, which will be published later from the `target_pose_node`.

### action_server_node
**Description:** This node runs separately after launching the previous nodes. It acts as an action server, managing long-running processes such as moving the robot and controlling the gripper.

### usage_node
**Description:** This node acts as a client, sending requests to the `action_server_node` to perform various tasks implemented on the server.

## Launch Files

### robot_driver.launch.py
**Description:** This launch file orchestrates the startup of several nodes within the `robot_driver` package, ensuring they are launched in the correct sequence and with the necessary parameters. It first launches the `sync_rviz_pose` node to synchronize the robot's state with RViz. Once this node has completed successfully, the launch file sequentially starts the `planning_scene_node`, `target_pose_node`, and `task_execution_node`. This ensures that the environment is set up correctly and the nodes that depend on each other are launched in the right order.

## Running the Nodes

### Step 1: Launch RViz with the Necessary Components
First, launch RViz with the necessary components using the provided launch file from the [mtc_tutorial](https://github.com/Sohaib-Snouber/master_project_ws/tree/main/src/mtc_tutorial) package:
```bash
ros2 launch mtc_tutorial mtc_demo.launch.py
```

### Step 2: Launch the `robot_driver` Nodes
Next, launch the nodes defined in the `robot_driver` package using its launch file:
```bash
ros2 launch robot_driver robot_driver.launch.py
```

This will start the `sync_rviz_pose` node first. Once it has successfully completed, the `planning_scene_node`, `target_pose_node`, and `task_execution_node` will be launched automatically.

### Step 3: Run the Action Server Node
After the nodes have been launched, run the `action_server_node` to manage long-running processes:
```bash
ros2 run robot_driver action_server_node
```

### Step 4: Run the Usage Node
Finally, run the `usage_node` to send requests to the `action_server_node` to perform various tasks:
```bash
ros2 run robot_driver usage_node
```

## Dependencies
- ROS2 Humble
- MoveIt2 Humble
- ur_rtde library

Ensure all dependencies are installed and sourced correctly before running the nodes.

## License
This project is licensed under the Apache 2.0 License. See the [LICENSE](/LICENSE) file for details.
