# FullDrive Package

## Overview

The `FullDrive` package is designed to control a robotic system using ROS 2, integrating motion planning, collision management, and gripper control. This package leverages MoveIt Task Constructor (MTC) for complex motion planning tasks and integrates with a UR robot through the RTDE interface. 

## Package Contents

1. **Nodes**:
    - **sync_rviz_node.cpp**: Synchronizes the RViz environment with the current state of the robot and the planning scene.
    - **planning_scene_node.cpp**: Manages the planning scene, including the addition and removal of collision objects.
    - **action_server_node.cpp**: Handles various actions requested by clients, including motion planning, collision management, and gripper control.

2. **Client Node**:
    - **usage_node.cpp**: Allows users to select and execute specific actions. Utilizes `client.h` for compact and efficient action requests.

3. **Header Files**:
    - **client.h**: Provides a streamlined interface for the client node to request actions from the action server.
    - **mtc.h**: Manages MTC stages and execution for motion planning tasks.
    - **update_planning_scene.h**: Interfaces with the MoveIt planning scene to add, remove, and modify collision objects.

## Action Definitions

The `FullDrive.action` file defines the structure of the actions that can be requested by the client. The available actions are:

- `add_collision_object`
- `delete_collision_object`
- `attach_object`
- `detach_object`
- `move_to`
- `move_linear`
- `check_robot_status`
- `allow_collision`
- `reenable_collision`
- `current_state`
- `set_gripper_position`

### Action Message Structure

**Goal Definition**:
```plaintext
geometry_msgs/PoseStamped target_pose
shape_msgs/SolidPrimitive object_primitive
geometry_msgs/Pose object_pose
std_msgs/ColorRGBA color
bool add_collision_object
bool delete_collision_object
bool attach_object
bool detach_object
bool move_to
bool move_linear
bool check_robot_status
bool allow_collision
bool reenable_collision
bool current_state
bool set_gripper_position
float64 gripper_position
string object_name
string target_name
string task
string id
string link
```

**Result Definition**:
```plaintext
bool success
string message
```

**Feedback Definition**:
```plaintext
float32 progress
string status
```

## How to Run

### Launching the Nodes

1. **Synchronize RViz**: 
    ```bash
    ros2 run full_drive sync_rviz_node
    ```
    - This node must run first to synchronize the RViz environment with the robot's state.

2. **Launch the Main Nodes**:
    ```bash
    ros2 launch full_drive full_drive.launch.py
    ```
    - This launch file will start the `planning_scene_node` and `action_server_node`.

### Running the Client Node

The client node can be run separately, allowing the user to request specific actions:

```bash
ros2 run full_drive usage_node
```

## Functionality

### Motion Planning

The action server uses MTC for motion planning. The following tasks can be planned and executed:

- **Move to a specific pose**: `move_to`
- **Move linearly to a specific pose**: `move_linear`

### Gripper Control

The action server can control the gripper by setting its position:

- **Set gripper position**: `set_gripper_position`

### Collision Management

The action server can manage collision objects in the planning scene:

- **Add collision object**: `add_collision_object`
- **Delete collision object**: `delete_collision_object`
- **Attach object to a link**: `attach_object`
- **Detach object from a link**: `detach_object`
- **Allow collision between objects**: `allow_collision`
- **Re-enable collision between objects**: `reenable_collision`

### Checking Robot Status

The action server can check if the robot is currently moving:

- **Check robot status**: `check_robot_status`

## Implementation Details

### `client.h`

The `client.h` file provides an interface for the client node (`usage_node.cpp`) to send action requests to the server (`action_server_node.cpp`). It simplifies the client code by encapsulating the action communication logic.

### `mtc.h`

The `mtc.h` file manages the stages of MTC tasks. It includes methods to create and execute various stages such as moving to a pose, linear movements, and managing gripper states.

### `update_planning_scene.h`

The `update_planning_scene.h` file interfaces with the MoveIt planning scene to add, remove, and modify collision objects. It ensures that the planning scene is updated in real-time based on the actions executed by the server.

## License
This project is licensed under the Apache 2.0 License. See the [LICENSE](/LICENSE) file for details.


## Acknowledgements
Thanks to the MoveIt and ROS 2 community for their continuous support and development of these powerful tools. I would like to specially thank the AMRL team at HAWK University for providing me the opportunity to work on this project.
.
