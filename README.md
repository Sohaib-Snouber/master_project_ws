# Master Project Workspace

## Overview
This project is a ROS2 workspace designed to control a UR5e robotic arm with a Robotiq 2F140 gripper. It utilizes MoveIt for motion planning and the `ur_rtde` library for real-time data exchange, enabling remote control of the robot. This README provides detailed instructions on setting up the environment, installing necessary dependencies, and running the project.

## Table of Contents
- [Overview](#overview)
- [Setup Instructions](#setup-instructions)
  - [System Requirements](#system-requirements)
  - [Installing ROS2 Humble](#installing-ros2-humble)
  - [Installing MoveIt2 Humble](#installing-moveit2-humble)
  - [Installing ur_rtde Library](#installing-ur_rtde-library)
  - [Cloning the Repository](#cloning-the-repository)
  - [Building the Workspace](#building-the-workspace)
- [Packages](#packages)
- [Running the Project](#running-the-project)
- [Common Issues](#common-issues)
- [Contributing](#contributing)
- [License](#license)

## Setup Instructions

### System Requirements
- Ubuntu 22.04
- ROS2 Humble
- MoveIt2 Humble
- ur_rtde library

### Installing ROS2 Humble
Follow the official instructions to install ROS2 Humble from [here](https://docs.ros.org/en/humble/Installation.html).

### Cloning the Repository
Clone this repository to your local machine:
```bash
git clone https://github.com/Sohaib-Snouber/master_project_ws.git
cd master_project_ws
```

### Installing MoveIt2 Humble
Follow the getting started guide for MoveIt2 Humble from [here](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html).
Note: Do not create another workspace; just navigate to the `cloned master_project_ws/src` directory and clone the MoveIt tutorials and other packages there.

### Installing ur_rtde Library
Install the `ur_rtde` library by following the instructions for Ubuntu from [here](https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html).
If you encounter issues related to the installation and usage of this library, refer to the [detailed setup instructions](https://github.com/Sohaib-Snouber/master_project_ws.git) for this workspace.

### Building the Workspace
1. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. Build the workspace:
   ```bash
   colcon build
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Packages
The following packages are used in this project, listed in their dependency order:

1. **UR5e_robotiq_gripper_RViz**: This package configures the robot and gripper in RViz, allowing you to verify that the configuration matches the real setup. For more details, see [here](https://github.com/Sohaib-Snouber/master_project_ws/tree/main/src/UR5e_robotiq_gripper_RViz).

2. **robot_moveit_config**: This package uses the previous package to set up the robot configuration in MoveIt using the MoveIt Setup Assistant, which generates this package. For more details, see [here](https://github.com/Sohaib-Snouber/master_project_ws/tree/main/src/robot_moveit_config).

3. **mtc_tutorial**: This package contains tutorials and examples for using the MoveIt Task Constructor (MTC). These tutorials are helpful for learning how to plan and execute complex tasks with the robotic arm. For more details, see [here](https://github.com/Sohaib-Snouber/master_project_ws/tree/main/src/mtc_tutorial).

4. **master_project_msgs**: This package contains the necessary message definitions used across the project. These messages help define tasks, stages, waypoints, and joint states within the project. For more details, see [here](https://github.com/Sohaib-Snouber/master_project_ws/tree/main/src/master_project_msgs).

5. **action_interfaces**: This package defines the action interfaces used for processing tasks within the project. It includes the `ProcessTask` action, which specifies the goal, feedback, and result messages for processing tasks. For more details, see [here](https://github.com/Sohaib-Snouber/master_project_ws/tree/main/src/action_interfaces).

6. **robot_driver**: Handles the core functionalities for controlling the UR5e robot and Robotiq gripper, including syncing poses, setting up the planning scene, executing tasks, and managing actions. For more details, see [here](https://github.com/Sohaib-Snouber/master_project_ws/tree/main/src/robot_driver).

## Running the Project
### Launching the Nodes
1. Launch RViz with the necessary components:
   ```bash
   ros2 launch mtc_tutorial mtc_demo.launch.py
   ```

2. Launch the `robot_driver` package nodes:
   ```bash
   ros2 launch robot_driver robot_driver.launch.py
   ```
   
3. Run the Action Server Node:
    ```bash
    ros2 run robot_driver action_server_node
    ```

4. Run the Usage Node (client):
    ```bash
    ros2 run robot_driver usage_node
    ```

## Common Issues


## Contributing
Contributions are welcome! Please fork this repository and submit pull requests.

## License
This project is licensed under the Apache 2.0 License. See the [LICENSE](LICENSE) file for details.
