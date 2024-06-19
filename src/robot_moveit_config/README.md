# UR5e Robot with 2F-140 Robotiq Gripper MoveIt Configuration

This repository contains the configuration files for the UR5e robot arm with a 2F-140 Robotiq gripper, generated using the MoveIt Setup Assistant. 


## Introduction

This repository provides all necessary configuration files to control a UR5e robot arm equipped with a 2F-140 Robotiq gripper using MoveIt. These files were generated using the MoveIt Setup Assistant and include URDF, SRDF, kinematics, and controller configurations.

## Setup Instructions

Follow these steps to set up the UR5e robot with the 2F-140 Robotiq gripper in your ROS2 workspace:


1. Navigate to your workspace:
   ```bash
   cd ~/ws_moveit2/src
   ```

2. Clone this repository:
   ```bash
   git clone https://github.com/Sohaib-Snouber/robot_moveit_config.git
   ```
   
3. Clone this repository: These packages contain the needed configuration files.
   ```bash
   git clone https://github.com/Sohaib-Snouber/UR5e_robotiq_gripper_RViz.git
   ```
   
4. Build your workspace:
   ```bash
   cd ~/ws_moveit2
   colcon build
   ```

5. Source your workspace:
   ```bash
   source install/setup.bash
   ```

## MoveIt Setup Assistant Configuration

To regenerate these configuration files or to make modifications, use the MoveIt Setup Assistant:

1. **Launch MoveIt Setup Assistant:**
   ```bash
   ross launch moveit_setup_assistant setup_assistant.launch.py
   ```

2. **Load the URDF file:**
   - In the Setup Assistant, click on "Create New MoveIt Configuration Package".
   - Load your URDF file (e.g., `ur5e_robot.urdf`) which includes both the UR5e robot and the 2F-140 Robotiq gripper.
   - An URDF file can be found in this repository [here](https://github.com/Sohaib-Snouber/UR5e_robotiq_gripper_RViz.git), that is being created from the xacro file of the robot configuration.

3. **Configure the Robot:**
   - Follow the steps to configure the Self-Collision Matrix, Virtual Joints, Planning Groups, Robot Poses, End Effectors, Passive Joints, and the 2F-140 gripper.
   - Generate the SRDF configuration file by specifying the planning groups and robot poses.
   
4. **Generate Configuration Files:**
   - Once all configurations are completed, generate the configuration package. The necessary files (SRDF, kinematics.yaml, controllers.yaml, etc.) will be generated in your chosen directory.

5. **Save the Configuration:**
   - Save the generated configuration files and integrate them into your workspace by replacing the existing ones if necessary.

## File Structure

Here is an overview of the main files and directories in this repository:

```plaintext
UR5e_gripper_moveit_config/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── initial_positions.yaml
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   ├── moveit.rviz
│   ├── moveit_controllers.yaml
│   ├── pilz_cartesian_limits.yaml
│   ├── ros2_controllers.yaml
│   ├── ur5e_robot.ros2_control.xacro
│   ├── ur5e_robot.srdf
│   ├── ur5e_robot.urdf.xacro
├── launch/
│   ├── demo.launch.py
│   ├── move_group.launch.py
│   ├── moveit_rviz.launch.py
│   ├── rsp.launch.py
│   ├── setup_assistant.launch.py
│   ├── spawn_controllers.launch.py
│   ├── static_virtual_joint_tfs.launch.py
│   ├── warehouse_db.launch.py
└── .setup_assistant
```

## Usage

To use the provided configurations, simply launch the demo launch file:

```bash
ros2 launch robot_moveit_config demo.launch.py
```

This will start the MoveIt! RViz interface with the UR5e robot and the 2F-140 Robotiq gripper, allowing you to plan and execute motions.


## Acknowledgment

This project was made possible thanks to the excellent tools and extensive development by the MoveIt community. The MoveIt Setup Assistant and the MoveIt framework have been instrumental in configuring and controlling the UR5e robot and the 2F-140 Robotiq gripper. We greatly appreciate their efforts and contributions to the robotics community.


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

