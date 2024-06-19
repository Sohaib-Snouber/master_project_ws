# Master Project Workspace

## Overview
This project is a ROS2 workspace designed to control a UR5e robotic arm with a Robotiq 2F140 gripper. It utilizes MoveIt for motion planning and the `ur_rtde` library for real-time data exchange, allowing remote control of the robot. This README provides detailed instructions on setting up the environment, installing necessary dependencies, and running the project.

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

### Installing MoveIt2 Humble
Follow the getting started guide for MoveIt2 Humble from [here](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html).

### Installing ur_rtde Library
Install the `ur_rtde` library by following the instructions for Ubuntu from [here](https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html).

### Cloning the Repository
Clone this repository to your local machine:
```bash
git clone https://github.com/Sohaib-Snouber/master_project_ws.git
cd master_project_ws
```

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


## Running the Project


## Common Issues


## Contributing
Contributions are welcome! Please fork this repository and submit pull requests.

## License
This project is licensed under the Apache 2.0 License. See the [LICENSE](LICENSE) file for details.
