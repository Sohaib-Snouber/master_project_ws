# Master Project Msgs Package

The `master_project_msgs` package contains the necessary message definitions that will be used in other packages. These message definitions are integral to the hierarchical structure of tasks, stages, waypoints, and joint states within the project.

## Task Message Structure

The `Task` message is composed of several nested messages that define a hierarchical structure. Below is a detailed description of each message type and their fields.

### Task.msg

The `Task` message includes the following fields:
- **name** (string): The name of the task.
- **stages** (array of `Stage`): The stages that make up the task.
- **number_of_stages** (uint32): The total number of stages in the task.

### Stage.msg

The `Stage` message includes the following fields:
- **name** (string): The name of the stage.
- **waypoints** (array of `Waypoint`): The waypoints within the stage.

### Waypoint.msg

The `Waypoint` message includes the following fields:
- **joints** (array of `JointState`): The joint states at the waypoint.

### JointState.msg

The `JointState` message includes the following fields:
- **name** (string): The name of the joint.
- **position** (float64): The position of the joint.

### Diagram

Below is a diagram illustrating the structure and relationships between the different messages:

```plaintext
Task
├── string name
├── Stage[] stages
│   ├── string name
│   └── Waypoint[] waypoints
│       └── JointState[] joints
│           ├── string name
│           └── float64 position
└── uint32 number_of_stages
```

This diagram represents the hierarchical structure of the `Task` message, showing how each task consists of multiple stages, each stage contains multiple waypoints, and each waypoint includes multiple joint states.

### Purpose and Usage

This task structure is designed to store the data that comes from the MoveIt Task Constructor. MoveIt Task Constructor plans complex tasks by breaking them down into multiple stages, each with a series of waypoints. This detailed planning is essential for executing tasks with precision and efficiency.

The `Task` message structure is also used in other packages, such as the RTDE (Real-Time Data Exchange) library, which facilitates remote control. By organizing the task data in this hierarchical manner, it becomes easier to manage and utilize the information across different components of the system.

### Summary

- **Task**: Represents an overarching task, composed of multiple stages.
- **Stage**: Represents a single stage within a task, composed of multiple waypoints.
- **Waypoint**: Represents a specific point within a stage, defined by the states of multiple joints.
- **JointState**: Represents the state of an individual joint, including its name and position.

## License
This project is licensed under the Apache 2.0 License. See the [LICENSE](/LICENSE) file for details.
