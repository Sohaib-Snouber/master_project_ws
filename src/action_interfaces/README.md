# Action Interfaces Package

The `action_interfaces` package defines the action interfaces used within the master project. These action interfaces facilitate communication for long-running tasks, providing goal, feedback, and result messages.

## ProcessTask Action

The `ProcessTask.action` file defines the action interface for processing tasks. This interface allows for initiating a task with various requests, receiving feedback during the task's execution, and obtaining the result once the task is complete.

### Action Definition

The `ProcessTask.action` file is structured as follows:

```plaintext
# Goal
bool request_open_gripper
bool request_close_gripper
bool request_greeting
bool move_robot
bool check_robot_status
---
# Result
string greeting_message
int32 goal_id
---
# Feedback
bool robot_moving
```


### Usage

This action interface can be used to define servers and clients for processing tasks. The clients can send a goal (with various requests) to the server, which will process the task, provide feedback during execution, and return the result upon completion.

### Summary

- **Goal**: Contains various task requests.
- **Result**: Contains the sequence of actions performed, a greeting message, and the goal ID.
- **Feedback**: Provides robot movement status during task execution.

