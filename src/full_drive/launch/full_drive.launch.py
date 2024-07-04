from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("robot").to_dict()

    sync_rviz_pose = Node(
        package='full_drive',
        executable='sync_rviz_pose',
        name='sync_rviz_pose',
        output='screen',
        parameters=[moveit_config]
    )

    action_server_node = Node(
        package='full_drive',
        executable='action_server_node',
        name='action_server_node',
        output='screen',
        parameters=[moveit_config]
    )

    return LaunchDescription([
        sync_rviz_pose,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=sync_rviz_pose,
                on_exit=[action_server_node],
            )
        ),
    ])
