from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    sync_rviz_pose = Node(
        package='full_drive',
        executable='sync_rviz_pose',
        name='sync_rviz_pose',
        output='screen'
    )

    planning_scene_node = Node(
        package='full_drive',
        executable='planning_scene_node',
        name='planning_scene_node',
        output='screen'
    )

    action_server_node = Node(
        package='full_drive',
        executable='action_server_node',
        name='action_server_node',
        output='screen'
    )

    return LaunchDescription([
        sync_rviz_pose,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=sync_rviz_pose,
                on_exit=[planning_scene_node, action_server_node],
            )
        ),
    ])
