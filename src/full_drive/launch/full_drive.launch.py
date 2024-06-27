from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load MoveIt configuration
    moveit_config = MoveItConfigsBuilder("robot").to_dict()

    sync_rviz_pose = Node(
        package='full_drive',
        executable='sync_rviz_pose',
        name='sync_rviz_pose',
        output='screen',
        parameters=[moveit_config]
    )

    planning_scene_node = Node(
        package='full_drive',
        executable='planning_scene_node',
        name='planning_scene_node',
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
    mtc_node = Node(
        package='full_drive',
        executable='mtc_node',
        name='mtc_node',
        output='screen',
        parameters=[moveit_config]
    )

    return LaunchDescription([
        sync_rviz_pose,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=sync_rviz_pose,
                on_exit=[planning_scene_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=planning_scene_node,
                on_start=[mtc_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=mtc_node,
                on_start=[TimerAction(period=5.0, actions=[action_server_node])],
            )
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
