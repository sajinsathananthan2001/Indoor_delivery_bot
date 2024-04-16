import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    controller_yaml = os.path.join(get_package_share_directory('d_bot_navigation'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('d_bot_navigation'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('d_bot_navigation'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('d_bot_navigation'), 'config', 'recovery.yaml')

    
    is_explore = DeclareLaunchArgument(
        'is_explore',
        default_value=False,
        description="Indicating whether autonomous mapping or navigation",
    )

    nav_controller = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml])

    nav_planner = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml])
            
    behaviour_server = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[recovery_yaml],)
    
    nav_bt_server = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml])
    lifecycle_server = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator']}])
    
    ld = LaunchDescription()
    ld.add_action(nav_controller)
    ld.add_action(nav_planner)
    ld.add_action(behaviour_server)
    ld.add_action(nav_bt_server)
    ld.add_action(lifecycle_server)

    return ld
