import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    d_bot_exploration_dir = get_package_share_directory('d_bot_exploration')
    d_bot_slam_dir = get_package_share_directory('d_bot_slam')
    d_bot_navigation_dir = get_package_share_directory('d_bot_navigation')
    explore_lite_launch = os.path.join(
        get_package_share_directory('explore_lite'), 'launch', 'explore.launch.py'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(d_bot_exploration_dir, 'config', 'explore.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(d_bot_slam_dir, 'launch', 'd_bot_slamtoolbox.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': 'false'
        }.items(),
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(d_bot_navigation_dir, 'launch', 'd_bot_navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    explore_lite_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([explore_lite_launch]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rviz_config_dir = os.path.join(
        d_bot_exploration_dir,
        'rviz',
        'explore.rviz',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # ld.add_action(slam_launch)
    # ld.add_action(nav2_bringup_launch)
    ld.add_action(explore_lite_launch)
    # ld.add_action(rviz_node)

    return ld