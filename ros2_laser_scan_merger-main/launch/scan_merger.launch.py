import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros2_laser_scan_merger'),
        'config',
        'laser_merge.yaml'
    )
    return LaunchDescription([
        
        launch_ros.actions.Node(
            name='ros2_laser_scan_merger',
            package='ros2_laser_scan_merger',
            executable='ros2_laser_scan_merger',
            parameters=[config],
            output='screen',
            respawn=True,
            respawn_delay=2,
        ),
        launch_ros.actions.Node(
            name='pointcloud_to_laserscan4',
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
                        remappings=[('cloud_in', '/laser_merged_cloud'),
                        ('scan', '/scan_merged')],
            parameters=[config]
        )

        
    ])
