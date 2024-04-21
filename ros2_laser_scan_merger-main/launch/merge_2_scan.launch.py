#
#   created by: Michael Jonathan (mich1342)
#   github.com/mich1342
#   24/2/2022
#
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
        'pointcloud_laser.yaml'
    )
    return LaunchDescription([
        
        launch_ros.actions.Node(
            name='ros2_pc_laser_scan_merger',
            package='ros2_laser_scan_merger',
            executable='ros2_laser_scan_merger',
            parameters=[config],
            output='screen',
            respawn=True,
            respawn_delay=2,
        ),
        launch_ros.actions.Node(
            name='pointcloud_to_laserscan3',
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
                        remappings=[('cloud_in', '/merged_cloud'),
                        ('scan', '/pointcloud_scan')],
            parameters=[config]
        )

        
    ])
