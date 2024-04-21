from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    config1 = os.path.join(
    get_package_share_directory('d_bot_driver'),
    'config',
    'pc2laserright.yaml')

    config2 = os.path.join(
    get_package_share_directory('d_bot_driver'),
    'config',
    'pc2laserleft.yaml')

    return LaunchDescription([

        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan1',
            remappings=[('cloud_in', '/right_camera/depth/points'),
                        ('scan', '/scan_right')],
            parameters=[config1],
            output='screen'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan2',
            remappings=[('cloud_in', '/left_camera/depth/points'),
                        ('scan', '/scan_left')],
            parameters=[config2],
            output='screen'
        )
    ])