from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='d_bot_localization_server',
            executable='initial_pose_pub',
            output='screen'),
    ])