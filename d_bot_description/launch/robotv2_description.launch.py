import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import substitutions
from ament_index_python.packages import get_package_share_directory
from launch.launch_context import LaunchContext
from typing import List
import launch_ros.descriptions

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


ARGUMENTS = [
    DeclareLaunchArgument(
        'model',
        default_value='d_bot_description',
        description='Robot Model'
    )
    ]	

def generate_launch_description():
    
    urdf_path = os.path.join(get_package_share_directory('d_bot_description'), 'urdf')
    urdf_path=urdf_path + '/d_bot_full.urdf.xacro'
    world = os.path.join(
        get_package_share_directory('d_bot_gazebo'),
        'worlds',
        'cafe.world'
    )
    # RViz 
    rviz_config_file = get_package_share_directory('d_bot_description') + "/rviz/urdf.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file])
    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )
    robot_gazebo = Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-topic', '/robot_description', '-entity', 'd_bot_description', '-x', '0', '-y', '0', '-z', '1.0',
                '-R', '0', '-P', '0', '-Y', '0',
                ],
            output='screen')
    
    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[
                                    {'robot_description': launch_ros.descriptions.ParameterValue(substitutions.Command(['xacro ',urdf_path]), value_type=str)}])

    return LaunchDescription(ARGUMENTS + [gzserver_cmd, gzclient_cmd, robot_state_publisher, robot_gazebo, rviz_node])
