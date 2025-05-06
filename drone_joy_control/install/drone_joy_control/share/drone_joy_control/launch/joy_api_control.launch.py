#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('drone_joy_control')
    config_file = os.path.join(pkg_dir, 'config', 'joy_config.yaml')
    
    # Launch arguments
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='drone0',
        description='ID of the drone to control'
    )
    
    # Joy node - reads from joystick hardware
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[config_file]
    )
    
    # Joystick controller node using the drone API
    joy_controller_node = Node(
        package='drone_joy_control',
        executable='drone_joy_api_controller.py',
        name='drone_joy_api_controller',
        parameters=[
            config_file,
            {'drone_id': LaunchConfiguration('drone_id')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        drone_id_arg,
        joy_node,
        joy_controller_node
    ]) 