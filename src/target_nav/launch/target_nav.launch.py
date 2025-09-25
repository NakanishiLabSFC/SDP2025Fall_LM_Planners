#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('target_nav')
    
    # Default object map path
    default_object_map_path = os.path.join(pkg_share, 'config', 'object_map.yaml')
    
    # Declare launch arguments
    declare_object_map_path = DeclareLaunchArgument(
        'object_map_path',
        default_value=default_object_map_path,
        description='Path to object map YAML file'
    )
    
    declare_goal_timeout = DeclareLaunchArgument(
        'goal_timeout_sec',
        default_value='120.0',
        description='Navigation goal timeout in seconds'
    )
    
    declare_max_retries = DeclareLaunchArgument(
        'max_retries',
        default_value='1',
        description='Maximum number of retries for failed navigation'
    )
    
    declare_approach_offset = DeclareLaunchArgument(
        'approach_offset',
        default_value='0.0',
        description='Offset distance from target for approach behavior'
    )
    
    # Create the target_nav node
    target_nav_node = Node(
        package='target_nav',
        executable='target_nav_node',
        name='target_nav',
        output='screen',
        parameters=[{
            'object_map_path': LaunchConfiguration('object_map_path'),
            'goal_timeout_sec': LaunchConfiguration('goal_timeout_sec'),
            'max_retries': LaunchConfiguration('max_retries'),
            'approach_offset': LaunchConfiguration('approach_offset'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        declare_object_map_path,
        declare_goal_timeout,
        declare_max_retries,
        declare_approach_offset,
        target_nav_node,
    ])