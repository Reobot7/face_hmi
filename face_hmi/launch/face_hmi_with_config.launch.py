#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Launch face_hmi with configuration files."""
    
    # Get package share directory
    pkg_share = FindPackageShare('face_hmi')
    
    # Configuration file paths
    default_config = PathJoinSubstitution([pkg_share, 'config', 'default.yaml'])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to configuration YAML file'
        ),

        Node(
            package='face_hmi',
            executable='face_hmi',
            name='face_hmi',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),
    ])
