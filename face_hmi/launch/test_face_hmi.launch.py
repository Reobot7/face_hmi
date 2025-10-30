#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Launch face_hmi with test publisher for demonstration."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_optical_frame',
            description='The TF frame of the camera.'
        ),
        DeclareLaunchArgument(
            'fov_x_deg',
            default_value='90.0',
            description='Horizontal field of view in degrees.'
        ),
        DeclareLaunchArgument(
            'fov_y_deg',
            default_value='60.0',
            description='Vertical field of view in degrees.'
        ),
        DeclareLaunchArgument(
            'fullscreen',
            default_value='false',  # Windowed mode for testing
            description='Whether to run in fullscreen mode.'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='60',
            description='Frames per second for rendering.'
        ),

        # Face HMI display node
        Node(
            package='face_hmi',
            executable='face_hmi',
            name='face_hmi',
            output='screen',
            parameters=[
                {'camera_frame': LaunchConfiguration('camera_frame')},
                {'fov_x_deg': LaunchConfiguration('fov_x_deg')},
                {'fov_y_deg': LaunchConfiguration('fov_y_deg')},
                {'fullscreen': LaunchConfiguration('fullscreen')},
                {'fps': LaunchConfiguration('fps')},
            ]
        ),

        # Test publisher node
        Node(
            package='face_hmi',
            executable='test_publisher',
            name='face_hmi_test_publisher',
            output='screen'
        ),
    ])
