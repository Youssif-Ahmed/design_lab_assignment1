import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='pkg1',
            executable='control',
            name='turtle_controller',
            output='screen',
            parameters=[{
                'max_linear_speed': 2.0,
                'max_angular_speed': 1.5
            }]
        )
    ])