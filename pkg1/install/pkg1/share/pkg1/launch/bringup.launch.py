from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Start the turtlesim simulation node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        
        # 2. Start your controller node and pass speed parameters
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