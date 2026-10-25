import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description(): 
    pkg_share = get_package_share_directory('pkg1')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'bringup.launch.py')
        )
    )

    trigger_square = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal', 
                    '/draw_square', 
                    'c_interface/action/Square', 
                    '{side_length: 4.0}'  # <-- Removed the extra double quotes wrapping the braces
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        bringup_launch,
        trigger_square
    ])