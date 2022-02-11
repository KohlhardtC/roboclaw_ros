from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roboclaw_ros',
            namespace='roboclaw_ros',
            executable='roboclaw_node_exec',
            name='roboclaw_ros',
            parameters=[
                {"~dev": "/dev/ttyUSB0"} 
            ], 
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

    ])
