from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('device', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('topic_odom_out', default_value='/odom'),

        Node(
          
            package='roboclaw_ros',
            namespace='roboclaw_ros',
            executable='roboclaw_node_exec',
            name='roboclaw_ros',
            parameters=[
                {"~dev":  LaunchConfiguration('device') ,
                "~topic_odom_out":  LaunchConfiguration('topic_odom_out')  } 
            ], 
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')  ]
        ),

    ])
