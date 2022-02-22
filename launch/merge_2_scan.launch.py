import os
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch_ros.actions
import pathlib


def generate_launch_description():
    
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ros2_laser_scan_merger',
            executable='ros2_laser_scan_merger',
            output='screen',
            parameters=[
                {"scan_topic_1" : "A2/scan"},
                {"scan_topic_2" : "S1/scan"},
            ],
         ),
    ])
