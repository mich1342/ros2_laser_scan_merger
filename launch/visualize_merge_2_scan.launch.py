#
#   created by: Michael Jonathan (mich1342)
#   github.com/mich1342
#   24/2/2022
#

from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('ros2_laser_scan_merger'),
        'rviz',
        'ros2_laser_scan_merge.rviz'
    )

    config = os.path.join(
        get_package_share_directory('ros2_laser_scan_merger'),
        'config',
        'params.yaml'
    )
    return LaunchDescription([       
        launch_ros.actions.Node(
            package='ros2_laser_scan_merger',
            executable='ros2_laser_scan_merger',
            parameters=[config],
            output='screen',
            respawn=True,
            respawn_delay=2,
        ),
        # TF2 for laser to map frame id, optional
        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #         '--frame-id', 'map', '--child-frame-id', 'laser'
        #     ]
        # ),

        # Call pointcloud_to_laserscan package
        launch_ros.actions.Node(
            name='pointcloud_to_laserscan',
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            parameters=[config]
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    ])

