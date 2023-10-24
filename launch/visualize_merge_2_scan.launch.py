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
    #general parameter for the cloud, do not change
    pointCloudTopic = LaunchConfiguration('pointCloudTopic', default="cloud_in")
    pointCloutFrameId = LaunchConfiguration('pointCloutFrameId', default="laser")
    
    #parameter for the first laserscan, feel free to duplicate and rename for other laserscans
    scanTopic1 = LaunchConfiguration('scanTopic1', default="/lidar_1/scan")
    laser1XOff = LaunchConfiguration('laser1XOff', default=-0.45)
    laser1YOff = LaunchConfiguration('laser1YOff', default=0.24)
    laser1ZOff = LaunchConfiguration('laser1ZOff', default=0.0)
    laser1Alpha = LaunchConfiguration('laser1Alpha', default=45.0)
    laser1AngleMin = LaunchConfiguration('laser1AngleMin', default=-181.0)
    laser1AngleMax = LaunchConfiguration('laser1AngleMax', default=181.0)
    laser1R = LaunchConfiguration('laser1R', default=255)
    laser1G = LaunchConfiguration('laser1G', default=0)
    laser1B = LaunchConfiguration('laser1B', default=0)
    show1 = LaunchConfiguration('show1', default=True)

    #parameter for the second laserscan, feel free to duplicate and rename for other laserscans
    scanTopic2 = LaunchConfiguration('scanTopic2', default="/lidar_2/scan")
    laser2XOff = LaunchConfiguration('laser2XOff', default=0.315)
    laser2YOff = LaunchConfiguration('laser2YOff', default=-0.24)
    laser2ZOff = LaunchConfiguration('laser2ZOff', default=0.0)
    laser2Alpha = LaunchConfiguration('laser2Alpha', default=225.0)
    laser2AngleMin = LaunchConfiguration('laser2AngleMin', default=-181.0)
    laser2AngleMax = LaunchConfiguration('laser2AngleMax', default=181.0)
    laser2R = LaunchConfiguration('laser2R', default=0)
    laser2G = LaunchConfiguration('laser2G', default=0)
    laser2B = LaunchConfiguration('laser2B', default=255)
    show2 = LaunchConfiguration('show2', default=True)
    
    rviz_config_dir = os.path.join(
        get_package_share_directory('ros2_laser_scan_merger'),
        'rviz',
        'ros2_laser_scan_merge.rviz')
        
    return LaunchDescription([
        DeclareLaunchArgument(
            'pointCloudTopic',
            default_value=pointCloudTopic,
            description='desc',
        ),
        DeclareLaunchArgument(
            'pointCloutFrameId',
            default_value=pointCloutFrameId,
            description='desc',
        ),

        DeclareLaunchArgument(
            'scanTopic1',
            default_value=scanTopic1,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1XOff',
            default_value=laser1XOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1YOff',
            default_value=laser1YOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1ZOff',
            default_value=laser1ZOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1Alpha',
            default_value=laser1Alpha,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1AngleMin',
            default_value=laser1AngleMin,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1AngleMax',
            default_value=laser1AngleMax,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1R',
            default_value=laser1R,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1G',
            default_value=laser1G,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1B',
            default_value=laser1B,
            description='desc',
        ),
        DeclareLaunchArgument(
            'show1',
            default_value=show1,
            description='desc',
        ),

        DeclareLaunchArgument(
            'scanTopic2',
            default_value=scanTopic2,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2XOff',
            default_value=laser2XOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2YOff',
            default_value=laser2YOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2ZOff',
            default_value=laser2ZOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2Alpha',
            default_value=laser2Alpha,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2AngleMin',
            default_value=laser2AngleMin,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2AngleMax',
            default_value=laser2AngleMax,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2R',
            default_value=laser2R,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2G',
            default_value=laser2G,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2B',
            default_value=laser2B,
            description='desc',
        ),
        DeclareLaunchArgument(
            'show2',
            default_value=show2,
            description='desc',
        ),
        
        launch_ros.actions.Node(
            package='ros2_laser_scan_merger',
            executable='ros2_laser_scan_merger',
            parameters=[{
                'pointCloudTopic' : pointCloudTopic,
                'pointCloutFrameId' : pointCloutFrameId,
                'scanTopic1' : scanTopic1,
                'laser1XOff' : laser1XOff,
                'laser1YOff' : laser1YOff,
                'laser1ZOff' : laser1ZOff,
                'laser1Alpha' : laser1Alpha,
                'laser1AngleMin' : laser1AngleMin,
                'laser1AngleMax' : laser1AngleMax,
                'laser1R' : laser1R,
                'laser1G' : laser1G,
                'laser1B' : laser1B,
                'show1' : show1,
                'scanTopic2' : scanTopic2,
                'laser2XOff' : laser2XOff,
                'laser2YOff' : laser2YOff,
                'laser2ZOff' : laser2ZOff,
                'laser2Alpha' : laser2Alpha,
                'laser2AngleMin' : laser2AngleMin,
                'laser2AngleMax' : laser2AngleMax,
                'laser2R' : laser2R,
                'laser2G' : laser2G,
                'laser2B' : laser2B,
                'show2' : show2,
            }],
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
            parameters=[{
                'target_frame': pointCloutFrameId,
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    ])

