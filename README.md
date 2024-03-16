# ros2_laser_scan_merger
![laser scan merger configurator](https://github.com/mich1342/ros2_laser_scan_merger/blob/main/LidarCallbration.png)
A full c++ based ros2 package to merge several laserscan / lidars topics by creating a new virtual laserscan topic. Each source laserscan could be configure via the parameter to determine the heading of each source laserscan and the relative position of each source laserscan to the virtual laserscan.

## Prerequisite
1. ROS2 (Tested on Humble)
2. Your laserscans driver (Tested using RPLIDAR S1 and RPLIDAR S1)
3. RVIZ2
4. RQT
5. [Pointcloud to Laserscan](https://github.com/ros-perception/pointcloud_to_laserscan)

## How to use 
1. Clone the repo to your ros2 workspace
```bash
git clone https://github.com/mich1342/ros2_laser_scan_merger.git
```
2. Edit the topic name in the launch file if needed

3. Build and Source
```bash
colcon build && source install/setup.bash
```
4. Launch the package
- To launch without visualizer
```bash
ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py
```
- To launch with visualizer (RVIZ2)
```bash
ros2 launch ros2_laser_scan_merger visualize_merge_2_scan.launch.py
```
Both of the launch file already integrated with the pointclound_to_laserscan package 

5. Open RQT to set the parameter
```bash
rqt
```

## Available Parameters

All parameters are being set in the `params.yaml` file inside the `config` directory.

All parameters similar for the first and second lidar data. The `{x}` marks means the index pattern for the lidar data. Example: `show{x}` means `show1` and `show2` for 2 lidar configuration.

| Parameter Name | Default Value | Description |
|----------------|---------------|-------------|
| scanTopic{x} | /lidar_1/scan <br/> /lidar_2/scan |  laser scan or lidar topic |
| show{x} | true | set as `true` to include the first lidar data or `false` to hide the specific lidar data |
| flip{x} | false | set as `true` for upside down lidar installation |
| laser{x}AngleMax | 180 | maximum angle in degree of the lidar data that are being used for the final merged result, usefull to hide some part of the lidars data. will highly depends on each lidar specification |
| laser{x}AngleMin | -180 | minimum angle in degree of the lidar data that are being used for the final merged result, usefull to hide some part of the lidars data. will highly depends on each lidar specification |
| inverse{x} | false | set as `true` to inverse the hidden lidar data based on the `laser{x}AngleMax` and `laser{x}AngleMin` value |
| laser{x}Alpha | 0 | angular offset of the lidar data |
| laser{x}XOff | -0.3 | linar offset of the lidar data in x axis |
| laser{x}YOff | -0.475 | linar offset of the lidar data in y axis |
| laser{x}ZOff | 0.176 | linar offset of the lidar data in z axis |
| laser{x}B | 0 | set color to the resulted pointclound2 data (0-255) |
| laser{x}G | 0 | set color to the resulted pointclound2 data (0-255) |
| laser{x}R | 255 | set color to the resulted pointclound2 data (0-255) |
| pointCloudTopic | cloud_in | pointcloud2 published topic (adjusted to `pointcloud_to_laserscan` package) |
| pointCloutFrameId | laser | frame id of the pointcloud2 published data |
