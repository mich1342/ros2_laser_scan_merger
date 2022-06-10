# ros2_laser_scan_merger
![laser scan merger configurator](https://github.com/mich1342/ros2_laser_scan_merger/blob/main/LidarCallbration.png)
A full c++ based ros2 package to merge several laserscan / lidars topics by creating a new virtual laserscan topic. Each source laserscan could be configure via the parameter to determine the heading of each source laserscan and the relative position of each source laserscan to the virtual laserscan.

## Prerequisite
1. ROS2 (Tested on Foxy)
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
5. Launch the Pointcloud to Laserscan nodes

*Make sure that your laserscans topics already published before launch anything from this package <br />
6. Open RQT to set the parameter
```bash
rqt
```
