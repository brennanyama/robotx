# SLAM changelog/instructions



### Changelog

* 10/8/2016: Created launch file that uses the tf, urg_node, and hector_mapping packages. Uses only laser data from LIDAR, no IMU or GPS integration yet. Only 2D currently.


### Instructions

1. install nesassary packages
urg_node: `sudo apt-get install ros-indigo-urg-node`
hector_mapping: `sudo apt-get install ros-indigo-hector-mapping`
hector_slam: `sudo apt-get install ros-indigo-hector-slam`
2. create new package in your catkin workspace named SLAM and catkin make
example: from catkin home, `catkin_make_pkg package name`
3. Add launch file and build catkin workspace
4. If you cannot launch using roslaunch in terminal, add catkin source to bashrc, 
`gedit ~/.bashrc`
 at end of file add `source ~/catkin_ws/devel/setup.bash`
5. launch slamming.launch from terminal after opening ROScore
`roscore
 roslaunch SLAM slamming.launch`
6. In RVIZ, check the following:
 * Fixed frame = map (or base_link)
 * add Grid, reference frame: <Fixed Frame>
 * add Map, Topic /map
 * add Path, topic: /trajectory
 * add Pose, topic: /slam_out_pose

### TODO:

1. Incorporate IMU and add tf for odom frame to get a more accurate reading of surroundings
2. Check /OccupancyGrid data, check compatibility with D* Lite
3. 3D vizualization?!?

### Troubleshooting

blah
