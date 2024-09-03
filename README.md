# pointcloud_tools

Library and ROS nodes for manipulating and analysing pointclouds.

## Prerequisites

* Eigen3
* PCL (`apt` install, comes default with `ros-noetic-desktop-full`)
* [convert](https://github.com/willat343/convert/tree/main)
* [mathbox](https://github.com/willat343/mathbox)
* pcl_conversions (`apt` install, comes default with `ros-noetic-desktop-full`)
* pcl_ros (`apt` install, comes default with `ros-noetic-desktop-full`)
* [statistics_msgs](https://github.com/willat343/statistics_msgs) (in catkin workspace)

## pointcloud_tools_ros

### Pointcloud Analyser

Print out useful metadata about an incoming pointcloud stream.

```bash
roslaunch pointcloud_tools pointcloud_analyser.launch input:=<input_topic>
```

### Pointcloud File Converter

Load or save pointclouds from or to files.

```bash
rosrun pointcloud_tools pointcloud_file_converter
```

Save example:
```bash
rosservice call /pointcloud_file_converter/save_pointcloud "filepath: 'my_pointcloud.pcd'
topic: '/points'
timeout: 0.0
is_pcl_type: false"
```

Load example:
```bash
rosservice call /pointcloud_file_converter/load_pointcloud "filepath: 'my_pointcloud.pcd'
topic: '/points'
frame_id: 'lidar'
latch: true
as_pcl_type: false"
```

### Pointcloud Compute Distances

Compute per-point euclidean distances between two pointclouds using nearest neighbour and publish a new pointcloud with this distance as a field.

```bash
roslaunch pointcloud_tools pointcloud_analyser.launch input_source:=<input_source_topic> input_target:=<input_source_target>
```

## Changelog

* [04.07.2024] Migrated from [serpent](https://github.com/jpl-eels/serpent/tree/develop)
