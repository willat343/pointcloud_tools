# pointcloud_tools_ros

ROS nodes for manipulating and analysing pointclouds.

## Prerequisites

System Libraries installable via `apt`:
* Eigen3
* PCL (`apt` install, comes default with `ros-noetic-desktop-full`)
* pcl_conversions (`apt` install, comes default with `ros-noetic-desktop-full`)
* pcl_ros (`apt` install, comes default with `ros-noetic-desktop-full`)

Libraries which must be cloned or linked in the catkin workspace:
* [convert](https://github.com/willat343/convert/tree/main)
* [mathbox](https://github.com/willat343/mathbox)
* [statistics_msgs](https://github.com/willat343/statistics_msgs) (in catkin workspace)

## Build

```bash
cd /path/to/catkin_ws/src
ln -s /path/to/pointcloud_tools
cd /path/to/catkin_ws
catkin build pointcloud_tools_ros
```

## Usage

### Pointcloud Analyser

Print out useful metadata about an incoming pointcloud stream.

```bash
roslaunch pointcloud_tools_ros pointcloud_analyser.launch input:=<input_topic>
```

### Pointcloud File Converter

Load or save pointclouds from or to files.

```bash
rosrun pointcloud_tools_ros pointcloud_file_converter
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
roslaunch pointcloud_tools_ros pointcloud_compute_distances.launch input_source:=<input_source_topic> input_target:=<input_source_target>
```
