# pointcloud_tools_catkin

Catkin wrapper for pointcloud_tools, which allows the library to be built and installed to a catkin workspace.

## Build

```bash
cd /path/to/catkin_ws/src
ln -s /path/to/pointcloud_tools
cd /path/to/catkin_ws
catkin build pointcloud_tools_catkin
```

Downstream packages built within the catkin workspace can depend on `pointcloud_tools_catkin` and can find the core library with `find_package(pointcloud_tools REQUIRED)`.
