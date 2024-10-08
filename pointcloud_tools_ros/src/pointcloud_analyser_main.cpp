#include <ros/ros.h>

#include "pointcloud_tools_ros/pointcloud_analyser.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_analyser");
    pct::PointcloudAnalyser pointcloud_analyser;
    ros::spin();
    return 0;
}
