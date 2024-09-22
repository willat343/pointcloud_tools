#ifndef POINTCLOUD_TOOLS_POINTCLOUD_ANALYSER_HPP
#define POINTCLOUD_TOOLS_POINTCLOUD_ANALYSER_HPP

#include <pcl/PCLPointCloud2.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <statistics_msgs/SummaryStatistics.h>
#include <statistics_msgs/SummaryStatisticsArray.h>

#include <string>

namespace pct {

statistics_msgs::SummaryStatistics statistics(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

statistics_msgs::SummaryStatisticsArray statistics(const pcl::PCLPointCloud2& pointcloud);

std::string summary(const pcl::PCLPointCloud2& pointcloud);

std::string summary(const pcl::PCLPointCloud2& pointcloud,
        const std::vector<statistics_msgs::SummaryStatistics>& statistics);

class PointcloudAnalyser {
public:
    PointcloudAnalyser();

private:
    void analyse(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // ROS objects
    ros::NodeHandle nh;
    ros::Publisher statistics_array_publisher;
    ros::Subscriber subscriber;
};

}

#endif
