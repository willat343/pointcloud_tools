#include "pointcloud_tools_ros/pointcloud_analyser.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <sstream>

#include "pointcloud_tools/pclpointcloud2_utilities.hpp"

namespace pct {

statistics_msgs::SummaryStatistics statistics(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    statistics_msgs::SummaryStatistics statistics_;
    statistics_.label = field.name;
    statistics_.min = min<double>(pointcloud, field);
    statistics_.max = max<double>(pointcloud, field);
    statistics_.mean = mean<double>(pointcloud, field);
    statistics_.variance = variance<double>(pointcloud, field, statistics_.mean);
    statistics_.count = size_points(pointcloud);
    return statistics_;
}

statistics_msgs::SummaryStatisticsArray statistics(const pcl::PCLPointCloud2& pointcloud) {
    statistics_msgs::SummaryStatisticsArray statistics_array;
    statistics_array.header = pcl_conversions::fromPCL(pointcloud.header);
    for (const pcl::PCLPointField& field : pointcloud.fields) {
        statistics_array.statistics.emplace_back(statistics(pointcloud, field));
    }
    return statistics_array;
}

std::string summary(const pcl::PCLPointCloud2& pointcloud) {
    statistics_msgs::SummaryStatisticsArray statistics_ = statistics(pointcloud);
    return summary(pointcloud, statistics_.statistics);
}

std::string summary(const pcl::PCLPointCloud2& pointcloud,
        const std::vector<statistics_msgs::SummaryStatistics>& statistics_) {
    if (pointcloud.fields.size() != statistics_.size()) {
        throw std::runtime_error("Fields and statistics had different sizes (" +
                                 std::to_string(pointcloud.fields.size()) + " and " +
                                 std::to_string(statistics_.size()) + ").");
    }
    std::stringstream ss;
    ss << "Pointcloud (" << pointcloud.header.seq << ", " << pointcloud.header.stamp << ", "
       << pointcloud.header.frame_id << ") @ " << std::fixed << timestamp_as_seconds(pointcloud)
       << "\n\tsize: h = " << pointcloud.height << ", w = " << pointcloud.width;
    for (std::size_t i = 0; i < pointcloud.fields.size(); ++i) {
        // Even though its inefficient, recompute max and min strings without the cast to double
        ss << "\n\t" << to_string(pointcloud.fields[i]) << "\n\t\tmax: " << max_str(pointcloud, pointcloud.fields[i])
           << ", min: " << min_str(pointcloud, pointcloud.fields[i])
           << ", mean: " << std::to_string(statistics_[i].mean)
           << ", variance: " << std::to_string(statistics_[i].variance);
    }
    return ss.str();
}

PointcloudAnalyser::PointcloudAnalyser()
    : nh("~") {
    subscriber = nh.subscribe<sensor_msgs::PointCloud2>("input", 100, &PointcloudAnalyser::analyse, this);
    statistics_array_publisher = nh.advertise<statistics_msgs::SummaryStatisticsArray>("output_statistics", 1);
}

void PointcloudAnalyser::analyse(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert pointcloud from ROS
    auto pointcloud = boost::make_shared<pcl::PCLPointCloud2>();
    pcl_conversions::toPCL(*msg, *pointcloud);

    // Print information with statistics
    statistics_msgs::SummaryStatisticsArray statistics_array = statistics(*pointcloud);
    ROS_INFO_STREAM(summary(*pointcloud, statistics_array.statistics));

    // Publish statistics
    statistics_array_publisher.publish(statistics_array);

    // Print normal information
    if (has_field(*pointcloud, "normal_x") && has_field(*pointcloud, "normal_y") &&
            has_field(*pointcloud, "normal_z")) {
        const int unnormalised_normals = check_normals(*pointcloud);
        if (unnormalised_normals > 0) {
            ROS_WARN_STREAM(unnormalised_normals << "/" << size_points(*pointcloud) << " normals are unnormalised");
        } else {
            ROS_INFO_STREAM("All point normals are normalised.");
        }
    }
}

}
