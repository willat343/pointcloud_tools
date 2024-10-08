#include <open3d/geometry/PointCloud.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d_conversions/open3d_conversions.h>
#include <pcl/io/auto_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "pointcloud_tools_ros/pointcloud_file_converter.hpp"

namespace pct {

PointcloudFileConverter::PointcloudFileConverter()
    : nh("~") {
    loader = nh.advertiseService("load_pointcloud", &PointcloudFileConverter::load, this);
    saver = nh.advertiseService("save_pointcloud", &PointcloudFileConverter::save, this);
}

bool PointcloudFileConverter::load(pointcloud_tools_ros::file_to_message::Request& request,
        pointcloud_tools_ros::file_to_message::Response& response) {
    auto it = publishers.find(request.topic);
    if (it == publishers.end()) {
        auto emplace_it = publishers.emplace(request.topic,
                nh.advertise<sensor_msgs::PointCloud2>(request.topic, 1, request.latch));
        if (!emplace_it.second) {
            throw std::runtime_error("Failed to create publisher for topic \'" + request.topic + "'\'");
        }
        it = emplace_it.first;
    }
    open3d::geometry::PointCloud pointcloud;
    if (!open3d::io::ReadPointCloud(request.filepath, pointcloud)) {
        throw std::runtime_error("Failed to read pointcloud from file \'" + request.filepath + "\'");
    }
    sensor_msgs::PointCloud2 msg;
    open3d_conversions::open3dToRos(pointcloud, msg, request.frame_id);
    msg.header.stamp = ros::Time::now();
    it->second.publish(msg);
    response.result_msg = "Published pointcloud (" + std::to_string(pointcloud.points_.size()) +
                          " points) to topic \'" + request.topic + "\'";
    return true;
}

bool PointcloudFileConverter::save(pointcloud_tools_ros::message_to_file::Request& request,
        pointcloud_tools_ros::message_to_file::Response& response) {
    if (request.is_pcl_type) {
        auto msg = ros::topic::waitForMessage<pcl::PCLPointCloud2>(request.topic, nh, ros::Duration(request.timeout));
        if (!msg) {
            throw std::runtime_error("Failed to receive PCL pointcloud on topic \'" + request.topic +
                                     "\' within"
                                     " timeout period of " +
                                     std::to_string(request.timeout) + " seconds");
        }
        int return_code = pcl::io::save(request.filepath, *msg);
        response.result_msg = "Saved PCL pointcloud (" + std::to_string(msg->width * msg->height) + " points) to " +
                              request.filepath + " with return code " + std::to_string(return_code);
    } else {
        auto msg =
                ros::topic::waitForMessage<sensor_msgs::PointCloud2>(request.topic, nh, ros::Duration(request.timeout));
        if (!msg) {
            throw std::runtime_error("Failed to receive pointcloud on topic \'" + request.topic +
                                     "\' within timeout"
                                     " period of " +
                                     std::to_string(request.timeout) + " seconds");
        }
        open3d::geometry::PointCloud pointcloud;
        open3d_conversions::rosToOpen3d(msg, pointcloud);
        if (!open3d::io::WritePointCloud(request.filepath, pointcloud)) {
            throw std::runtime_error("Failed to save pointcloud to file \'" + request.filepath + "\'");
        }
        response.result_msg =
                "Saved pointcloud (" + std::to_string(pointcloud.points_.size()) + " points) to " + request.filepath;
    }
    return true;
}

}
