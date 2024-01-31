/*
 * Write.cpp
 *
 *  Created on: Nov 13, 2015
 *      Author: Remo Diethelm
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "pointcloud_io/Write.hpp"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pointcloud_io {

Write::Write() :
  rclcpp::Node("mesh_saver"),
  filePrefix_("point_cloud"), fileEnding_("ply")
{
  if (!readParameters()) {
    rclcpp::shutdown();
  }
  using std::placeholders::_1;
  pointCloudSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointCloudTopic_, rclcpp::QoS(1), std::bind(&Write::pointCloudCallback, this, _1) );
  RCLCPP_INFO_STREAM(this->get_logger(), "Subscribed to topic \"" << pointCloudTopic_ << "\".");
}

bool Write::readParameters() {
  // essential parameters
  this->declare_parameter("topic", rclcpp::PARAMETER_STRING);
  this->declare_parameter("folder_path", rclcpp::PARAMETER_STRING);

  this->declare_parameter("file_prefix", rclcpp::PARAMETER_STRING);
  this->declare_parameter("file_ending", rclcpp::PARAMETER_STRING);
  this->declare_parameter("add_counter_to_path", rclcpp::PARAMETER_STRING);
  this->declare_parameter("add_frame_id_to_path", rclcpp::PARAMETER_STRING);
  this->declare_parameter("add_stamp_sec_to_path", rclcpp::PARAMETER_STRING);
  this->declare_parameter("add_stamp_nsec_to_path", rclcpp::PARAMETER_STRING);

  bool allParametersRead = true;
  allParametersRead = this->get_parameter("topic", pointCloudTopic_) && allParametersRead;
  allParametersRead = this->get_parameter("folder_path", folderPath_) && allParametersRead;

  this->get_parameter("file_prefix", filePrefix_);
  this->get_parameter("file_ending", fileEnding_);
  this->get_parameter("add_counter_to_path", addCounterToPath_);
  this->get_parameter("add_frame_id_to_path", addFrameIdToPath_);
  this->get_parameter("add_stamp_sec_to_path", addStampSecToPath_);
  this->get_parameter("add_stamp_nsec_to_path", addStampNSecToPath_);

  if (!allParametersRead) {
    RCLCPP_WARN( this->get_logger(),
        "Could not read all parameters. Typical command-line usage:\n"
        "rosrun pointcloud_io write"
        " _topic:=/my_topic"
        " _folder_path:=/home/user/my_point_clouds"
        " (optional: _file_prefix:=my_prefix"
        " _file_ending:=my_ending"
        " _add_counter_to_path:=true/false"
        " _add_frame_id_to_path:=true/false"
        " _add_stamp_sec_to_path:=true/false"
        " _add_stamp_nsec_to_path:=true/false)");
    return false;
  }

  return true;
}

void Write::pointCloudCallback(const sensor_msgs::msg::PointCloud2& cloud) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Received point cloud with " << cloud.height * cloud.width << " points.");
  std::cout << folderPath_ << std::endl;
  std::stringstream filePath;
  filePath << folderPath_ << "/";
  if (!filePrefix_.empty()) {
    filePath << filePrefix_;
  }
  if (addCounterToPath_) {
    filePath << "_" << counter_;
    counter_++;
  }
  if (addFrameIdToPath_) {
    filePath << "_" << cloud.header.frame_id;
  }
  if (addStampSecToPath_) {
    filePath << "_" << cloud.header.stamp.sec;
  }
  if (addStampNSecToPath_) {
    filePath << "_" << cloud.header.stamp.nanosec;
  }
  filePath << ".";
  filePath << fileEnding_;

  if (fileEnding_ == "ply") {
    // Write .ply file.
    pcl::PointCloud<pcl::PointXYZRGBNormal> pclCloud;
    pcl::fromROSMsg(cloud, pclCloud);

    pcl::PLYWriter writer;
    bool binary = false;
    bool use_camera = false;
    if (writer.write(filePath.str(), pclCloud, binary, use_camera) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Something went wrong when trying to write the point cloud file.");
      return;
    }
  } else if (fileEnding_ == "pcd") {
    // Write pcd file
    pcl::PointCloud<pcl::PointXYZRGBNormal> pclCloud;
    pcl::fromROSMsg(cloud, pclCloud);
    pcl::io::savePCDFile(filePath.str(), pclCloud);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Data format not supported.");
    return;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Saved point cloud to " << filePath.str() << ".");
}

}  // namespace pointcloud_io
