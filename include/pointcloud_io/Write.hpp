/*
 * Write.hpp
 *
 *  Created on: Nov 13, 2015
 *      Author: Remo Diethelm
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS2
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pointcloud_io {

class Write : public rclcpp::Node {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  explicit Write();

  /*!
   * Destructor.
   */
  virtual ~Write() = default;

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Point cloud callback function
   * @param cloud point cloud message.
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2& cloud);

  //! ROS node handle.
  //TODO remove ros::NodeHandle nodeHandle_;

  //! Point cloud subscriber.
  //TODO remove ros::Subscriber pointCloudSubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSubscriber_;

  //! Point cloud topic to subscribe to.
  std::string pointCloudTopic_;

  //! Path to the point cloud folder.
  std::string folderPath_;

  //! Point cloud file prefix.
  std::string filePrefix_;

  //! Point cloud file ending.
  std::string fileEnding_;

  //! Point cloud counter.
  unsigned int counter_ = 0;

  //! Settings for generating file name.
  bool addCounterToPath_ = true;
  bool addFrameIdToPath_ = false;
  bool addStampSecToPath_ = false;
  bool addStampNSecToPath_ = false;
};

}  // namespace point_cloud_io
