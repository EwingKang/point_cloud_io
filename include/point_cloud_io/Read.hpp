/*
 * Read.hpp
 *
 *  Created on: Aug 7, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shape_msgs/msg/mesh.hpp>

namespace point_cloud_io {

class Read : public rclcpp::Node {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  explicit Read();

  /*!
   * Destructor.
   */
  virtual ~Read() = default;

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Initializes node.
   */
  void initialize();

  /*!
   * Read the point cloud from a .ply or .vtk file.
   * @param filePath the path to the .ply or .vtk file.
   * @param pointCloudFrameId the id of the frame of the point cloud data.
   * @return true if successful.
   */
  bool readFile(const std::string& filePath, const std::string& pointCloudFrameId);

  /*!
   * Timer callback function.
   * @param timerEvent the timer event.
   */
  void timerCallback();

  /*!
   * Publish the point cloud as a PointCloud2.
   * @return true if successful.
   */
  bool publish();

  //! ROS node handle.

  //! Point cloud message to publish.
  //sensor_msgs::PointCloud2::Ptr pointCloudMessage_;
  sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMessage_;
  shape_msgs::msg::Mesh::SharedPtr meshMessage_;

  //! Point cloud publisher.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPublisher_;
  rclcpp::Publisher<shape_msgs::msg::Mesh>::SharedPtr meshPublisher_;

  //! Timer for publishing the point cloud.
  //ros::Timer timer_;
  rclcpp::TimerBase::SharedPtr timer_;

  //! Path to the point cloud file.
  std::string filePath_;

  //! Point cloud topic to be published at.
  std::string pointCloudTopic_;

  //! Mesh topic to be published at. Only published if it's non-empty
  std::string meshTopic_;

  //! Point cloud frame id.
  std::string pointCloudFrameId_;

  //! Scale for the mesh file.
  double scale_;

  /*!
   * If true, continuous publishing is used.
   * If false, point cloud is only published once.
   */
  bool isContinuouslyPublishing_ = false;


  //! Duration between publishing steps.
  std::chrono::microseconds updateDuration_;

};

}  // namespace point_cloud_io
