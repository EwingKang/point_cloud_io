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
#include <pointcloud_io/msg/texture_mesh.hpp>
#include <pointcloud_io/srv/get_mesh.hpp>
#include <pointcloud_io/srv/get_texture_mesh.hpp>

namespace pointcloud_io {

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

  /*!
   * Callback from publisher event
   * @return (none)
   */
  void publisherMatchedCbPointcloud(const rclcpp::MatchedInfo &info);
  void publisherMatchedCbMesh(const rclcpp::MatchedInfo &info);
  void publisherMatchedCbTexturedMesh(const rclcpp::MatchedInfo &info);

  //! ROS node handle.

  //! Point cloud message to publish.
  //sensor_msgs::PointCloud2::Ptr pointCloudMessage_;
  sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMessage_;
  shape_msgs::msg::Mesh::SharedPtr meshMessage_;
  pointcloud_io::msg::TextureMesh::SharedPtr texMeshMessage_;

  //! Point cloud publisher.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPublisher_;
  rclcpp::Publisher<shape_msgs::msg::Mesh>::SharedPtr meshPublisher_;
  rclcpp::Publisher<pointcloud_io::msg::TextureMesh>::SharedPtr texMeshPublisher_;

  //! Services
  using ServiceGetMesh = pointcloud_io::srv::GetMesh;
  using ServiceGetTextureMesh = pointcloud_io::srv::GetTextureMesh;
  rclcpp::Service<ServiceGetMesh>::SharedPtr srvGetMesh_;
  rclcpp::Service<ServiceGetTextureMesh>::SharedPtr srvGetTextureMesh_;
  void ServeMesh(
        const std::shared_ptr<ServiceGetMesh::Request> request,
        std::shared_ptr<ServiceGetMesh::Response>      response);
  void ServeTextureMesh(
        const std::shared_ptr<ServiceGetTextureMesh::Request> request,
        std::shared_ptr<ServiceGetTextureMesh::Response>      response);

  //! Timer for publishing the point cloud.
  //ros::Timer timer_;
  rclcpp::TimerBase::SharedPtr timer_;

  //! Path to the point cloud file.
  std::string filePath_;

  //! Point cloud topic to be published at.
  std::string pointCloudTopic_;

  //! Mesh topic to be published at. Only published if it's non-empty
  std::string meshTopic_, texMeshTopic_;

  //! Point cloud frame id.
  std::string pointCloudFrameId_;

  //! Mesh id.
  std::string meshId_;

  //! Scale for the mesh file.
  double scale_ = 1.0;
  //! Rotation for the mesh file
  std::vector<double> ypr_ = {0, 0, 0};
  std::vector<double> offset_ = {0, 0, 0};

  /*!
   * If true, continuous publishing is used.
   * If false, point cloud is only published once.
   */
  bool isContinuouslyPublishing_ = false;


  //! Duration between publishing steps.
  std::chrono::microseconds updateDuration_;

};

}  // namespace pointcloud_io
