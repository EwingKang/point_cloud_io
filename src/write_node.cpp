/*
 * write_node.cpp
 *
 *  Created on: Nov 13, 2015
 *      Author: Remo Diethelm
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "rclcpp/rclcpp.hpp"
#include "pointcloud_io/Write.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud_io::Write>());
  rclcpp::shutdown();
  return 0;
}
