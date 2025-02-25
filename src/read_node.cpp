/*
 * read_node.cpp
 *
 *  Created on: Aug 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "rclcpp/rclcpp.hpp"
#include "point_cloud_io/Read.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<point_cloud_io::Read>());
  rclcpp::shutdown();
  return 0;
}
