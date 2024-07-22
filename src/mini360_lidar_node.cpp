// Copyright <2020> [Copyright rossihwang@gmail.com]

#include <rclcpp/rclcpp.hpp>

#include "mini360_lidar.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Mini360Lidar>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
