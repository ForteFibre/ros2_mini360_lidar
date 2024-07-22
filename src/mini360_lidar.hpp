// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include <io_context/io_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <serial_driver/serial_driver.hpp>

class Mini360Lidar : public rclcpp::Node
{
public:
  Mini360Lidar(const rclcpp::NodeOptions & options);
  Mini360Lidar(const rclcpp::NodeOptions & options, drivers::common::IoContext & ctx);

  ~Mini360Lidar();

private:
  enum class State { SYNC1, SYNC2, SYNC3, SYNC4, SPEED, START, DATA, END, CRC, PACK_FIN };

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  std::shared_ptr<drivers::common::IoContext> io_context_;
  drivers::serial_driver::SerialDriver serial_driver_;

  std::string frame_id_;
  std::string port_;

  State state_;

  uint8_t buffer_[56];
  size_t buffer_index_;

  std::vector<float> ranges_;
  std::vector<float> intensities_;

  void parse(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred);
  void reset_data();
};