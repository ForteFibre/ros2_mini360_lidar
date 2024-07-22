// Copyright <2020> [Copyright rossihwang@gmail.com]

#include "mini360_lidar.hpp"

#include <cmath>
#include <limits>
#include <rcl_interfaces/msg/parameter.hpp>

constexpr uint8_t kSync1 = 0x55;
constexpr uint8_t kSync2 = 0xaa;
constexpr uint8_t kSync3 = 0x23;
constexpr uint8_t kSync4 = 0x10;
constexpr double kIndexMultiplier = 400.0 / 360;

Mini360Lidar::Mini360Lidar(const rclcpp::NodeOptions & options)
: Node("mini360_lidar_driver", options),
  io_context_(std::make_shared<drivers::common::IoContext>()),
  serial_driver_(*io_context_),
  frame_id_("laser"),
  port_("/dev/ttyUSB0"),
  state_(State::SYNC1),
  buffer_index_(0)
{
  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

  frame_id_ = declare_parameter<std::string>("frame_id", "laser");
  port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");

  reset_data();

  using namespace drivers::serial_driver;
  using namespace std::placeholders;

  SerialPortConfig port_config(230400, FlowControl::NONE, Parity::NONE, StopBits::ONE);

  try {
    serial_driver_.init_port(port_, port_config);
    if (!serial_driver_.port()->is_open()) {
      serial_driver_.port()->open();
      serial_driver_.port()->async_receive(std::bind(&Mini360Lidar::parse, this, _1, _2));
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error creating serial port: %s - %s", port_.c_str(), ex.what());
    return;
  }

  RCLCPP_INFO(get_logger(), "%s is open", port_.c_str());
}

Mini360Lidar::~Mini360Lidar()
{
  if (io_context_) {
    io_context_->waitForExit();
  }
}

void Mini360Lidar::parse(const std::vector<uint8_t> & buffer_raw, const size_t & bytes_transferred)
{
  std::vector<uint8_t> buffer(buffer_raw.begin(), buffer_raw.begin() + bytes_transferred);

  auto it = buffer.begin();
  while (it != buffer.end()) {
    switch (state_) {
      case State::SYNC1:
        if (*it++ == kSync1) {
          state_ = State::SYNC2;
        }
        break;
      case State::SYNC2:
        if (*it++ == kSync2) {
          state_ = State::SYNC3;
        } else {
          state_ = State::SYNC1;
        }
        break;
      case State::SYNC3:
        if (*it++ == kSync3) {
          state_ = State::SYNC4;
        } else {
          state_ = State::SYNC1;
        }
        break;
      case State::SYNC4:
        if (*it++ == kSync4) {
          state_ = State::DATA;
        } else {
          state_ = State::SYNC1;
        }
        break;
      case State::DATA: {
        while (it != buffer.end() && buffer_index_ < 56) {
          buffer_[buffer_index_++] = *it++;
        }

        if (buffer_index_ < 56) {
          break;
        }

        buffer_index_ = 0;

        // TODO: validate crc
        uint16_t u;
        memcpy(&u, buffer_ + 2, sizeof(u));
        auto start_angle = u / 64.f - 640;
        memcpy(&u, buffer_ + 52, sizeof(u));
        auto end_angle = u / 64.f - 640;

        float angle_res;
        if (end_angle < start_angle) {
          angle_res = (end_angle + 360 - start_angle) / 8.0;
        } else {
          angle_res = (end_angle - start_angle) / 8.0;
        }

        for (int i = 0; i < 16; ++i) {
          uint16_t range;
          memcpy(&range, buffer_ + 4 + 3 * i, sizeof(range));
          uint8_t intensity = buffer_[4 + 3 * i + 2];

          double measured_angle = start_angle + angle_res * i;
          int angle_index = std::round(measured_angle * kIndexMultiplier);
          angle_index %= 400;
          angle_index = 399 - angle_index;

          range = range & 0x3fff;

          ranges_[angle_index] = static_cast<float>(range) / 1000.0;
          intensities_[angle_index] = static_cast<float>(intensity);
        }

        if (end_angle < start_angle) {
          sensor_msgs::msg::LaserScan message;
          message.header.stamp = now();
          message.header.frame_id = frame_id_;
          message.angle_increment = (2.0 * M_PI) / 400.0;
          message.angle_min = 0.0;
          message.angle_max = 2.0 * M_PI - message.angle_increment;
          message.scan_time = 0.001;
          message.range_min = 0.08;  // camsense x1 spec
          message.range_max = 8;     // camsense x1 spec
          message.ranges = ranges_;
          message.intensities = intensities_;
          scan_pub_->publish(message);
          reset_data();
        }

        state_ = State::SYNC1;
        break;
      }
      default:
        break;
    }
  }
}

void Mini360Lidar::reset_data()
{
  ranges_.resize(400);
  intensities_.resize(400);

  for (int i = 0; i < 400; ++i) {
    ranges_[i] = std::numeric_limits<double>::quiet_NaN();
    intensities_[i] = 0;
  }
}
