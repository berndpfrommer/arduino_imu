// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARDUINO_IMU__ARDUINO_IMU_HPP_
#define ARDUINO_IMU__ARDUINO_IMU_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <string>
#include <thread>
#include <vector>

namespace arduino_imu
{
class ArduinoImu : public rclcpp::Node
{
public:
  using Imu = sensor_msgs::msg::Imu;
  using MagneticField = sensor_msgs::msg::MagneticField;
  explicit ArduinoImu(const rclcpp::NodeOptions & options);
  ~ArduinoImu();

private:
  enum state_t {
    INITIALIZING = 0,
    GET_MESSAGE_TYPE = 1,
    GET_PAYLOAD = 2,
    GET_CRC = 3
  };
  void receivedCallback(uint8_t msgType, const std::vector<uint8_t> & data);
  void serialThread();
  bool openSerialPort();
  void processSerialBytes(const uint8_t * buf, size_t len);
  void parseImuMessage(const std::vector<uint8_t> & data);
  // ------------------------  variables ------------------------------
  rclcpp::Publisher<Imu>::SharedPtr imuPub_;
  Imu imuMsg_;
  rclcpp::Publisher<MagneticField>::SharedPtr magPub_;
  MagneticField magMsg_;
  std::string device_;
  uint32_t inCount_{0};
  rclcpp::Time lastStatTime_;
  int serialFD_{-1};
  int baud_{500000};
  state_t serialState_{INITIALIZING};
  bool keepRunning_{true};
  size_t startMarkerIndex_{0};
  std::vector<uint8_t> messageBuf_;
  uint8_t messageType_;
  uint8_t messageLen_;
  std::vector<uint8_t> crcBytes_;
  std::shared_ptr<std::thread> serialThread_;
};
}  // namespace arduino_imu
#endif  // ARDUINO_IMU__ARDUINO_IMU_HPP_
