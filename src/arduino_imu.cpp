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

#include "arduino_imu/arduino_imu.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <cstdlib>
#include <functional>
#include <iostream>
#include <rclcpp_components/register_node_macro.hpp>

#include "arduino_imu/arduino_imu.hpp"

// #define SIMULATE_TRANSMISSION_ERRORS

namespace arduino_imu
{
static constexpr int IMU_DATA_SIZE = 60;
static constexpr int IMU_OMEGA_OFFSET = 8;
static constexpr int IMU_ACC_OFFSET = 20;
static constexpr int IMU_MAG_OFFSET = 32;
static constexpr int IMU_Q_OFFSET = 44;

static constexpr const char * START_MARKER = "\1AIMU";
static constexpr const size_t START_MARKER_SIZE = strlen(START_MARKER);

static const std::map<uint8_t, uint8_t> MESSAGE_TYPE_TO_LENGTH = {
  {1, 4}, {2, IMU_DATA_SIZE}};

ArduinoImu::ArduinoImu(const rclcpp::NodeOptions & options)
: Node(
    "arduino_imu", rclcpp::NodeOptions(options)
                     .automatically_declare_parameters_from_overrides(true))
{
  this->get_parameter_or(
    "frame_id", imuMsg_.header.frame_id, std::string("imu"));
  magMsg_.header.frame_id = imuMsg_.header.frame_id;
  this->get_parameter_or("device", device_, std::string("/dev/ttyACM0"));
  this->get_parameter_or("baud", baud_, 500000);
  RCLCPP_INFO_STREAM(
    get_logger(), "opening port " << device_ << " at speed " << baud_);
  int queueSize(1);
  this->get_parameter_or("queue_size", queueSize, 10);

  for (int i = 0; i < 9; i++) {
    imuMsg_.orientation_covariance[i] = (i == 0) ? -1 : 0;
    imuMsg_.angular_velocity_covariance[i] = (i == 0) ? -1 : 0;
    imuMsg_.linear_acceleration_covariance[i] = (i == 0) ? -1 : 0;
    magMsg_.magnetic_field_covariance[i] = (i == 0) ? -1 : 0;
  }
  lastStatTime_ = this->get_clock()->now();
  imuPub_ = this->create_publisher<Imu>("~/imu", queueSize);
  magPub_ =
    this->create_publisher<MagneticField>("~/magnetic_field", queueSize);
  if (!openSerialPort()) {
    throw std::runtime_error("serial port initialization failed!");
  }
  serialThread_ =
    std::make_shared<std::thread>(&ArduinoImu::serialThread, this);
}

ArduinoImu::~ArduinoImu()
{
  if (serialThread_) {
    keepRunning_ = false;
    serialThread_->join();
    serialThread_.reset();
  }
  if (serialFD_ > 0) {
    close(serialFD_);
  }
}

void ArduinoImu::serialThread()
{
  while (rclcpp::ok() && keepRunning_) {
    uint8_t buf[256];
    int bytesRead =
      read(serialFD_, reinterpret_cast<char *>(buf), sizeof(buf) - 1);
    if (bytesRead > 0) {
#ifdef SIMULATE_TRANSMISSION_ERRORS
      unsigned int seed;
      uint8_t buf2[256];
      memcpy(buf2, buf, bytesRead);
      for (int i = 0; i < 5; i++) {
        if (rand_r(&seed) % 10 == 0) {
          buf2[rand_r(&seed) % bytesRead] = rand_r(&seed) % 256;
        }
      }
      processSerialBytes(buf2, bytesRead);
#else
      processSerialBytes(buf, bytesRead);
#endif
    }
  }
  RCLCPP_INFO(get_logger(), "serial thread exited!");
}

// from stack exchange:
//  https://stackoverflow.com/questions/10564491/function-to-calculate-a-crc16-checksum
static uint16_t crc16(uint8_t const * data, size_t size)
{
  uint16_t crc = 0;
  while (size--) {
    crc ^= *data++;
    for (unsigned k = 0; k < 8; k++)
      crc = crc & 1 ? (crc >> 1) ^ 0xa001 : crc >> 1;
  }
  return crc;
}

void ArduinoImu::processSerialBytes(const uint8_t * buf, size_t len)
{
#ifdef DEBUG_DATA
  std::cout << "line:";
  for (size_t k = 0; k < len; k++) {
    std::cout << " [" << k << "]=" << static_cast<int>(buf[k]);
  }
  std::cout << std::endl;
#endif

  size_t i = 0;
  while (i < len) {
    if (serialState_ == INITIALIZING) {
      for (; i < len && startMarkerIndex_ < START_MARKER_SIZE; i++) {
        if (buf[i] == START_MARKER[startMarkerIndex_]) {
          startMarkerIndex_++;
        } else {
          startMarkerIndex_ = 0;
        }
      }
      if (startMarkerIndex_ == START_MARKER_SIZE) {
        startMarkerIndex_ = 0;
        serialState_ = GET_MESSAGE_TYPE;
        RCLCPP_INFO(get_logger(), "found start marker in serial stream!");
      }
    }
    if (serialState_ == GET_MESSAGE_TYPE) {
      if (i < len) {
        messageType_ = buf[i];
        auto it = MESSAGE_TYPE_TO_LENGTH.find(messageType_);
        if (it == MESSAGE_TYPE_TO_LENGTH.end()) {
          RCLCPP_WARN_STREAM(
            get_logger(), "got bad msg type: " << (int)messageType_
                                               << ", starting marker search!");
          serialState_ = INITIALIZING;  // throw away rest of buffer
        } else {
          messageLen_ = it->second;
          serialState_ = GET_PAYLOAD;
        }
        i++;  // to mark the byte as consumed
      }
    }
    if (serialState_ == GET_PAYLOAD) {
      for (; i < len && messageBuf_.size() < messageLen_; i++) {
        messageBuf_.push_back(buf[i]);
      }
      if (messageBuf_.size() == messageLen_) {
        if (messageType_ == 1) {
          // we got a start-marker message which has no CRC
          // and shall be skipped.
          messageBuf_.clear();
          serialState_ = GET_MESSAGE_TYPE;  // skip delivery
        } else {
          serialState_ = GET_CRC;
        }
      }
    }
    if (serialState_ == GET_CRC) {
      for (; i < len && crcBytes_.size() < 2; i++) {
        crcBytes_.push_back(buf[i]);
      }
      if (crcBytes_.size() == 2) {
        const uint16_t crcSend = crcBytes_[1] << 8 | crcBytes_[0];
        const uint16_t crcRecv = crc16(messageBuf_.data(), messageBuf_.size());
        if (crcSend == crcRecv) {
          receivedCallback(messageType_, messageBuf_);
        } else {
          RCLCPP_WARN(get_logger(), "crc checksum error!");
        }
        messageBuf_.clear();
        crcBytes_.clear();
        serialState_ = GET_MESSAGE_TYPE;
      }
    }
  }
}

void ArduinoImu::parseImuMessage(const std::vector<uint8_t> & data)
{
  const rclcpp::Time t = this->get_clock()->now();
  if (static_cast<int>(data.size()) != IMU_DATA_SIZE) {
    RCLCPP_ERROR_STREAM(get_logger(), "bad data length: " << data.size());
    return;
  }
  // assume that endianess is the same as on arduino (little endian)
  //  int64_t time = *reinterpret_cast<const uint64_t *>(data.data());
  if (imuPub_->get_subscription_count() > 0) {
    const float * omega =
      reinterpret_cast<const float *>(data.data() + IMU_OMEGA_OFFSET);
    const float * acc =
      reinterpret_cast<const float *>(data.data() + IMU_ACC_OFFSET);
    const float * q =
      reinterpret_cast<const float *>(data.data() + IMU_Q_OFFSET);
    imuMsg_.header.stamp = t;
    imuMsg_.angular_velocity.x = omega[0];
    imuMsg_.angular_velocity.y = omega[1];
    imuMsg_.angular_velocity.z = omega[2];
    imuMsg_.linear_acceleration.x = acc[0];
    imuMsg_.linear_acceleration.y = acc[1];
    imuMsg_.linear_acceleration.z = acc[2];
    imuMsg_.orientation.w = q[0];
    imuMsg_.orientation.x = q[1];
    imuMsg_.orientation.y = q[2];
    imuMsg_.orientation.z = q[3];
    imuPub_->publish(imuMsg_);
  }
  if (magPub_->get_subscription_count() > 0) {
    const float * mag =
      reinterpret_cast<const float *>(data.data() + IMU_MAG_OFFSET);
    magMsg_.magnetic_field.x = mag[0];
    magMsg_.magnetic_field.y = mag[1];
    magMsg_.magnetic_field.z = mag[2];
    magMsg_.header.stamp = t;
    magPub_->publish(magMsg_);
  }
  inCount_++;
  const auto dt = t - lastStatTime_;
  if (dt.seconds() >= 1.0) {
    const float rate = inCount_ / dt.seconds();
    RCLCPP_INFO(get_logger(), "in msg rate: %6.2f hz", rate);
    lastStatTime_ = t;
    inCount_ = 0;
  }
}

void ArduinoImu::receivedCallback(
  uint8_t msgType, const std::vector<uint8_t> & data)
{
  if (msgType == 2) {
    parseImuMessage(data);
  }
}

const std::map<int, speed_t> speedMap = {
  {1200, B1200},     {2400, B2400},     {4800, B4800},     {9600, B9600},
  {19200, B19200},   {38400, B38400},   {57600, B57600},   {115200, B115200},
  {230400, B230400}, {460800, B460800}, {500000, B500000}, {576000, B576000},
  {921600, B921600}};

bool ArduinoImu::openSerialPort()
{
  auto speed = speedMap.find(baud_);
  if (speed == speedMap.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "invalid port speed: " << baud_);
    return (false);
  }
  serialFD_ = open(device_.c_str(), O_RDWR);
  if (serialFD_ < 0) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "cannot open serial port " << device_ << ": " << strerror(errno));
    return (false);
  }
  struct termios tty;

  // Read in existing settings, and handle any error
  if (tcgetattr(serialFD_, &tty) != 0) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "cannot get serial port attributes: " << errno << " " << strerror(errno));
    close(serialFD_);
    serialFD_ = -1;
    return (false);
  }

  const bool useHardwareFlowControl{false};
  tty.c_cflag &=
    ~(PARENB | CSTOPB | CSIZE);  // no party, 1 stop bit, no data size
  tty.c_cflag |= CS8;            // 8 bits per byte (most common)
  if (useHardwareFlowControl) {
    tty.c_cflag |= CRTSCTS;
  } else {
    tty.c_cflag &= ~CRTSCTS;
  }
  tty.c_cflag |= CREAD | CLOCAL;  // enable read & ignore ctrl lines

  // disable canonical, echoing, and signal interpret
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
  // disable software flow control and special handling of bytes
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~(OPOST | ONLCR);
  tty.c_cc[VTIME] = 10;  // wait max of 10 deciseconds
  tty.c_cc[VMIN] = 0;    // but return immediately if data received

  cfsetispeed(&tty, speed->second);
  cfsetospeed(&tty, speed->second);

  if (tcsetattr(serialFD_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "cannot set serial port attributes: " << errno << " " << strerror(errno));
    close(serialFD_);
    serialFD_ = -1;
    return (false);
  }
  return (true);
}

}  // namespace arduino_imu

RCLCPP_COMPONENTS_REGISTER_NODE(arduino_imu::ArduinoImu)
