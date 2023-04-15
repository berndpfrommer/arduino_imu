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

#include "icm_20948_utilities.h"  // data conversion utilities etc
#include <ICM_20948.h>            // SparkFun ICM 20948 library
#include "icm_20948_overrides.h"  // override init for higher sample rate
#include <stdio.h>
#include <Wire.h>


// --------------- various constants ---------------

// the serial port speed between arduino and host computer
const int BAUD_RATE = 500000;

// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1
// and when the ADR jumper is closed the value becomes 0
const int AD0_VAL = 1;

// Which pin the warning LED is attached to.
const int LED_PIN = 13;

// How often to send out a marker message
const int MARKER_MESSAGE_INTERVAL = 20;
// -------------------------------------

// The IMU data structure published to the serial port
struct imu_data_t {
  // ----- time stamp
  uint64_t usec{0};
  // ---------- angular velocity
  float omega[3] = {0, 0, 0};
  // ---------- linear acceleration
  float acc[3] = {0, 0, 0};
  // ---------- magnetic field
  float mag[3] = {0, 0, 0};
  // ---------- quaternion orientation
  float q[4] = { 1, 0, 0, 0}; // q[0] = w, q[1] = x ....
} __attribute__((packed));


ICM_20948_I2C icm; // the handle to the ICM 20948

void setup(void) {
  Serial.begin(BAUD_RATE);
  icm_20948_utilities::setup_icm(icm, AD0_VAL);
  // the digital motion processor is needed to get
  // the orientation quaternion
  icm_20948_utilities::setup_icm_dmp(icm, LED_PIN);
}

int cnt = 0;  // for start marker message

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

imu_data_t imu_data; // will be populated in loop()

void loop() {
  bool got_fresh_data = false;
  
  // wait until fresh data has arrived.
  icm_20948_DMP_data_t data;
  for (icm.readDMPdataFromFIFO(&data); icm.status != ICM_20948_Stat_Ok; ) {
    icm.readDMPdataFromFIFO(&data);
  }
  
  // ------ parse acceleration data
  if ((data.header & DMP_header_bitmap_Accel) > 0) {
    imu_data.acc[0] = icm_20948_utilities::to_ms2(data.Raw_Accel.Data.X);
    imu_data.acc[1] = icm_20948_utilities::to_ms2(data.Raw_Accel.Data.Y);
    imu_data.acc[2] = icm_20948_utilities::to_ms2(data.Raw_Accel.Data.Z);
    got_fresh_data = true;
  }

  // ------- parse gyro data
  if ((data.header & DMP_header_bitmap_Gyro) > 0) {
    imu_data.omega[0] = icm_20948_utilities::to_radps(data.Raw_Gyro.Data.X);
    imu_data.omega[1] = icm_20948_utilities::to_radps(data.Raw_Gyro.Data.Y);
    imu_data.omega[2] = icm_20948_utilities::to_radps(data.Raw_Gyro.Data.Z);
    got_fresh_data = true;
  }
  
  // ------- parse orientation
  if ((data.header & DMP_header_bitmap_Quat9) > 0) {
     // convert to double and divide by 2^30
    imu_data.q[1] = ((double)data.Quat9.Data.Q1) / 1073741824.0;
    imu_data.q[2] = ((double)data.Quat9.Data.Q2) / 1073741824.0;
    imu_data.q[3] = ((double)data.Quat9.Data.Q3) / 1073741824.0;

    imu_data.q[0] = sqrt(1.0 - ((imu_data.q[1] * imu_data.q[1]) +
                    (imu_data.q[2] * imu_data.q[2]) +
                    (imu_data.q[3] * imu_data.q[3])));
    got_fresh_data = true;
  }

  // -------- parse magnetic field
  if ((data.header & DMP_header_bitmap_Compass) > 0) {
    // according to the sparkfun library DMP initialization
    // code, dividing by 4912 gives the field in microtesla
    // WARNING: no idea if this is correct
    const float fac = 1e-6 / 4912;
    imu_data.mag[0] = (float)data.Compass.Data.X * fac;
    imu_data.mag[1] = (float)data.Compass.Data.Y * fac;
    imu_data.mag[2] = (float)data.Compass.Data.Z * fac;
    got_fresh_data = true;
  }

  if (got_fresh_data) {
    // occasionally write special marker message such that
    // the receiving host can find the beginning of the message
    // boundaries more easily
    if (cnt++ >= MARKER_MESSAGE_INTERVAL) {
      cnt = 0;
      const uint8_t msg_type = 1;
      Serial.write(msg_type); // start marker message type
      const char *buf = "AIMU";
      Serial.write(buf); // no CRC for this message
    }
    // write regular message
    imu_data.usec = micros();
    const uint8_t msg_type = 2; // regular message
    const uint16_t u16crc = crc16(reinterpret_cast<const uint8_t*>(&imu_data), sizeof(imu_data));
    // write message type, payload, and checksum
    Serial.write(msg_type);
    Serial.write(reinterpret_cast<const char*>(&imu_data), sizeof(imu_data));
    Serial.write(reinterpret_cast<const char *>(&u16crc), sizeof(u16crc));
  }
                                 
}
