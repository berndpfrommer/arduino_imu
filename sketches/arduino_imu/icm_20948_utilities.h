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

#ifndef ARDUINO_IMU__ICM_20948_UTILITIES_H_
#define ARDUINO_IMU__ICM_20948_UTILITIES_H_

#include <ICM_20948.h>  // SparkFun ICM 20948 library
#include <Wire.h>

namespace icm_20948_utilities
{
void flash_error(int pin, int period)
{
  const int half_period = period / 2;
  while (true) {
    digitalWrite(pin, !digitalRead(pin));
    delay(half_period);
  }
}

void setup_icm(ICM_20948_I2C & icm, int ad0_val)
{
  Wire.begin();
  Wire.setClock(400000);
  for (icm.begin(Wire, ad0_val); icm.status != ICM_20948_Stat_Ok;) {
    delay(500);
    icm.begin(Wire, ad0_val);
  }
}

void setup_icm_dmp(ICM_20948_I2C & icm, int led_pin)
{
  bool success = false;
  while (!success) {
    success = true;
    ICM_20948_Status_e rc = icm.initializeDMP();
    success &= (icm.initializeDMP() == ICM_20948_Stat_Ok);
    success &=
      (icm.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) ==
       ICM_20948_Stat_Ok);
    success &=
      (icm.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) ==
       ICM_20948_Stat_Ok);
    success &=
      (icm.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) ==
       ICM_20948_Stat_Ok);
    success &=
      (icm.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) ==
       ICM_20948_Stat_Ok);
    success &=
      (icm.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) ==
       ICM_20948_Stat_Ok);  // full speed
    success &= (icm.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok);
    success &= (icm.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok);
    success &= (icm.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (icm.enableDMP() == ICM_20948_Stat_Ok);
    success &= (icm.resetDMP() == ICM_20948_Stat_Ok);
    success &= (icm.resetFIFO() == ICM_20948_Stat_Ok);
    if (!success) {
      flash_error(led_pin, 1000);
    }
  }
}

float to_ms2(int16_t raw)
{
  // raw value ranges from -2^15 ... 2^15
  // this value represents the range of -4g .. 4g (sensor configurable!)
  constexpr float fac = 9.80655 / (1 << 13);
  return ((float)(raw)*fac);
}

float to_radps(int16_t raw)
{
  // raw value ranges from -2^15 ... 2^15
  // this value represents -2000 deg/s -> 2000 deg/s
  constexpr float fac = 3.14159265 / 180.0 / (1 << 15);
  return ((float)(raw)*fac);
}

}  // namespace icm_20948_utilities

#endif  // ARDUINO_IMU__ICM_20948_UTILITIES_H_
