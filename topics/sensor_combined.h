/****************************************************************************
 *
 *   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#pragma once


#include <uORB/uorbNew/uorb.h>
#include <inttypes.h>


#ifndef __cplusplus
#define SENSOR_COMBINED_RELATIVE_TIMESTAMP_INVALID 2147483647

#endif


#ifdef __cplusplus
struct __EXPORT sensor_combined_s {
#else
struct sensor_combined_s {
#endif
  uint64_t timestamp;
  float gyro_rad[3];
  float gyro_integral_dt;
  int32_t accelerometer_timestamp_relative;
  float accelerometer_m_s2[3];
  float accelerometer_integral_dt;
  int32_t magnetometer_timestamp_relative;
  float magnetometer_ga[3];
  int32_t baro_timestamp_relative;
  float baro_alt_meter;
  float baro_temp_celcius;
  bool reinit;
  uint64_t sample_counter;
  uint8_t _padding0[7];  // required for logger


#ifdef __cplusplus
  static constexpr int32_t RELATIVE_TIMESTAMP_INVALID = 2147483647;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(sensor_combined, sensor_combined_s);

