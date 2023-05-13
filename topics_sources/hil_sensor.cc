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


#include <uORB/topics/hil_sensor.h>

static constexpr char orb_hil_sensor_fields[] =
  "uint64_t timestamp;uint64_t accelerometer_timestamp;uint64_t magnetometer_timestamp;uint64_t gyro1_timestamp;uint64_t accelerometer1_timestamp;uint64_t magnetometer1_timestamp;uint64_t gyro2_timestamp;uint64_t accelerometer2_timestamp;uint64_t magnetometer2_timestamp;uint64_t baro_timestamp;uint64_t baro1_timestamp;uint64_t differential_pressure_timestamp;uint64_t differential_pressure1_timestamp;float[3] gyro_rad_s;uint32_t gyro_errcount;float gyro_temp;float[3] accelerometer_m_s2;float accelerometer_range_m_s2;uint32_t accelerometer_errcount;float accelerometer_temp;float[3] magnetometer_ga;float magnetometer_range_ga;float magnetometer_cuttoff_freq_hz;uint32_t magnetometer_errcount;float magnetometer_temp;float[3] gyro1_rad_s;uint32_t gyro1_errcount;float gyro1_temp;float[3] accelerometer1_m_s2;uint32_t accelerometer1_errcount;float accelerometer1_temp;float[3] magnetometer1_ga;uint32_t magnetometer1_errcount;float magnetometer1_temp;float[3] gyro2_rad_s;uint32_t gyro2_errcount;float gyro2_temp;float[3] accelerometer2_m_s2;uint32_t accelerometer2_errcount;float accelerometer2_temp;float[3] magnetometer2_ga;uint32_t magnetometer2_errcount;float magnetometer2_temp;float baro_pres_mbar;float baro_alt_meter;float baro_temp_celcius;float baro1_pres_mbar;float baro1_alt_meter;float baro1_temp_celcius;float[10] adc_voltage_v;float mcu_temp_celcius;float differential_pressure_pa;float differential_pressure_filtered_pa;float differential_pressure1_pa;float differential_pressure1_filtered_pa;int16_t[3] gyro_raw;int16_t[3] accelerometer_raw;int16_t accelerometer_mode;int16_t[3] magnetometer_raw;int16_t magnetometer_mode;int16_t[3] gyro1_raw;int16_t[3] accelerometer1_raw;int16_t[3] magnetometer1_raw;int16_t[3] gyro2_raw;int16_t[3] accelerometer2_raw;int16_t[3] magnetometer2_raw;uint16_t[10] adc_mapping;uint8_t[6] _padding0;";

ORB_DEFINE(hil_sensor, struct hil_sensor_s, 458, orb_hil_sensor_fields, 1);

