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


#include <uORB/topics/sensor_correction.h>

static constexpr char orb_sensor_correction_fields[] =
  "uint64_t timestamp;float[3] gyro_offset_0;float[3] gyro_scale_0;float[3] gyro_offset_1;float[3] gyro_scale_1;float[3] gyro_offset_2;float[3] gyro_scale_2;float[3] accel_offset_0;float[3] accel_scale_0;float[3] accel_offset_1;float[3] accel_scale_1;float[3] accel_offset_2;float[3] accel_scale_2;float baro_offset_0;float baro_scale_0;float baro_offset_1;float baro_scale_1;float baro_offset_2;float baro_scale_2;uint8_t selected_gyro_instance;uint8_t selected_accel_instance;uint8_t selected_baro_instance;uint8_t[3] gyro_mapping;uint8_t[3] accel_mapping;uint8_t[3] baro_mapping;uint8_t[4] _padding0;";

ORB_DEFINE(sensor_correction, struct sensor_correction_s, 188, orb_sensor_correction_fields, 1);

