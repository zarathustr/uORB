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


#include <uORB/topics/ekf2_replay.h>

static constexpr char orb_ekf2_replay_fields[] =
  "uint64_t timestamp;uint64_t magnetometer_timestamp;uint64_t baro_timestamp;uint64_t rng_timestamp;uint64_t flow_timestamp;uint64_t asp_timestamp;uint64_t ev_timestamp;uint64_t time_usec;float gyro_integral_dt;float accelerometer_integral_dt;float[3] gyro_rad;float[3] accelerometer_m_s2;float[3] magnetometer_ga;float baro_alt_meter;int32_t lat;int32_t lon;int32_t alt;float eph;float epv;float sacc;float vel_m_s;float vel_n_m_s;float vel_e_m_s;float vel_d_m_s;float range_to_ground;float[2] flow_pixel_integral;float[2] flow_gyro_integral;uint32_t flow_time_integral;float indicated_airspeed_m_s;float true_airspeed_m_s;float[3] pos_ev;float[4] quat_ev;float pos_err;float ang_err;uint8_t fix_type;uint8_t nsats;bool vel_ned_valid;uint8_t flow_quality;";

ORB_DEFINE(ekf2_replay, struct ekf2_replay_s, 224, orb_ekf2_replay_fields, 1);

