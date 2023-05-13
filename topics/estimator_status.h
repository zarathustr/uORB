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
#define ESTIMATOR_STATUS_GPS_CHECK_FAIL_MIN_SAT_COUNT 0
#define ESTIMATOR_STATUS_GPS_CHECK_FAIL_MIN_GDOP 1
#define ESTIMATOR_STATUS_GPS_CHECK_FAIL_MAX_HORZ_ERR 2
#define ESTIMATOR_STATUS_GPS_CHECK_FAIL_MAX_VERT_ERR 3
#define ESTIMATOR_STATUS_GPS_CHECK_FAIL_MAX_SPD_ERR 4
#define ESTIMATOR_STATUS_GPS_CHECK_FAIL_MAX_HORZ_DRIFT 5
#define ESTIMATOR_STATUS_GPS_CHECK_FAIL_MAX_VERT_DRIFT 6
#define ESTIMATOR_STATUS_GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR 7
#define ESTIMATOR_STATUS_GPS_CHECK_FAIL_MAX_VERT_SPD_ERR 8

#endif


#ifdef __cplusplus
struct __EXPORT estimator_status_s {
#else
struct estimator_status_s {
#endif
  uint64_t timestamp;
  float states[24];
  float n_states;
  float vibe[3];
  float covariances[24];
  float pos_horiz_accuracy;
  float pos_vert_accuracy;
  float mag_test_ratio;
  float vel_test_ratio;
  float pos_test_ratio;
  float hgt_test_ratio;
  float tas_test_ratio;
  float hagl_test_ratio;
  float time_slip;
  uint16_t gps_check_fail_flags;
  uint16_t control_mode_flags;
  uint16_t filter_fault_flags;
  uint16_t innovation_check_flags;
  uint16_t solution_status_flags;
  uint8_t nan_flags;
  uint8_t health_flags;
  uint8_t timeout_flags;
  uint8_t _padding0[7];  // required for logger


#ifdef __cplusplus
  static constexpr uint16_t GPS_CHECK_FAIL_MIN_SAT_COUNT = 0;
  static constexpr uint16_t GPS_CHECK_FAIL_MIN_GDOP = 1;
  static constexpr uint16_t GPS_CHECK_FAIL_MAX_HORZ_ERR = 2;
  static constexpr uint16_t GPS_CHECK_FAIL_MAX_VERT_ERR = 3;
  static constexpr uint16_t GPS_CHECK_FAIL_MAX_SPD_ERR = 4;
  static constexpr uint16_t GPS_CHECK_FAIL_MAX_HORZ_DRIFT = 5;
  static constexpr uint16_t GPS_CHECK_FAIL_MAX_VERT_DRIFT = 6;
  static constexpr uint16_t GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR = 7;
  static constexpr uint16_t GPS_CHECK_FAIL_MAX_VERT_SPD_ERR = 8;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(estimator_status, estimator_status_s);

