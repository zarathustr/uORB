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
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_THROTTLE 0
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_ROLL 1
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_PITCH 2
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_YAW 3
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_MODE 4
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_RETURN 5
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_POSCTL 6
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_LOITER 7
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_OFFBOARD 8
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_ACRO 9
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_FLAPS 10
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_AUX_1 11
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_AUX_2 12
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_AUX_3 13
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_AUX_4 14
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_AUX_5 15
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_PARAM_1 16
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_PARAM_2 17
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_PARAM_3_5 18
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_RATTITUDE 19
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_KILLSWITCH 20
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_TRANSITION 21
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_GEAR 22
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_ARMSWITCH 23
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_STAB 24
#define RC_CHANNELS_RC_CHANNELS_FUNCTION_MAN 25

#endif


#ifdef __cplusplus
struct __EXPORT rc_channels_s {
#else
struct rc_channels_s {
#endif
  uint64_t timestamp;
  uint64_t timestamp_last_valid;
  float channels[18];
  uint32_t frame_drop_count;
  uint8_t channel_count;
  int8_t function[26];
  uint8_t rssi;
  bool signal_lost;
  uint8_t _padding0[7];  // required for logger


#ifdef __cplusplus
  static constexpr uint8_t RC_CHANNELS_FUNCTION_THROTTLE = 0;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_ROLL = 1;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_PITCH = 2;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_YAW = 3;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_MODE = 4;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_RETURN = 5;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_POSCTL = 6;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_LOITER = 7;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_OFFBOARD = 8;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_ACRO = 9;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_FLAPS = 10;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_AUX_1 = 11;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_AUX_2 = 12;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_AUX_3 = 13;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_AUX_4 = 14;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_AUX_5 = 15;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_PARAM_1 = 16;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_PARAM_2 = 17;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_PARAM_3_5 = 18;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_RATTITUDE = 19;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_KILLSWITCH = 20;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_TRANSITION = 21;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_GEAR = 22;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_ARMSWITCH = 23;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_STAB = 24;
  static constexpr uint8_t RC_CHANNELS_FUNCTION_MAN = 25;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(rc_channels, rc_channels_s);

