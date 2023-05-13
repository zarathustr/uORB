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
#define VTOL_VEHICLE_STATUS_VEHICLE_VTOL_STATE_UNDEFINED 0
#define VTOL_VEHICLE_STATUS_VEHICLE_VTOL_STATE_TRANSITION_TO_FW 1
#define VTOL_VEHICLE_STATUS_VEHICLE_VTOL_STATE_TRANSITION_TO_MC 2
#define VTOL_VEHICLE_STATUS_VEHICLE_VTOL_STATE_MC 3
#define VTOL_VEHICLE_STATUS_VEHICLE_VTOL_STATE_FW 4
#define VTOL_VEHICLE_STATUS_VEHICLE_VTOL_STATE_EXTERNAL 5

#endif


#ifdef __cplusplus
struct __EXPORT vtol_vehicle_status_s {
#else
struct vtol_vehicle_status_s {
#endif
  uint64_t timestamp;
  float airspeed_tot;
  bool vtol_in_rw_mode;
  bool vtol_in_trans_mode;
  bool in_transition_to_fw;
  bool vtol_transition_failsafe;
  bool fw_permanent_stab;
  uint8_t _padding0[7];  // required for logger


#ifdef __cplusplus
  static constexpr uint8_t VEHICLE_VTOL_STATE_UNDEFINED = 0;
  static constexpr uint8_t VEHICLE_VTOL_STATE_TRANSITION_TO_FW = 1;
  static constexpr uint8_t VEHICLE_VTOL_STATE_TRANSITION_TO_MC = 2;
  static constexpr uint8_t VEHICLE_VTOL_STATE_MC = 3;
  static constexpr uint8_t VEHICLE_VTOL_STATE_FW = 4;
  static constexpr uint8_t VEHICLE_VTOL_STATE_EXTERNAL = 5;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(vtol_vehicle_status, vtol_vehicle_status_s);

