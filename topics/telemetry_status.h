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
#define TELEMETRY_STATUS_TELEMETRY_STATUS_RADIO_TYPE_GENERIC 0
#define TELEMETRY_STATUS_TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO 1
#define TELEMETRY_STATUS_TELEMETRY_STATUS_RADIO_TYPE_UBIQUITY_BULLET 2
#define TELEMETRY_STATUS_TELEMETRY_STATUS_RADIO_TYPE_WIRE 3
#define TELEMETRY_STATUS_TELEMETRY_STATUS_RADIO_TYPE_USB 4
#define TELEMETRY_STATUS_TELEMETRY_STATUS_RADIO_TYPE_IRIDIUM 5

#endif


#ifdef __cplusplus
struct __EXPORT telemetry_status_s {
#else
struct telemetry_status_s {
#endif
  uint64_t timestamp;
  uint64_t heartbeat_time;
  uint64_t telem_time;
  uint16_t rxerrors;
  uint16_t fixed;
  uint8_t type;
  uint8_t rssi;
  uint8_t remote_rssi;
  uint8_t noise;
  uint8_t remote_noise;
  uint8_t txbuf;
  uint8_t system_id;
  uint8_t component_id;
  uint8_t _padding0[4];  // required for logger


#ifdef __cplusplus
  static constexpr uint8_t TELEMETRY_STATUS_RADIO_TYPE_GENERIC = 0;
  static constexpr uint8_t TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO = 1;
  static constexpr uint8_t TELEMETRY_STATUS_RADIO_TYPE_UBIQUITY_BULLET = 2;
  static constexpr uint8_t TELEMETRY_STATUS_RADIO_TYPE_WIRE = 3;
  static constexpr uint8_t TELEMETRY_STATUS_RADIO_TYPE_USB = 4;
  static constexpr uint8_t TELEMETRY_STATUS_RADIO_TYPE_IRIDIUM = 5;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(telemetry_status, telemetry_status_s);

