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
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_GYRO 1
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_ACC 2
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_MAG 4
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_ABSPRESSURE 8
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_DIFFPRESSURE 16
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_GPS 32
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_OPTICALFLOW 64
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_CVPOSITION 128
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_LASERPOSITION 256
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_EXTERNALGROUNDTRUTH 512
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_ANGULARRATECONTROL 1024
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_ATTITUDESTABILIZATION 2048
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_YAWPOSITION 4096
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_ALTITUDECONTROL 16384
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_POSITIONCONTROL 32768
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_MOTORCONTROL 65536
#define SUBSYSTEM_INFO_SUBSYSTEM_TYPE_RANGEFINDER 131072

#endif


#ifdef __cplusplus
struct __EXPORT subsystem_info_s {
#else
struct subsystem_info_s {
#endif
  uint64_t timestamp;
  uint64_t subsystem_type;
  bool present;
  bool enabled;
  bool ok;
  uint8_t _padding0[5];  // required for logger


#ifdef __cplusplus
  static constexpr uint64_t SUBSYSTEM_TYPE_GYRO = 1;
  static constexpr uint64_t SUBSYSTEM_TYPE_ACC = 2;
  static constexpr uint64_t SUBSYSTEM_TYPE_MAG = 4;
  static constexpr uint64_t SUBSYSTEM_TYPE_ABSPRESSURE = 8;
  static constexpr uint64_t SUBSYSTEM_TYPE_DIFFPRESSURE = 16;
  static constexpr uint64_t SUBSYSTEM_TYPE_GPS = 32;
  static constexpr uint64_t SUBSYSTEM_TYPE_OPTICALFLOW = 64;
  static constexpr uint64_t SUBSYSTEM_TYPE_CVPOSITION = 128;
  static constexpr uint64_t SUBSYSTEM_TYPE_LASERPOSITION = 256;
  static constexpr uint64_t SUBSYSTEM_TYPE_EXTERNALGROUNDTRUTH = 512;
  static constexpr uint64_t SUBSYSTEM_TYPE_ANGULARRATECONTROL = 1024;
  static constexpr uint64_t SUBSYSTEM_TYPE_ATTITUDESTABILIZATION = 2048;
  static constexpr uint64_t SUBSYSTEM_TYPE_YAWPOSITION = 4096;
  static constexpr uint64_t SUBSYSTEM_TYPE_ALTITUDECONTROL = 16384;
  static constexpr uint64_t SUBSYSTEM_TYPE_POSITIONCONTROL = 32768;
  static constexpr uint64_t SUBSYSTEM_TYPE_MOTORCONTROL = 65536;
  static constexpr uint64_t SUBSYSTEM_TYPE_RANGEFINDER = 131072;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(subsystem_info, subsystem_info_s);

