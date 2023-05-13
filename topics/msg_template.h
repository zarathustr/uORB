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
#define MSG_TEMPLATE_u32_constant 10

#endif


#ifdef __cplusplus
struct __EXPORT msg_template_s {
#else
struct msg_template_s {
#endif
  uint64_t timestamp;
  int64_t s64_value;
  uint64_t u64_value;
  double f64_value;
  int32_t s32_value;
  uint32_t u32_value;
  float f32_value;
  uint32_t uint32_array[4];
  int16_t s16_value;
  uint16_t u16_value;
  char char_value;
  bool bool_value;
  int8_t s8_value;
  uint8_t u8_value;
  int8_t str[5];
  uint8_t _padding0[7];  // required for logger


#ifdef __cplusplus
  static constexpr uint32_t u32_constant = 10;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(msg_template, msg_template_s);

