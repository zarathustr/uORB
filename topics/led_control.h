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
#define LED_CONTROL_COLOR_OFF 0
#define LED_CONTROL_COLOR_RED 1
#define LED_CONTROL_COLOR_GREEN 2
#define LED_CONTROL_COLOR_BLUE 3
#define LED_CONTROL_COLOR_YELLOW 4
#define LED_CONTROL_COLOR_PURPLE 5
#define LED_CONTROL_COLOR_AMBER 6
#define LED_CONTROL_COLOR_CYAN 7
#define LED_CONTROL_COLOR_WHITE 8
#define LED_CONTROL_MODE_OFF 0
#define LED_CONTROL_MODE_ON 1
#define LED_CONTROL_MODE_DISABLED 2
#define LED_CONTROL_MODE_BLINK_SLOW 3
#define LED_CONTROL_MODE_BLINK_NORMAL 4
#define LED_CONTROL_MODE_BLINK_FAST 5
#define LED_CONTROL_MODE_BREATHE 6
#define LED_CONTROL_MAX_PRIORITY 2

#endif


#ifdef __cplusplus
struct __EXPORT led_control_s {
#else
struct led_control_s {
#endif
  uint64_t timestamp;
  uint8_t led_mask;
  uint8_t color;
  uint8_t mode;
  uint8_t num_blinks;
  uint8_t priority;
  uint8_t _padding0[3];  // required for logger


#ifdef __cplusplus
  static constexpr uint8_t COLOR_OFF = 0;
  static constexpr uint8_t COLOR_RED = 1;
  static constexpr uint8_t COLOR_GREEN = 2;
  static constexpr uint8_t COLOR_BLUE = 3;
  static constexpr uint8_t COLOR_YELLOW = 4;
  static constexpr uint8_t COLOR_PURPLE = 5;
  static constexpr uint8_t COLOR_AMBER = 6;
  static constexpr uint8_t COLOR_CYAN = 7;
  static constexpr uint8_t COLOR_WHITE = 8;
  static constexpr uint8_t MODE_OFF = 0;
  static constexpr uint8_t MODE_ON = 1;
  static constexpr uint8_t MODE_DISABLED = 2;
  static constexpr uint8_t MODE_BLINK_SLOW = 3;
  static constexpr uint8_t MODE_BLINK_NORMAL = 4;
  static constexpr uint8_t MODE_BLINK_FAST = 5;
  static constexpr uint8_t MODE_BREATHE = 6;
  static constexpr uint8_t MAX_PRIORITY = 2;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(led_control, led_control_s);

