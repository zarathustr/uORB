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


#include <uORB/topics/vehicle_control_mode.h>

static constexpr char orb_vehicle_control_mode_fields[] =
  "uint64_t timestamp;bool flag_armed;bool flag_external_manual_override_ok;bool flag_system_hil_enabled;bool flag_control_manual_enabled;bool flag_control_auto_enabled;bool flag_control_offboard_enabled;bool flag_control_rates_enabled;bool flag_control_attitude_enabled;bool flag_control_rattitude_enabled;bool flag_control_force_enabled;bool flag_control_acceleration_enabled;bool flag_control_velocity_enabled;bool flag_control_position_enabled;bool flag_control_altitude_enabled;bool flag_control_climb_rate_enabled;bool flag_control_termination_enabled;bool flag_control_fixed_hdg_enabled;uint8_t[7] _padding0;";

ORB_DEFINE(vehicle_control_mode, struct vehicle_control_mode_s, 25, orb_vehicle_control_mode_fields, 1);

