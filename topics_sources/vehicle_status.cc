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


#include <uORB/topics/vehicle_status.h>

static constexpr char orb_vehicle_status_fields[] =
  "uint64_t timestamp;uint32_t system_id;uint32_t component_id;uint32_t onboard_control_sensors_present;uint32_t onboard_control_sensors_enabled;uint32_t onboard_control_sensors_health;uint8_t nav_state;uint8_t arming_state;uint8_t hil_state;bool failsafe;uint8_t system_type;bool is_rotary_wing;bool is_vtol;bool vtol_fw_permanent_stab;bool in_transition_mode;bool in_transition_to_fw;bool rc_signal_lost;uint8_t rc_input_mode;bool data_link_lost;uint8_t data_link_lost_counter;bool engine_failure;bool engine_failure_cmd;bool mission_failure;uint8_t[3] _padding0;";

ORB_DEFINE(vehicle_status, struct vehicle_status_s, 45, orb_vehicle_status_fields, 1);

