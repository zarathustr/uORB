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


#include <uORB/topics/vehicle_attitude_setpoint.h>

static constexpr char orb_vehicle_attitude_setpoint_fields[] =
  "uint64_t timestamp;float roll_body;float pitch_body;float yaw_body;float yaw_sp_move_rate;float[4] q_d;float thrust;float landing_gear;bool q_d_valid;bool roll_reset_integral;bool pitch_reset_integral;bool yaw_reset_integral;bool fw_control_yaw;bool disable_mc_yaw_control;bool apply_flaps;uint8_t[1] _padding0;";

ORB_DEFINE(vehicle_attitude_setpoint, struct vehicle_attitude_setpoint_s, 55, orb_vehicle_attitude_setpoint_fields, 1);
ORB_DEFINE(mc_virtual_attitude_setpoint, struct vehicle_attitude_setpoint_s, 55, orb_vehicle_attitude_setpoint_fields, 1);
ORB_DEFINE(fw_virtual_attitude_setpoint, struct vehicle_attitude_setpoint_s, 55, orb_vehicle_attitude_setpoint_fields, 1);

