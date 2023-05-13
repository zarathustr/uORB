/****************************************************************************
 *
 *   Copyright (C) 2013-2020 PX4 Development Team. All rights reserved.
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

#include <uORB/uorbNew/uorb.h>
#include <uORB/topics/uorb_topics.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_direct.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/camera_capture.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/collision_report.h>
#include <uORB/topics/commander_state.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/ekf2_replay.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/esc_report.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/example_string.h>
#include <uORB/topics/fence.h>
#include <uORB/topics/fence_vertex.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/gps_dump.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/hil_sensor.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/mount_orientation.h>
#include <uORB/topics/msg_template.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/output_pwm.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/qshell_req.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/rc_parameter_map.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_image.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_preflight.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/servorail_status.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/task_stack_info.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/test_motor.h>
#include <uORB/topics/time_offset.h>
#include <uORB/topics/uavcan_parameter_request.h>
#include <uORB/topics/uavcan_parameter_value.h>
#include <uORB/topics/ulog_stream.h>
#include <uORB/topics/ulog_stream_ack.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_force_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_roi.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/wind_estimate.h>


const constexpr struct orb_metadata *const uorb_topics_list[] = {
  &uorb::msg::actuator_armed,
  &uorb::msg::actuator_controls,
  &uorb::msg::actuator_controls_0,
  &uorb::msg::actuator_controls_1,
  &uorb::msg::actuator_controls_2,
  &uorb::msg::actuator_controls_3,
  &uorb::msg::actuator_controls_virtual_fw,
  &uorb::msg::actuator_controls_virtual_mc,
  &uorb::msg::actuator_direct,
  &uorb::msg::actuator_outputs,
  &uorb::msg::adc_report,
  &uorb::msg::airspeed,
  &uorb::msg::att_pos_mocap,
  &uorb::msg::battery_status,
  &uorb::msg::camera_capture,
  &uorb::msg::camera_trigger,
  &uorb::msg::collision_report,
  &uorb::msg::commander_state,
  &uorb::msg::control_state,
  &uorb::msg::cpuload,
  &uorb::msg::debug_key_value,
  &uorb::msg::differential_pressure,
  &uorb::msg::distance_sensor,
  &uorb::msg::ekf2_innovations,
  &uorb::msg::ekf2_replay,
  &uorb::msg::ekf2_timestamps,
  &uorb::msg::esc_report,
  &uorb::msg::esc_status,
  &uorb::msg::estimator_status,
  &uorb::msg::example_string,
  &uorb::msg::fence,
  &uorb::msg::fence_vertex,
  &uorb::msg::filtered_bottom_flow,
  &uorb::msg::follow_target,
  &uorb::msg::fw_pos_ctrl_status,
  &uorb::msg::fw_virtual_attitude_setpoint,
  &uorb::msg::fw_virtual_rates_setpoint,
  &uorb::msg::geofence_result,
  &uorb::msg::gps_dump,
  &uorb::msg::gps_inject_data,
  &uorb::msg::hil_sensor,
  &uorb::msg::home_position,
  &uorb::msg::input_rc,
  &uorb::msg::led_control,
  &uorb::msg::log_message,
  &uorb::msg::manual_control_setpoint,
  &uorb::msg::mavlink_log,
  &uorb::msg::mc_att_ctrl_status,
  &uorb::msg::mc_virtual_attitude_setpoint,
  &uorb::msg::mc_virtual_rates_setpoint,
  &uorb::msg::mission,
  &uorb::msg::mission_result,
  &uorb::msg::mount_orientation,
  &uorb::msg::msg_template,
  &uorb::msg::multirotor_motor_limits,
  &uorb::msg::offboard_control_mode,
  &uorb::msg::offboard_mission,
  &uorb::msg::onboard_mission,
  &uorb::msg::optical_flow,
  &uorb::msg::output_pwm,
  &uorb::msg::parameter_update,
  &uorb::msg::position_setpoint,
  &uorb::msg::position_setpoint_triplet,
  &uorb::msg::pwm_input,
  &uorb::msg::qshell_req,
  &uorb::msg::rc_channels,
  &uorb::msg::rc_parameter_map,
  &uorb::msg::safety,
  &uorb::msg::satellite_info,
  &uorb::msg::sensor_accel,
  &uorb::msg::sensor_baro,
  &uorb::msg::sensor_combined,
  &uorb::msg::sensor_correction,
  &uorb::msg::sensor_gyro,
  &uorb::msg::sensor_image,
  &uorb::msg::sensor_mag,
  &uorb::msg::sensor_preflight,
  &uorb::msg::sensor_selection,
  &uorb::msg::servorail_status,
  &uorb::msg::subsystem_info,
  &uorb::msg::system_power,
  &uorb::msg::task_stack_info,
  &uorb::msg::telemetry_status,
  &uorb::msg::test_motor,
  &uorb::msg::time_offset,
  &uorb::msg::uavcan_parameter_request,
  &uorb::msg::uavcan_parameter_value,
  &uorb::msg::ulog_stream,
  &uorb::msg::ulog_stream_ack,
  &uorb::msg::vehicle_attitude,
  &uorb::msg::vehicle_attitude_groundtruth,
  &uorb::msg::vehicle_attitude_setpoint,
  &uorb::msg::vehicle_command,
  &uorb::msg::vehicle_command_ack,
  &uorb::msg::vehicle_control_mode,
  &uorb::msg::vehicle_force_setpoint,
  &uorb::msg::vehicle_global_position,
  &uorb::msg::vehicle_global_position_groundtruth,
  &uorb::msg::vehicle_gps_position,
  &uorb::msg::vehicle_land_detected,
  &uorb::msg::vehicle_local_position,
  &uorb::msg::vehicle_local_position_groundtruth,
  &uorb::msg::vehicle_local_position_setpoint,
  &uorb::msg::vehicle_rates_setpoint,
  &uorb::msg::vehicle_roi,
  &uorb::msg::vehicle_status,
  &uorb::msg::vehicle_status_flags,
  &uorb::msg::vehicle_vision_attitude,
  &uorb::msg::vehicle_vision_position,
  &uorb::msg::vtol_vehicle_status,
  &uorb::msg::wind_estimate
};

const struct orb_metadata *const *orb_get_topics(size_t *size) {
  if (size) *size = sizeof(uorb_topics_list)/sizeof(uorb_topics_list[0]);
  return uorb_topics_list;
}
