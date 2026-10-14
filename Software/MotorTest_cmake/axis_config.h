#ifndef AXIS_CONFIG_H
#define AXIS_CONFIG_H
#include <stdint.h>
#include <math.h>
#include <float.h>
#include "crowerfoc_proto.h"
#include "calibration_profile.h"
/* Motion coordinates are measured at the output encoder. The former fixed
   reduction-ratio conversion is intentionally removed. */
#define CFF_AXIS_MAX_RPM 230.0f
#define CFF_AXIS_TRIP_RPM 253.0f
#define CFF_MOTOR_MAX_RPM 2100.0f
#define CFF_AXIS_MAX_RAD_S (CFF_AXIS_MAX_RPM * 0.10471975512f)
#define CFF_AXIS_TRIP_RAD_S (CFF_AXIS_TRIP_RPM * 0.10471975512f)
#define CFF_MOTOR_MAX_RAD_S (CFF_MOTOR_MAX_RPM * 0.10471975512f)
/* 1.25 kHz PI design target compensates the measured ~0.64 kHz response
   toward an approximately 1 kHz closed-loop -3 dB bandwidth. */
#define CFF_CURRENT_BANDWIDTH_DEFAULT_RAD_S 7853.98163397f
#define CFF_CURRENT_BANDWIDTH_MAX_RAD_S 10000.0f
/* Versioned persistent configuration. Runtime targets are never restored. */
typedef struct {
  uint16_t pole_pairs;
  float phase_resistance, phase_inductance, current_limit, calibration_current;
  float encoder_offset;
  int8_t encoder_dir;
  float encoder_bandwidth, pos_gain, vel_gain, vel_integrator_gain;
  float current_gain_p, current_gain_i, limit_vel, overspeed_limit;
  uint8_t control_mode, input_mode;
  float input_pos, input_vel, input_torque;
  float torque_constant, current_bandwidth, vel_ramp, torque_ramp;
  float input_filter_bandwidth, traj_vel, traj_accel, traj_decel, inertia;
  float pos_min, pos_max, pos_zero, circular_range;
  float bus_uv, bus_uv_recover, bus_ov;
  float fet_temp_lower, fet_temp_upper, motor_temp_lower, motor_temp_upper;
  float motor_ntc_beta, motor_ntc_r25, regen_current, input_power;
  uint16_t can_node, can_bitrate_kbps, can_heartbeat_ms, can_encoder_ms;
  uint8_t torque_units_nm, torque_vel_limit, pos_limits_enable, circular_enable;
  uint8_t motor_temp_enable, model_ff_enable, fet_temp_enable;
} cff_config_t;

/* Clamp saved motor-speed configuration without inventing a gearbox ratio. */
static inline int cff_config_migrate_speed_units(cff_config_t *c) {
  int changed=0;
  if(fabsf(c->limit_vel-CFF_AXIS_MAX_RAD_S)<0.01f){c->limit_vel=CFF_MOTOR_MAX_RAD_S;changed=1;}
  if(c->limit_vel>CFF_MOTOR_MAX_RAD_S){c->limit_vel=CFF_MOTOR_MAX_RAD_S;changed=1;}
  if(c->overspeed_limit>CFF_AXIS_TRIP_RAD_S){c->overspeed_limit=CFF_AXIS_TRIP_RAD_S;changed=1;}
  if(c->traj_vel>CFF_AXIS_MAX_RAD_S){c->traj_vel=CFF_AXIS_MAX_RAD_S;changed=1;}
  return changed;
}

/* TEMP2 was previously described as an unverified NTC. Keep the record layout
   unchanged and translate its former B-value into the measured two-point PTC
   line on load. The R25 and slope fields retain their binary positions. */
static inline int cff_config_migrate_motor_ptc(cff_config_t *c) {
  if (c->motor_ntc_beta > 100.0f) {
    c->motor_ntc_beta = 9.9130435f; /* ohm / degree C */
    c->motor_ntc_r25 = 620.0f;
    return 1;
  }
  return 0;
}

static inline int cff_input_compatible(unsigned control, unsigned input) {
  return input == 1u || (input == 2u && control == 1u) ||
    ((input == 3u || input == 4u) && control == 2u) || (input == 5u && control == 0u);
}
static inline int cff_config_valid(const cff_config_t *c) {
  /* Check every persisted float, including fields not active in this mode. */
#define F(x,lo,hi) (isfinite(c->x) && c->x >= (lo) && c->x <= (hi))
  int valid = c->pole_pairs <= 64u && (c->encoder_dir == 1 || c->encoder_dir == -1) &&
    F(phase_resistance,0,100) && F(phase_inductance,0,1) && F(current_limit,0.01f,40) &&
    c->calibration_current == PH_CAL_CURRENT_A && F(encoder_offset,0,6.283186f) &&
    F(encoder_bandwidth,10,800) && F(pos_gain,0,1000) && F(vel_gain,0,10000) &&
    F(vel_integrator_gain,0,100000) && F(current_gain_p,0,10000) && F(current_gain_i,0,1000000) &&
    F(limit_vel,0.01f,CFF_MOTOR_MAX_RAD_S) && F(overspeed_limit,0.01f,CFF_AXIS_TRIP_RAD_S) &&
    F(input_pos,-1000000,1000000) && F(input_vel,-CFF_MOTOR_MAX_RAD_S,CFF_MOTOR_MAX_RAD_S) && F(input_torque,-1000,1000) &&
    c->control_mode <= 2u && cff_input_compatible(c->control_mode,c->input_mode) &&
    F(torque_constant,0,100) && F(current_bandwidth,50,CFF_CURRENT_BANDWIDTH_MAX_RAD_S) && F(vel_ramp,0.01f,10000) &&
    F(torque_ramp,0.001f,10000) && F(input_filter_bandwidth,0.1f,100) &&
    F(traj_vel,0.01f,1000) && F(traj_accel,0.01f,10000) && F(traj_decel,0.01f,10000) &&
    F(inertia,0,1000) && F(pos_min,-1000000,1000000) && F(pos_max,-1000000,1000000) &&
    c->pos_min < c->pos_max && F(pos_zero,-1000000,1000000) && F(circular_range,0.001f,1000000) &&
    F(bus_uv,5,30) && F(bus_uv_recover,6,32) && F(bus_ov,17,34) &&
    c->bus_uv < c->bus_uv_recover && c->bus_uv_recover < c->bus_ov &&
    F(fet_temp_lower,30,100) && F(fet_temp_upper,40,120) && c->fet_temp_lower < c->fet_temp_upper &&
    F(motor_temp_lower,30,150) && F(motor_temp_upper,40,180) && c->motor_temp_lower < c->motor_temp_upper &&
    ((F(motor_ntc_beta,0.1f,100.0f)) || F(motor_ntc_beta,1000,6000)) &&
    F(motor_ntc_r25,100,1000000) &&
    F(regen_current,0,100) && F(input_power,0,10000) && c->can_node >= 1u && c->can_node <= 7u &&
    (c->can_bitrate_kbps == 125u || c->can_bitrate_kbps == 250u ||
     c->can_bitrate_kbps == 500u || c->can_bitrate_kbps == 1000u) &&
    c->can_heartbeat_ms <= 60000u && c->can_encoder_ms <= 60000u &&
    c->torque_units_nm <= 1u && c->torque_vel_limit <= 1u && c->pos_limits_enable <= 1u &&
    c->circular_enable <= 1u && c->motor_temp_enable <= 1u && c->model_ff_enable <= 1u && c->fet_temp_enable<=1u &&
    (!c->torque_units_nm || c->torque_constant > 0.0f) &&
    (!c->model_ff_enable || (c->torque_constant > 0 && c->phase_resistance > 0 && c->phase_inductance > 0)) &&
    !(c->pos_limits_enable && c->circular_enable);
#undef F
  return valid;
}
#endif
