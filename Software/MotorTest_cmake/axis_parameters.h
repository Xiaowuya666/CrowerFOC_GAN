#ifndef AXIS_PARAMETERS_H
#define AXIS_PARAMETERS_H
#include <stddef.h>
#include "axis_config.h"
typedef struct { uint16_t id,offset; uint8_t kind; float lo,hi; } cff_param_desc_t;
#define FP(id,field,lo,hi) {id,offsetof(cff_config_t,field),0,lo,hi}
#define BP(id,field) {id,offsetof(cff_config_t,field),1,0,1}
#define UP(id,field,lo,hi) {id,offsetof(cff_config_t,field),2,lo,hi}
static const cff_param_desc_t cff_params[]={
  UP(0x0100,pole_pairs,1,64), FP(0x0101,phase_resistance,0.00001f,100),
  FP(0x0102,phase_inductance,0.0000001f,1), FP(0x0103,current_limit,0.01f,40),
  FP(0x0105,torque_constant,0,100), BP(0x0106,torque_units_nm), FP(0x0107,current_bandwidth,50,CFF_CURRENT_BANDWIDTH_MAX_RAD_S),
  FP(0x0200,encoder_offset,-6.283186f,6.283186f), {0x0201,offsetof(cff_config_t,encoder_dir),3,-1,1},
  FP(0x0202,encoder_bandwidth,10,800), FP(0x0300,pos_gain,0,1000),
  FP(0x0301,vel_gain,0,10000), FP(0x0302,vel_integrator_gain,0,100000),
  FP(0x0303,current_gain_p,0,10000), FP(0x0304,current_gain_i,0,1000000),
  FP(0x0305,vel_ramp,0.01f,10000), FP(0x0306,torque_ramp,0.001f,10000),
  FP(0x0307,input_filter_bandwidth,0.1f,100), BP(0x0308,model_ff_enable),
  FP(0x0310,traj_vel,0.01f,CFF_AXIS_MAX_RAD_S), FP(0x0311,traj_accel,0.01f,10000),
  FP(0x0312,traj_decel,0.01f,10000), FP(0x0313,inertia,0,1000),
  FP(0x0400,limit_vel,0.01f,CFF_MOTOR_MAX_RAD_S), FP(0x0401,current_limit,0.01f,40),
  FP(0x0402,overspeed_limit,0.01f,CFF_AXIS_TRIP_RAD_S),
  FP(0x0410,pos_min,-1000000,1000000), FP(0x0411,pos_max,-1000000,1000000), BP(0x0412,pos_limits_enable),
  FP(0x0413,pos_zero,-1000000,1000000), BP(0x0414,circular_enable), FP(0x0415,circular_range,0.001f,1000000),
  FP(0x0420,bus_uv,5,30), FP(0x0421,bus_uv_recover,6,32), FP(0x0422,bus_ov,17,34),
  FP(0x0430,fet_temp_lower,30,100), FP(0x0431,fet_temp_upper,40,120),
  BP(0x0432,motor_temp_enable), FP(0x0433,motor_temp_lower,30,150), FP(0x0434,motor_temp_upper,40,180),
  FP(0x0435,motor_ntc_beta,0.1f,100), FP(0x0436,motor_ntc_r25,100,1000000),
  BP(0x0437,fet_temp_enable),
  FP(0x0440,regen_current,0,100), FP(0x0441,input_power,0,10000),
  {0x0500,offsetof(cff_config_t,control_mode),1,0,2}, {0x0501,offsetof(cff_config_t,input_mode),1,1,5},
  FP(0x0503,input_pos,-1000000,1000000), FP(0x0504,input_vel,-CFF_MOTOR_MAX_RAD_S,CFF_MOTOR_MAX_RAD_S), FP(0x0505,input_torque,-1000,1000),
  UP(0x0800,can_node,1,7), UP(0x0801,can_bitrate_kbps,125,1000)
};
#undef FP
#undef BP
#undef UP
static inline const cff_param_desc_t *cff_param_find(uint16_t id) {
  for(unsigned i=0;i<sizeof(cff_params)/sizeof(cff_params[0]);i++) if(cff_params[i].id==id) return &cff_params[i];
  return 0;
}
static inline float cff_param_read(const cff_config_t *c,const cff_param_desc_t *d) {
  const uint8_t *p=(const uint8_t*)c+d->offset;
  if(d->kind==1) return *p;
  if(d->kind==2) return *(const uint16_t*)p;
  if(d->kind==3) return *(const int8_t*)p;
  return *(const float*)p;
}
static inline int cff_param_write(cff_config_t *c,const cff_param_desc_t *d,float v) {
  if(!isfinite(v)||v<d->lo||v>d->hi||(d->kind&&v!=floorf(v))) return 0;
  if(d->kind==3&&v!=1&&v!=-1) return 0;
  uint8_t *p=(uint8_t*)c+d->offset;
  if(d->kind==1) *p=(uint8_t)v; else if(d->kind==2) *(uint16_t*)p=(uint16_t)v;
  else if(d->kind==3) *(int8_t*)p=(int8_t)v; else *(float*)p=v;
  return 1;
}
#endif
