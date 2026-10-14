#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H
#include <math.h>
#include <string.h>
#include "axis_config.h"
#define CFF_POSITION_MAX_OUTPUT_RPM 15.0f
#define CFF_POSITION_MAX_RAD_S (CFF_POSITION_MAX_OUTPUT_RPM * 0.10471975512f)
#define CFF_POSITION_STEP_OUTPUT_RAD 0.5235987756f /* 30 degrees at output per command */
static inline float cff_clamp(float x,float lo,float hi) { return fminf(hi,fmaxf(lo,x)); }
static inline float cff_slew(float x,float target,float rate,float dt) {
  return x+cff_clamp(target-x,-rate*dt,rate*dt);
}
static inline float cff_position_error(float target,float pos,const cff_config_t *c) {
  float e=target-pos;
  if(c->circular_enable) { e=fmodf(e,c->circular_range); if(e>c->circular_range/2) e-=c->circular_range;
    if(e<-c->circular_range/2) e+=c->circular_range; }
  return e;
}
/* Exact integration across trapezoid boundaries. Replans first brake the
   existing reference velocity, then move from the resulting stopping point. */
typedef struct { float p,v,a,target,t[4],acc[4]; unsigned phase; int done; } cff_traj_t;
static inline void cff_traj_plan(cff_traj_t *t,float p,float v,float target,float vmax,float accel,float decel) {
  memset(t,0,sizeof(*t)); t->p=p; t->v=v; t->target=target;
  t->t[0]=fabsf(v)/decel; t->acc[0]=v>0?-decel:decel;
  float stop=p+v*t->t[0]*0.5f, distance=target-stop, sign=distance>=0?1.0f:-1.0f;
  float peak=fminf(vmax,sqrtf(2*fabsf(distance)/(1/accel+1/decel)));
  t->t[1]=peak/accel; t->acc[1]=sign*accel;
  t->t[2]=peak>0?fmaxf(0,(fabsf(distance)-peak*peak*0.5f*(1/accel+1/decel))/peak):0;
  t->t[3]=peak/decel; t->acc[3]=-sign*decel;
}
static inline void cff_traj_step(cff_traj_t *t,float dt) {
  for(unsigned i=0;i<5 && dt>0 && t->phase<4;i++) {
    unsigned ph=t->phase; float step=fminf(dt,t->t[ph]); t->a=t->acc[ph];
    t->p+=t->v*step+0.5f*t->a*step*step; t->v+=t->a*step;
    t->t[ph]-=step; dt-=step;
    if(t->t[ph]<=0.0000001f) { t->phase++; if(ph==0 || ph==3) t->v=0; }
  }
  if(t->phase>=4) { t->p=t->target; t->v=t->a=0; t->done=1; }
}
typedef struct { float pos,vel,current,integral,last_target; cff_traj_t traj; int initialized; } cff_motion_t;
static inline void cff_motion_reset(cff_motion_t *m,float pos,float vel) {
  memset(m,0,sizeof(*m)); m->pos=pos; m->vel=vel; m->last_target=NAN; m->initialized=1; m->traj.done=1;
}
static inline float cff_torque_to_current(float torque,const cff_config_t *c) {
  return c->torque_units_nm ? torque/c->torque_constant : torque;
}
static inline float cff_motion_step(cff_motion_t *m,const cff_config_t *c,float pos,float vel,float limit,float dt) {
  float torque_ff=cff_torque_to_current(c->input_torque,c);
  if(c->control_mode==CFF_CTRL_TORQUE) {
    float target=torque_ff;
    if(c->input_mode==5) target=cff_slew(m->current,target,cff_torque_to_current(c->torque_ramp,c),dt);
    m->integral=0; m->current=target;
    if(c->torque_vel_limit) target=cff_clamp(target,c->vel_gain*(-c->limit_vel-vel),c->vel_gain*(c->limit_vel-vel));
    return cff_clamp(target,-limit,limit);
  }
  float vt=cff_clamp(c->input_vel,-c->limit_vel,c->limit_vel), acceleration=0;
  if(c->control_mode==CFF_CTRL_POSITION) {
    /* Position commands are deliberately slow and bounded. A stale or
       accidental absolute target must not invoke the full motor speed limit. */
    float pos_vmax=fminf(c->limit_vel,CFF_POSITION_MAX_RAD_S);
    float target=m->pos+cff_position_error(c->input_pos,m->pos,c);
    if(c->input_mode==CFF_INPUT_TRAP_TRAJ) {
      if(c->input_pos!=m->last_target) {
        cff_traj_plan(&m->traj,m->pos,m->vel,target,fminf(c->traj_vel,pos_vmax),c->traj_accel,c->traj_decel);
        m->last_target=c->input_pos;
      }
      cff_traj_step(&m->traj,dt); m->pos=m->traj.p; m->vel=m->traj.v; acceleration=m->traj.a;
      vt=m->vel;
    } else if(c->input_mode==CFF_INPUT_POS_FILTER) {
      float w=6.2831853f*c->input_filter_bandwidth;
      acceleration=cff_clamp(w*w*(target-m->pos)+2*w*(vt-m->vel),-c->traj_decel,c->traj_accel);
      m->vel=cff_clamp(m->vel+acceleration*dt,-pos_vmax,pos_vmax); m->pos+=m->vel*dt; vt=m->vel;
    } else { m->pos=target; m->vel=cff_clamp(vt,-pos_vmax,pos_vmax); vt=m->vel; }
    vt+=c->pos_gain*cff_position_error(m->pos,pos,c);
    torque_ff+=c->torque_constant>0?c->inertia*acceleration/c->torque_constant:0;
  } else if(c->input_mode==CFF_INPUT_VEL_RAMP) { m->vel=cff_slew(m->vel,vt,c->vel_ramp,dt); vt=m->vel; }
  float vmax=(c->control_mode==CFF_CTRL_POSITION)?fminf(c->limit_vel,CFF_POSITION_MAX_RAD_S):c->limit_vel;
  vt=cff_clamp(vt,-vmax,vmax);
  if(c->pos_limits_enable) {
    float hi=sqrtf(2*c->traj_decel*fmaxf(0,c->pos_max-pos));
    float lo=-sqrtf(2*c->traj_decel*fmaxf(0,pos-c->pos_min)); vt=cff_clamp(vt,lo,hi);
  }
  float error=vt-vel, proposed=m->integral+c->vel_integrator_gain*error*dt;
  float output=c->vel_gain*error+proposed+torque_ff;
  if((output<=limit&&output>=-limit)||(output>limit&&error<0)||(output<-limit&&error>0)) m->integral=proposed;
  m->integral=cff_clamp(m->integral,-limit,limit);
  m->current=cff_clamp(c->vel_gain*error+m->integral+torque_ff,-limit,limit);
  return m->current;
}
static inline float cff_temperature_derating(float temperature,float lower,float upper) {
  return cff_clamp((upper-temperature)/(upper-lower),0,1);
}
static inline float cff_ptc_temperature(uint16_t adc,float r25,float slope_ohm_per_c) {
  if(adc<8u||adc>4087u) return NAN;
  float resistance=10000.0f*(float)adc/(4095.0f-(float)adc);
  if(!isfinite(resistance)||!isfinite(r25)||!isfinite(slope_ohm_per_c)||
     r25<=0.0f||slope_ohm_per_c<=0.0f) return NAN;
  return 25.0f+(resistance-r25)/slope_ohm_per_c;
}
static inline float cff_ntc_temperature(uint16_t adc,float r25,float beta) {
  if(adc<8u||adc>4087u) return NAN;
  float resistance=10000.0f*(float)adc/(4095.0f-(float)adc);
  return 1.0f/(1.0f/298.15f+logf(resistance/r25)/beta)-273.15f;
}
#endif
