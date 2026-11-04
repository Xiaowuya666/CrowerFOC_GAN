#ifndef MIT_CAN_PROTOCOL_H
#define MIT_CAN_PROTOCOL_H
#include <stdint.h>
#include <math.h>

/* MIT-compatible 8-byte bit layout with CrowerFOC current-only semantics.
 * The final 12-bit field is Iq feedforward in amperes. */
#define CFF_MIT_P_MIN (-12.5f)
#define CFF_MIT_P_MAX ( 12.5f)
#define CFF_MIT_V_MIN (-30.0f)
#define CFF_MIT_V_MAX ( 30.0f)
#define CFF_MIT_KP_MIN (0.0f)
#define CFF_MIT_KP_MAX (500.0f)
#define CFF_MIT_KD_MIN (0.0f)
#define CFF_MIT_KD_MAX (5.0f)
#define CFF_MIT_I_MIN (-40.0f)
#define CFF_MIT_I_MAX ( 40.0f)
#define CFF_MIT_CURRENT_FULL_SCALE_A (40.0f)

#define CFF_MIT_CMD_ENABLE  0xFCu
#define CFF_MIT_CMD_DISABLE 0xFDu
#define CFF_MIT_CMD_ZERO    0xFEu

typedef struct {
  float position_rad;
  float velocity_rad_s;
  float kp_a_per_rad;
  float kd_a_per_rad_s;
  float iq_ff_A;
} cff_mit_command_t;

/* Unsigned cycle counters wrap naturally; elapsed-time watchdog checks must
 * remain correct across that wrap. Keep this helper shared with native tests. */
static inline int cff_mit_watchdog_expired(uint32_t now, uint32_t last,
                                            uint32_t timeout_cycles) {
  return (uint32_t)(now - last) > timeout_cycles;
}

/* FC is an arming request, not a way for CAN to take over a live USB axis. */
static inline int cff_mit_can_enable_allowed(uint8_t inhibited, uint8_t faulted,
                                              uint8_t calibrated, uint8_t auto_arm,
                                              uint8_t idle, uint16_t modulation) {
  return !inhibited && !faulted && calibrated && auto_arm && idle && modulation == 0u;
}

static inline uint32_t cff_mit_uint(float x,float lo,float hi,unsigned bits) {
  if(!isfinite(x)) x=0.0f;
  if(x<lo) x=lo;
  if(x>hi) x=hi;
  uint32_t max=(1u<<bits)-1u;
  float scaled=(x-lo)*(float)max/(hi-lo);
  uint32_t value=(uint32_t)(scaled+0.5f);
  return value>max?max:value;
}

static inline int16_t cff_i16_saturate(float x) {
  if(!isfinite(x)) return 0;
  if(x>32767.0f) return 32767;
  if(x<-32768.0f) return -32768;
  return (int16_t)x;
}

static inline float cff_mit_float(uint32_t x,float lo,float hi,unsigned bits) {
  uint32_t max=(1u<<bits)-1u;
  if(x>max) x=max;
  return (float)x*(hi-lo)/(float)max+lo;
}

static inline int cff_mit_special(const uint8_t *p,unsigned len,uint8_t *opcode) {
  if(!p||len!=8u) return 0;
  for(unsigned i=0;i<7u;i++) if(p[i]!=0xFFu) return 0;
  if(p[7]!=CFF_MIT_CMD_ENABLE&&p[7]!=CFF_MIT_CMD_DISABLE&&p[7]!=CFF_MIT_CMD_ZERO) return 0;
  if(opcode) *opcode=p[7];
  return 1;
}

static inline int cff_mit_decode(const uint8_t *p,unsigned len,cff_mit_command_t *out) {
  if(!p||!out||len!=8u) return 0;
  uint32_t pi=((uint32_t)p[0]<<8)|p[1];
  uint32_t vi=((uint32_t)p[2]<<4)|(p[3]>>4);
  uint32_t kpi=((uint32_t)(p[3]&0x0Fu)<<8)|p[4];
  uint32_t kdi=((uint32_t)p[5]<<4)|(p[6]>>4);
  uint32_t ii=((uint32_t)(p[6]&0x0Fu)<<8)|p[7];
  out->position_rad=cff_mit_float(pi,CFF_MIT_P_MIN,CFF_MIT_P_MAX,16);
  out->velocity_rad_s=cff_mit_float(vi,CFF_MIT_V_MIN,CFF_MIT_V_MAX,12);
  out->kp_a_per_rad=cff_mit_float(kpi,CFF_MIT_KP_MIN,CFF_MIT_KP_MAX,12);
  out->kd_a_per_rad_s=cff_mit_float(kdi,CFF_MIT_KD_MIN,CFF_MIT_KD_MAX,12);
  out->iq_ff_A=cff_mit_float(ii,CFF_MIT_I_MIN,CFF_MIT_I_MAX,12);
  return 1;
}

static inline void cff_mit_feedback(uint8_t p[8],unsigned node,float position_rad,
                                    float velocity_rad_s,float iq_A,float motor_temp_C,
                                    uint8_t fault) {
  if(!p) return;
  uint16_t pos=(uint16_t)cff_mit_uint(position_rad,CFF_MIT_P_MIN,CFF_MIT_P_MAX,16);
  uint16_t vel=(uint16_t)cff_mit_uint(velocity_rad_s,CFF_MIT_V_MIN,CFF_MIT_V_MAX,12);
  uint16_t current=(uint16_t)cff_mit_uint(iq_A,CFF_MIT_I_MIN,CFF_MIT_I_MAX,12);
  p[0]=(uint8_t)node; p[1]=(uint8_t)(pos>>8); p[2]=(uint8_t)pos;
  p[3]=(uint8_t)(vel>>4); p[4]=(uint8_t)((vel&0x0Fu)<<4)|(uint8_t)(current>>8);
  p[5]=(uint8_t)current;
  p[6]=isfinite(motor_temp_C)?(uint8_t)fminf(255.0f,fmaxf(0.0f,motor_temp_C+40.0f)):0xFFu;
  p[7]=fault;
}
#endif
