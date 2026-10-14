#ifndef CROWER_FOC_LIMITS_H
#define CROWER_FOC_LIMITS_H
#include <math.h>
#include <stdint.h>

/* Sampling is placed between phase switching edges. Require at least two
 * phase-current apertures to be 240 timer counts (1.67 us) from an edge. */
#define PH_PWM_SAMPLE_GUARD 240u
#define PH_PWM_MIN_EDGE 20u
#define PH_PWM_SAMPLE_MARGIN 480u /* legacy V/f fallback only */
#define PH_PWM_SAMPLE_TRIGGER_MIN 420u /* 2.9 us: ADC scan is ~328 TIM1 counts */
#define PH_ADC_RANK_CENTER_A 38u
#define PH_ADC_RANK_CENTER_B 140u
#define PH_ADC_RANK_CENTER_C 242u
#define PH_ADC_SAMPLE_HALF 26u

typedef struct { float angle, velocity; unsigned initialized; } ph_angle_observer_t;

static inline float ph_angle_error(float angle)
{
  while (angle > 3.14159265359f) { angle -= 6.28318530718f; }
  while (angle < -3.14159265359f) { angle += 6.28318530718f; }
  return angle;
}

/* Sample-time alpha/beta observer; update ONCE per new SSC word, not per ISR.
 * Its phase tracks a constant velocity without the lag of an angle lowpass. */
static inline void ph_angle_observer_step(ph_angle_observer_t *o, float measured,
                                          float dt, float bandwidth)
{
  if (!o->initialized || !(dt > 0.000001f && dt < 0.003f)) {
    o->angle = measured; o->velocity = 0.0f; o->initialized = 1u; return;
  }
  float gain = bandwidth * dt;
  if (gain > 0.25f) { gain = 0.25f; }
  float predicted = o->angle + o->velocity * dt;
  float error = ph_angle_error(measured - predicted);
  o->angle = ph_angle_error(predicted + 2.0f * gain * error);
  o->velocity += gain * gain * error / dt;
}

static inline float ph_tle_delay_us(unsigned mod1, unsigned mod2)
{
  unsigned filter = (mod1 >> 14u) & 3u;
  if (mod2 & 4u) {
    return filter == 1u ? 45.0f : (filter == 2u ? 65.0f : (filter == 3u ? 105.0f : -1.0f));
  }
  return filter == 1u ? 85.0f : (filter == 2u ? 150.0f : (filter == 3u ? 275.0f : -1.0f));
}

static inline void ph_clarke_three(float ia, float ib, float ic, float *alpha, float *beta)
{
  /* Reject zero-sequence sensor error. Raw Ia+Ib+Ic remains in diagnostics and OC. */
  *alpha = (2.0f * ia - ib - ic) / 3.0f;
  *beta = (ib - ic) * 0.57735026919f;
}

/* Circular alpha/beta voltage limit for SVPWM. The guaranteed phase duty
 * window is +/- (0.5 - sample_margin/period). Centered common-mode injection
 * uses the worst-case phase span sqrt(3)*|V|; retain 1% for quantization and
 * dead-time. */
static inline float ph_svpwm_linear_voltage_limit(float vbus)
{
  return vbus * 0.57735026919f * 0.99f;
}

static inline float ph_svpwm_voltage_limit(float vbus, unsigned period)
{
  (void)period;
  /* Allow a voltage vector up to 99% of a space-vector hexagon vertex.
   * The angle-dependent edge constraint is applied in the PWM mapper. */
  return vbus * (2.0f/3.0f) * 0.99f;
}

/* Legacy safe modulation: all three compares retain the quiet window around
 * the update-triggered ADC scan. This is intentionally not a 100% duty mode. */
static inline void ph_svpwm_duty(float va,float vb,float vc,float vbus,
                                 unsigned period,unsigned *da,unsigned *db,unsigned *dc)
{
  float maxv=fmaxf(va,fmaxf(vb,vc)), minv=fminf(va,fminf(vb,vc));
  float common=-0.5f*(maxv+minv);
  const unsigned lo=PH_PWM_SAMPLE_MARGIN;
  const unsigned hi=period>PH_PWM_SAMPLE_MARGIN?period-PH_PWM_SAMPLE_MARGIN:lo;
  float x[3]={va+common,vb+common,vc+common}; unsigned *out[3]={da,db,dc};
  for(unsigned i=0;i<3u;i++) {
    int duty=(int)((0.5f+x[i]/vbus)*(float)period);
    if(duty<(int)lo)duty=(int)lo; else if(duty>(int)hi)duty=(int)hi;
    *out[i]=(unsigned)duty;
  }
}

static inline unsigned ph_absdiff_u(unsigned a,unsigned b) { return a>b?a-b:b-a; }

/* Select a compare-event sample point that leaves two or more inline phase
 * shunts quiet through their ADC apertures. ADC rank centers are measured in
 * TIM1 counts from the CH4 trigger. */
static inline int ph_svpwm_sample_point(unsigned a,unsigned b,unsigned c,unsigned period,
                                        unsigned descending,unsigned *sample,unsigned *valid_mask)
{
  (void)descending;
  unsigned edges[5]={0u,a,b,c,period};
  for(unsigned i=1u;i<4u;i++) for(unsigned j=i;j>0u&&edges[j]<edges[j-1u];j--) {
    unsigned t=edges[j];edges[j]=edges[j-1u];edges[j-1u]=t;
  }
  const unsigned offset[3]={PH_ADC_RANK_CENTER_A,PH_ADC_RANK_CENTER_B,PH_ADC_RANK_CENTER_C};
  unsigned best_count=0u,best_clearance=0u,best_sample=period/2u,best_mask=0u;
  for(unsigned gap=0u;gap<4u;gap++) {
    unsigned q=(edges[gap]+edges[gap+1u])/2u;
    /* Keep compare sampling close to the timer extremum. This guarantees the
       scan has settled before the next update interrupt, while still moving
       away from phase edges that coincide with the nominal center point. */
    unsigned qlo=(period*35u)/100u, qhi=(period*65u)/100u;
    if(q<qlo) q=qlo;
    if(q>qhi) q=qhi;
    unsigned cc[3]={a,b,c},mask_up=0u,mask_down=0u,min_clear=UINT32_MAX;
    for(unsigned i=0u;i<3u;i++) {
      unsigned phase_mask[2]={0u,0u};
      for(unsigned dir=0u;dir<2u;dir++) {
        unsigned pos=dir?(q-offset[i]):(q+offset[i]);
        unsigned before=pos+cc[i];
        unsigned at=ph_absdiff_u(pos,cc[i]);
        unsigned after=2u*period-cc[i]-pos;
        unsigned d=before<at?before:at;if(after<d)d=after;
        if(d>=PH_PWM_SAMPLE_GUARD+PH_ADC_SAMPLE_HALF) {
          phase_mask[dir]=1u;
          unsigned clearance=d-PH_ADC_SAMPLE_HALF;
          if(clearance<min_clear)min_clear=clearance;
        }
      }
      if(phase_mask[0])mask_up|=1u<<i;
      if(phase_mask[1])mask_down|=1u<<i;
    }
    unsigned mask=mask_up&mask_down;
    unsigned count=(unsigned)__builtin_popcount(mask);
    if(count>best_count||(count==best_count&&min_clear>best_clearance)) {
      best_count=count;best_clearance=min_clear;best_sample=q;best_mask=mask;
    }
  }
  *sample=best_sample;*valid_mask=best_mask;
  return best_count>=2u;
}

/* Bounded-cost sample validation for the 40 kHz ISR. The ADC can finish a
 * three-rank scan before either timer extremum when triggered at mid-period. */
static inline int ph_svpwm_fixed_sample_mask(unsigned a,unsigned b,unsigned c,unsigned period,
                                             unsigned q,unsigned *valid_mask)
{
  const unsigned cc[3]={a,b,c};
  const unsigned offset[3]={PH_ADC_RANK_CENTER_A,PH_ADC_RANK_CENTER_B,PH_ADC_RANK_CENTER_C};
  unsigned mask=0u;
  for(unsigned i=0u;i<3u;i++) {
    int valid=1;
    for(unsigned dir=0u;dir<2u;dir++) {
      unsigned pos=dir?(q-offset[i]):(q+offset[i]);
      unsigned before=pos+cc[i],at=ph_absdiff_u(pos,cc[i]),after=2u*period-cc[i]-pos;
      unsigned d=before<at?before:at;if(after<d)d=after;
      if(d<PH_PWM_SAMPLE_GUARD+PH_ADC_SAMPLE_HALF) {valid=0;break;}
    }
    if(valid)mask|=1u<<i;
  }
  *valid_mask=mask;
  return __builtin_popcount(mask)>=2;
}

/* Centered SVPWM with fixed-phase ADC sampling. CH4 stays at the timer center,
 * which gives a repeatable conversion before the control update. A common-mode
 * shift moves the three phase compares together to clear that sample aperture;
 * the line-to-line voltage is unchanged. Back off voltage only if no feasible
 * common-mode placement leaves two inline shunts quiet. */
static inline float ph_svpwm_duty_sample(float va,float vb,float vc,float vbus,unsigned period,
                                         unsigned descending,
                                         unsigned *da,unsigned *db,unsigned *dc,
                                         unsigned *sample,unsigned *valid_mask)
{
  if (!(vbus>0.1f) || period<800u) {
    *da=*db=*dc=period/2u; *sample=period/2u; *valid_mask=0u; return 0.0f;
  }
  const float requested_scales[3]={1.0f,0.90f,0.95f};
  unsigned best_a=period/2u,best_b=period/2u,best_c=period/2u;
  unsigned best_q=period/2u,best_mask=0u;
  float best=0.0f;
  for(unsigned iter=0u;iter<3u;iter++) {
    float scale=requested_scales[iter];
    float sa=va*scale,sb=vb*scale,sc=vc*scale;
    float maxv=fmaxf(sa,fmaxf(sb,sc)),minv=fminf(sa,fminf(sb,sc));
    float common=-0.5f*(maxv+minv);
    int ia=(int)((0.5f+(sa+common)/vbus)*(float)period);
    int ib=(int)((0.5f+(sb+common)/vbus)*(float)period);
    int ic=(int)((0.5f+(sc+common)/vbus)*(float)period);
    if(ia<0)ia=0;else if(ia>(int)period)ia=(int)period;
    if(ib<0)ib=0;else if(ib>(int)period)ib=(int)period;
    if(ic<0)ic=0;else if(ic>(int)period)ic=(int)period;
    int lo=-(ia<ib?(ia<ic?ia:ic):(ib<ic?ib:ic));
    int hi=(int)period-(ia>ib?(ia>ic?ia:ic):(ib>ic?ib:ic));
    unsigned local_a=0u,local_b=0u,local_c=0u,local_mask=0u,local_count=0u;
    int local_shift=0;
    /* This five-point common-mode set covers the ADC-safe windows across the
       electrical cycle with much less ISR work than a full sweep. */
    const int offsets[5]={0,-1,1,-2,2};
    for(unsigned candidate=0u;candidate<5u;candidate++) {
      int shift=offsets[candidate]*(int)period/8;
      if(shift<lo)shift=lo;
      if(shift>hi)shift=hi;
      unsigned a=(unsigned)(ia+shift),b=(unsigned)(ib+shift),c=(unsigned)(ic+shift),mask;
      (void)ph_svpwm_fixed_sample_mask(a,b,c,period,period/2u,&mask);
      unsigned count=(unsigned)__builtin_popcount(mask&7u);
      unsigned shift_mag=(unsigned)(shift<0?-shift:shift);
      unsigned local_mag=(unsigned)(local_shift<0?-local_shift:local_shift);
      if(count>local_count || (count==local_count && count>=2u && shift_mag<local_mag)) {
        local_count=count;local_shift=shift;local_a=a;local_b=b;local_c=c;local_mask=mask;
      }
    }
    if(local_count>=2u) {
      best=scale;best_a=local_a;best_b=local_b;best_c=local_c;best_q=period/2u;best_mask=local_mask;
      if(iter!=1u)break;
    }
  }
  (void)descending;
  *da=best_a;*db=best_b;*dc=best_c;*sample=best_q;*valid_mask=best_mask;
  return best;
}

/* Reconstruct the missing inline phase from Kirchhoff's current law. */
static inline int ph_reconstruct_two_currents(float current[3],unsigned valid_mask)
{
  unsigned count=0u,missing=3u;
  for(unsigned i=0u;i<3u;i++) if(valid_mask&(1u<<i)) count++; else missing=i;
  if(count==3u)return 1;
  if(count!=2u)return 0;
  current[missing]=0.0f;
  for(unsigned i=0u;i<3u;i++) if(i!=missing) current[missing]-=current[i];
  return 1;
}

static inline void ph_current_pi_step(float ed, float eq, float kp, float ki,
                                     float dt, float vmax, float *integral_d,
                                     float *integral_q, float *vd, float *vq)
{
  float d = kp * ed + *integral_d + ki * ed * dt;
  float q = kp * eq + *integral_q + ki * eq * dt;
  float mag = sqrtf(d*d + q*q);
  if (mag > vmax && mag > 0.000001f) { d *= vmax / mag; q *= vmax / mag; }
  /* Back-calculate the integral from achievable voltage, in every mode. */
  *integral_d = d - kp * ed;
  *integral_q = q - kp * eq;
  *vd = d; *vq = q;
}

static inline void ph_current_pi_ff_step(float ed,float eq,float kp,float ki,float dt,float vmax,
                                         float ffd,float ffq,float *integral_d,float *integral_q,float *vd,float *vq)
{
  float d=kp*ed+*integral_d+ki*ed*dt+ffd;
  float q=kp*eq+*integral_q+ki*eq*dt+ffq;
  float mag=sqrtf(d*d+q*q);
  if(mag>vmax&&mag>0.000001f) {d*=vmax/mag;q*=vmax/mag;}
  *integral_d=d-kp*ed-ffd;*integral_q=q-kp*eq-ffq;*vd=d;*vq=q;
}

#endif
