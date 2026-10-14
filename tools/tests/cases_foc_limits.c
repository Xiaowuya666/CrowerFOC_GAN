#include "../../Software/MotorTest_cmake/foc_limits.h"
#include "../../Software/MotorTest_cmake/calibration_profile.h"
#include "../../Software/MotorTest_cmake/cogging_compensation.h"

int test_entry(void)
{
  float id = 0.0f, iq = 0.0f, vd = 0.0f, vq = 0.0f;
  /* Long infeasible demand must not accumulate an unbounded integrator. */
  for (unsigned i = 0; i < 20000; i++) {
    ph_current_pi_step(20, 20, .05f, 100, .000025f, 3, &id, &iq, &vd, &vq);
    if (vd*vd + vq*vq > 9.0001f || fabsf(id) > 4 || fabsf(iq) > 4) { return 1; }
  }
  for (unsigned i = 0; i < 100; i++) {
    ph_current_pi_step(-20, -20, .05f, 100, .000025f, 3, &id, &iq, &vd, &vq);
  }
  if (!(vd < 0 && vq < 0)) { return 2; }
  /* Same code against three RL plants, with feasible and infeasible EMF. */
  for (unsigned plant = 0; plant < 3; plant++) {
    float resistance = .096f, inductance = .000124f;
    float emf = plant == 0 ? 0 : (plant == 1 ? 1.5f : 5.0f);
    float current = 0.0f;
    id = iq = 0;
    for (unsigned i = 0; i < 8000; i++) {
      ph_current_pi_step(0, 1-current, .05f, 100, .000025f, 3, &id, &iq, &vd, &vq);
      current += .000025f * (vq - emf - resistance*current) / inductance;
      if (fabsf(vq) > 3.00001f || fabsf(iq) > 5.0f) { return 3; }
    }
    if (plant < 2 && fabsf(current - 1) > .02f) { return 4; }
    /* Anti-windup cannot overcome EMF above the available voltage. */
    if (plant == 2 && !(current < -15.0f)) { return 5; }
  }
  float limit = ph_svpwm_voltage_limit(23.1f, 1800);
  if (!(limit > 15.0f && limit < 15.3f) ||
      !(ph_svpwm_linear_voltage_limit(23.1f)>13.1f)) { return 10; }
  /* Near-linear SVPWM limits must retain two valid inline-current samples
     and select a trigger point between phase switching edges for every angle. */
  for (unsigned k=0; k<72u; ++k) {
    float angle=6.28318530718f*(float)k/72.0f, v=ph_svpwm_linear_voltage_limit(23.1f)*.999f;
    float a=v*cosf(angle), b=v*cosf(angle-2.09439510239f), c=v*cosf(angle+2.09439510239f);
    unsigned da,db,dc,q,mask;
    float scale=ph_svpwm_duty_sample(a,b,c,23.1f,1800,0u,&da,&db,&dc,&q,&mask);
    unsigned valid=(mask&1u)+((mask>>1u)&1u)+((mask>>2u)&1u);
    if (scale<.70f||da>1800u||db>1800u||dc>1800u||valid<2u||q>1480u) { return 21; }
  }
  for(unsigned k=0;k<72u;k++) {
    float angle=6.28318530718f*(float)k/72.0f,v=ph_svpwm_linear_voltage_limit(23.1f)*.999f;
    unsigned da,db,dc,q,mask;
    float scale=ph_svpwm_duty_sample(v*cosf(angle),v*cosf(angle-2.09439510239f),
      v*cosf(angle+2.09439510239f),23.1f,1800,1u,&da,&db,&dc,&q,&mask);
    unsigned valid=(mask&1u)+((mask>>1u)&1u)+((mask>>2u)&1u);
    if(scale<.70f||valid<2u||q<PH_PWM_SAMPLE_TRIGGER_MIN||q>1760u)return 28;
  }
  /* Exercise the intended 1.14x linear overmodulation envelope. */
  for(unsigned k=0;k<72u;k++) {
    float angle=6.28318530718f*(float)k/72.0f,v=23.1f*(2.0f/3.0f)*.99f;
    unsigned da,db,dc,q,mask;
    float scale=ph_svpwm_duty_sample(v*cosf(angle),v*cosf(angle-2.09439510239f),
      v*cosf(angle+2.09439510239f),23.1f,1800,1u,&da,&db,&dc,&q,&mask);
    unsigned valid=(mask&1u)+((mask>>1u)&1u)+((mask>>2u)&1u);
    if(scale<.97f||valid<2u||q<PH_PWM_SAMPLE_TRIGGER_MIN||
       da>1800u||db>1800u||dc>1800u)return 29;
  }
  /* 25 kHz build: keep the hardware ADC compare fixed at the PWM center,
     use common-mode shift to avoid the 50% phase-edge collision, and retain
     two valid current ranks through the intended overmodulation envelope. */
  unsigned da,db,dc,q,mask;
  float scale=ph_svpwm_duty_sample(0,0,0,23.1f,2880,0u,&da,&db,&dc,&q,&mask);
  if(scale<.97f||q!=1440u||__builtin_popcount(mask&7u)<2u||
     (da==1440u&&db==1440u&&dc==1440u))return 30;
  for(unsigned k=0;k<72u;k++) {
    float angle=6.28318530718f*(float)k/72.0f,v=23.1f*(2.0f/3.0f)*.99f;
    scale=ph_svpwm_duty_sample(v*cosf(angle),v*cosf(angle-2.09439510239f),
      v*cosf(angle+2.09439510239f),23.1f,2880,0u,&da,&db,&dc,&q,&mask);
    if(scale<.90f||q!=1440u||__builtin_popcount(mask&7u)<2u||
       da>2880u||db>2880u||dc>2880u)return 31;
  }
  float currents[3]={2.0f,-.7f,99.0f};
  if(!ph_reconstruct_two_currents(currents,3u)||fabsf(currents[2]+1.3f)>.0001f) return 26;
  if(ph_reconstruct_two_currents(currents,1u)) return 27;
  float r, intercept;
  if (!ph_cal_fit_r(.955f, 2.5f, 1.195f, 5.0f, &r, &intercept)) { return 11; }
  if (fabsf(r-.096f) > .0001f || fabsf(intercept-.715f) > .0001f) { return 12; }
  if (ph_cal_fit_r(.9f, 2.5f, 4.0f, 5.0f, &r, &intercept)) { return 13; }
  if (ph_cal_fit_r(.9f, 2.5f, 1.2f, 3.0f, &r, &intercept)) { return 14; }
  if(ph_cal_bw_total_ticks(25000u)!=PH_CAL_BW_TEST_TICKS||PH_CAL_BW_TEST_TICKS!=5463u)return 32;
  float bw[PH_CAL_BW_TEST_COUNT]={1.0f,1.0f,1.0f,.8f,.5f,.2f};
  float cutoff=ph_cal_bw_cutoff_hz(bw);
  if(!(cutoff>1100.0f&&cutoff<1200.0f))return 33;
  float flat[PH_CAL_BW_TEST_COUNT]={1,1,1,1,1,1};
  if(ph_cal_bw_cutoff_hz(flat)!=2001.0f)return 35;
  float sum_sin=100.0f*.5f*.1f*.8f*cosf(.523598776f);
  float sum_cos=-100.0f*.5f*.1f*.8f*sinf(.523598776f);
  if(fabsf(ph_cal_bw_gain(sum_sin,sum_cos,100u,.1f)-.8f)>.001f||
     fabsf(ph_cal_bw_lag_deg(sum_sin,sum_cos)-30.0f)>.01f)return 34;
  for(unsigned i=0;i<PH_CAL_BW_TEST_COUNT;i++) {
    unsigned f=ph_cal_bw_frequency(i);
    float command=ph_cal_bw_command(.09f,.000085f,f);
    float z=hypotf(.09f,6.28318530718f*(float)f*.000085f);
    if(!(command>0.0f&&command<=PH_CAL_BW_COMMAND_A&&command*z<=PH_CAL_MAX_VOLTAGE*.7001f))return 36;
  }
  if(ph_cal_progress(6,0)!=0.0f||ph_cal_progress(6,PH_CAL_BW_TEST_TICKS)!=100.0f)return 37;
  if (ph_cal_progress(3, 0) >= ph_cal_progress(3, 100000)) { return 15; }
  float alpha, beta;
  ph_clarke_three(1.0f, -.5f, -.5f, &alpha, &beta);
  if (fabsf(alpha-1) > .00001f || fabsf(beta) > .00001f) { return 16; }
  ph_clarke_three(11.0f, 9.5f, 9.5f, &alpha, &beta);
  if (fabsf(alpha-1) > .00001f || fabsf(beta) > .00001f) { return 17; }
  if (ph_tle_delay_us(0x8000,0) != 150 || ph_tle_delay_us(0x4000,4) != 45 ||
      ph_tle_delay_us(0,0) >= 0) { return 18; }
  /* Noisy, wrap-crossing 80rad/s shaft sampled every 500us: estimator must
     keep velocity/phase coherent without differentiating raw angle noise. */
  ph_angle_observer_t observer = {0};
  float raw_variance = 0, observed_variance = 0, previous = 0;
  for (unsigned i=0; i<2000; i++) {
    float measured = ph_angle_error(80*.0005f*(float)i + ((i&1u) ? .0008f : -.0008f));
    ph_angle_observer_step(&observer,measured,.0005f,300);
    if (i>200) {
      float raw_error = ph_angle_error(measured-previous)/.0005f-80;
      raw_variance += raw_error*raw_error;
      float velocity_error = observer.velocity-80;
      observed_variance += velocity_error*velocity_error;
      if (fabsf(ph_angle_error(observer.angle-80*.0005f*(float)i)) > .001f) { return 19; }
    }
    previous = measured;
  }
  if (!(observed_variance < .05f*raw_variance)) { return 20; }
  float forward[CFF_COGGING_BINS], reverse[CFF_COGGING_BINS];
  uint16_t nf[CFF_COGGING_BINS], nr[CFF_COGGING_BINS];
  int16_t cog[CFF_COGGING_BINS];
  for (unsigned i=0; i<CFF_COGGING_BINS; ++i) {
    float angle=6.28318530718f*((float)i+.5f)/(float)CFF_COGGING_BINS;
    float cogging=.2f*sinf(angle);
    forward[i]=8.0f*(cogging+.1f); /* forward Coulomb friction */
    reverse[i]=8.0f*(cogging-.1f); /* reverse friction has opposite sign */
    nf[i]=nr[i]=8u;
  }
  if (!cff_cogging_finish(forward,reverse,nf,nr,cog)) { return 22; }
  for (unsigned i=0; i<CFF_COGGING_BINS; i+=8u) {
    float angle=6.28318530718f*((float)i+.5f)/(float)CFF_COGGING_BINS;
    if (fabsf(cff_cogging_lookup_a(cog,angle,1.0f)-.2f*sinf(angle))>.015f) { return 23; }
  }
  if (cff_cogging_bin(-.001f)>=CFF_COGGING_BINS ||
      fabsf(cff_cogging_lookup_a(cog,0.0f,0.0f))>.00001f) { return 24; }
  nf[0]=0u;
  if (cff_cogging_finish(forward,reverse,nf,nr,cog)) { return 25; }
  return 0;
}
