#ifndef CROWER_CALIBRATION_PROFILE_H
#define CROWER_CALIBRATION_PROFILE_H
#include <math.h>

/* Fixed commissioning profile, separate from the normal running current limit.
 * Motor ratings do not establish the board's continuous/peak rating. */
#define PH_CAL_CURRENT_A 5.0f
#define PH_CAL_CURRENT_LIMIT_A 6.0f
#define PH_CAL_RAW_TRIP_A 8.0f
#define PH_CAL_MAX_VOLTAGE 4.0f
#define PH_CAL_R_LEVEL_TICKS 16000u
#define PH_CAL_R_TICKS (2u * PH_CAL_R_LEVEL_TICKS)
#define PH_CAL_L_START 4000u
#define PH_CAL_L_REPEATS 16u
#define PH_CAL_L_PULSE_MAX_TICKS 16u
#define PH_CAL_L_CYCLE_TICKS 816u /* 0.64 ms max pulse + 32 ms zero-current settling */
#define PH_CAL_L_TICKS (PH_CAL_L_START + PH_CAL_L_REPEATS * PH_CAL_L_CYCLE_TICKS)
#define PH_CAL_LOCK_TICKS 32000u
#define PH_CAL_PP_TICKS 1050000u
#define PH_CAL_STATIC_TICKS 32000u
#define PH_CAL_STATIC_SETTLE 20000u
#define PH_CAL_SWEEP_LEG_TICKS 80000u
#define PH_CAL_SWEEP_SETTLE 20000u
#define PH_CAL_SWEEP_TICKS (2u * PH_CAL_SWEEP_LEG_TICKS)
#define PH_CAL_BW_TEST_COUNT 16u
#define PH_CAL_BW_SETTLE_CYCLES 3u
#define PH_CAL_BW_MEASURE_CYCLES 20u
/* Larger stationary d-axis excitation reduces inverter dead-time/device-drop
 * error in the low-voltage small-signal test. D-axis current ideally produces
 * no torque, but alignment error and reluctance torque can still cause motion. */
#define PH_CAL_BW_COMMAND_A 5.0f
/* PI design target is raised 25% to compensate the measured loop shortfall;
 * the intended closed-loop -3 dB bandwidth is approximately 1 kHz. */
#define PH_CAL_CURRENT_BW_TARGET_HZ 1250u

/* Commissioning must not reuse a potentially stale operating PI profile.
 * Before identification use conservative controller defaults (not motor R/L).
 * Afterwards design a slower calibration loop from this run's measurements. */
static inline void ph_cal_current_gains(unsigned stage, float resistance,
                                        float inductance, float *kp, float *ki)
{
  *kp=0.05f; *ki=100.0f;
  if (stage>=2u && isfinite(resistance) && isfinite(inductance) &&
      resistance>0.0f && inductance>0.0f) {
    const float bandwidth=6.28318530718f*250.0f;
    *kp=inductance*bandwidth; *ki=resistance*bandwidth;
  }
}

static inline unsigned ph_cal_bw_frequency(unsigned index)
{
  static const unsigned frequencies[PH_CAL_BW_TEST_COUNT] = {
    100u, 150u, 200u, 300u, 400u, 500u, 650u, 800u,
    1000u, 1250u, 1500u, 1800u, 2200u, 2700u, 3300u, 4000u
  };
  return index < PH_CAL_BW_TEST_COUNT ? frequencies[index] : 0u;
}

static inline unsigned ph_cal_bw_settle_ticks(unsigned sample_hz, unsigned index)
{
  unsigned f = ph_cal_bw_frequency(index);
  return f ? (sample_hz * PH_CAL_BW_SETTLE_CYCLES + f/2u) / f : 0u;
}

static inline unsigned ph_cal_bw_measure_ticks(unsigned sample_hz, unsigned index)
{
  unsigned f = ph_cal_bw_frequency(index);
  return f ? (sample_hz * PH_CAL_BW_MEASURE_CYCLES + f/2u) / f : 0u;
}

static inline unsigned ph_cal_bw_total_ticks(unsigned sample_hz)
{
  unsigned total = 0u;
  for (unsigned i=0u; i<PH_CAL_BW_TEST_COUNT; ++i)
    total += ph_cal_bw_settle_ticks(sample_hz,i) + ph_cal_bw_measure_ticks(sample_hz,i);
  return total;
}

static inline float ph_cal_bw_command(float resistance,float inductance,unsigned frequency)
{
  float omega=6.28318530718f*(float)frequency;
  float impedance=sqrtf(resistance*resistance+omega*omega*inductance*inductance);
  if(!(impedance>0.00001f) || !isfinite(impedance)) return 0.0f;
  return fminf(PH_CAL_BW_COMMAND_A,0.70f*PH_CAL_MAX_VOLTAGE/impedance);
}

static inline float ph_cal_bw_gain(float sum_sin,float sum_cos,unsigned samples,float command)
{
  return samples ? (2.0f * sqrtf(sum_sin*sum_sin + sum_cos*sum_cos) /
                    (float)samples) / command : 0.0f;
}

static inline float ph_cal_bw_lag_deg(float sum_sin,float sum_cos)
{
  float lag = -atan2f(sum_cos,sum_sin) * (180.0f / 3.14159265359f);
  if (lag < 0.0f) lag += 360.0f;
  return lag;
}

static inline float ph_cal_bw_cutoff_hz(const float gain[PH_CAL_BW_TEST_COUNT])
{
  const float threshold = 0.70710678f;
  if (!isfinite(gain[0]) || gain[0] < threshold) return 0.0f;
  float baseline=gain[0];
  if (baseline > 1.5f) baseline=1.0f; /* reject resonance as the passband reference */
  float previous_gain=gain[0]/baseline;
  for (unsigned i=1u; i<PH_CAL_BW_TEST_COUNT; ++i) {
    if (!isfinite(gain[i])) return 0.0f;
    /* Use the descending envelope. Noise or resonance must not make a later
       high-frequency point hide the first -3 dB crossing. */
    float normalized=gain[i]/baseline;
    float current_gain=fminf(previous_gain,normalized);
    if (current_gain < threshold) {
      float f0=(float)ph_cal_bw_frequency(i-1u);
      float f1=(float)ph_cal_bw_frequency(i);
      float g0=previous_gain,g1=current_gain;
      float fraction=(threshold-g0)/(g1-g0);
      if(fraction<0.0f)fraction=0.0f; else if(fraction>1.0f)fraction=1.0f;
      return f0+(f1-f0)*fraction;
    }
    previous_gain=current_gain;
  }
  return (float)ph_cal_bw_frequency(PH_CAL_BW_TEST_COUNT-1u)+1.0f; /* +1 means cutoff is above sweep range. */
}

#define PH_CAL_TOTAL_TICKS (PH_CAL_R_TICKS + PH_CAL_L_TICKS + PH_CAL_LOCK_TICKS + PH_CAL_PP_TICKS + PH_CAL_STATIC_TICKS + PH_CAL_SWEEP_TICKS)

static inline unsigned ph_cal_stage_ticks(unsigned stage)
{
  switch (stage) {
    case 0: return PH_CAL_R_TICKS;
    case 1: return PH_CAL_L_TICKS;
    case 2: return PH_CAL_LOCK_TICKS;
    case 3: return PH_CAL_PP_TICKS;
    case 4: return PH_CAL_STATIC_TICKS;
    case 5: return PH_CAL_SWEEP_TICKS;
    case 6: return ph_cal_bw_total_ticks(25000u); /* standalone current-loop sweep */
    case 7: return 0u; /* complete; cogging compensation disabled */
    default: return 0u;
  }
}

static inline float ph_cal_progress(unsigned stage, unsigned ticks)
{
  if (stage == 6u) {
    unsigned total=ph_cal_bw_total_ticks(25000u);
    return 100.0f * (float)(ticks < total ? ticks : total) / (float)total;
  }
  if (stage >= 7u) { return 100.0f; }
  unsigned elapsed = 0u;
  for (unsigned i = 0u; i < stage; ++i) { elapsed += ph_cal_stage_ticks(i); }
  unsigned duration = ph_cal_stage_ticks(stage);
  elapsed += ticks < duration ? ticks : duration;
  return 100.0f * (float)elapsed / (float)PH_CAL_TOTAL_TICKS;
}

static inline float ph_cal_r_target(unsigned ticks)
{
  unsigned level = ticks / PH_CAL_R_LEVEL_TICKS;
  unsigned local = ticks % PH_CAL_R_LEVEL_TICKS;
  float from = level ? PH_CAL_CURRENT_A * 0.5f : 0.0f;
  float to = level ? PH_CAL_CURRENT_A : PH_CAL_CURRENT_A * 0.5f;
  float fraction = (float)local / 4000.0f; /* 100 ms ramp */
  if (fraction > 1.0f) { fraction = 1.0f; }
  return from + (to - from) * fraction;
}

/* Same-polarity two-level slope separates a nearly constant voltage error.
 * This remains an inverter+motor estimate; gain/Rds/contact errors persist. */
static inline int ph_cal_fit_r(float vlo, float ilo, float vhi, float ihi,
                               float *resistance, float *voltage_error)
{
  if (!(ilo > 2.2f && ilo < 2.8f && ihi > 4.5f && ihi < 5.5f &&
        vhi < PH_CAL_MAX_VOLTAGE - 0.1f && vlo > 0.0f && vhi > vlo)) { return 0; }
  float r = (vhi - vlo) / (ihi - ilo);
  if (!(r > 0.00001f && r < 4.0f)) { return 0; }
  *resistance = r;
  *voltage_error = vlo - r * ilo;
  return 1;
}
#endif
