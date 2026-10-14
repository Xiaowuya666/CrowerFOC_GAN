#ifndef CROWER_COGGING_COMPENSATION_H
#define CROWER_COGGING_COMPENSATION_H

#include <math.h>
#include <stdint.h>

#define CFF_COGGING_BINS 128u
#define CFF_COGGING_TWO_PI 6.28318530718f
#define CFF_COGGING_MAX_COMP_A 1.0f
#define CFF_COGGING_MIN_SAMPLES 8u

/* Angle is the motor-side encoder angle in radians, wrapped by this helper. */
static inline unsigned cff_cogging_bin(float angle)
{
  if (!isfinite(angle)) { return 0u; }
  angle = fmodf(angle, CFF_COGGING_TWO_PI);
  if (angle < 0.0f) { angle += CFF_COGGING_TWO_PI; }
  unsigned bin = (unsigned)(angle * ((float)CFF_COGGING_BINS / CFF_COGGING_TWO_PI));
  return bin < CFF_COGGING_BINS ? bin : 0u;
}

/* During a constant-speed sweep, average forward/reverse Iq cancels most
 * Coulomb friction. The remaining periodic component is stored as mA. */
static inline int cff_cogging_finish(const float *forward_sum,
                                    const float *reverse_sum,
                                    const uint16_t *forward_count,
                                    const uint16_t *reverse_count,
                                    int16_t *map_ma)
{
  float raw[CFF_COGGING_BINS];
  float mean = 0.0f;
  for (unsigned i = 0u; i < CFF_COGGING_BINS; ++i) {
    if (forward_count[i] < CFF_COGGING_MIN_SAMPLES ||
        reverse_count[i] < CFF_COGGING_MIN_SAMPLES) { return 0; }
    /* Both sweeps store the velocity regulator's required Iq command. Their
     * average preserves cogging compensation and cancels direction-dependent
     * Coulomb friction; this current is added directly to the Iq reference. */
    raw[i] = 0.5f * (forward_sum[i] / (float)forward_count[i] +
                      reverse_sum[i] / (float)reverse_count[i]);
    mean += raw[i];
  }
  mean /= (float)CFF_COGGING_BINS;
  for (unsigned i = 0u; i < CFF_COGGING_BINS; ++i) {
    float smooth = (raw[(i + CFF_COGGING_BINS - 2u) % CFF_COGGING_BINS] +
                    2.0f * raw[(i + CFF_COGGING_BINS - 1u) % CFF_COGGING_BINS] +
                    3.0f * raw[i] +
                    2.0f * raw[(i + 1u) % CFF_COGGING_BINS] +
                    raw[(i + 2u) % CFF_COGGING_BINS]) * 0.11111111111f - mean;
    if (smooth > CFF_COGGING_MAX_COMP_A) { smooth = CFF_COGGING_MAX_COMP_A; }
    if (smooth < -CFF_COGGING_MAX_COMP_A) { smooth = -CFF_COGGING_MAX_COMP_A; }
    map_ma[i] = (int16_t)(smooth * 1000.0f);
  }
  return 1;
}

static inline float cff_cogging_lookup_a(const int16_t *map_ma, float angle,
                                         float strength)
{
  if (!isfinite(angle) || !isfinite(strength)) { return 0.0f; }
  angle = fmodf(angle, CFF_COGGING_TWO_PI);
  if (angle < 0.0f) { angle += CFF_COGGING_TWO_PI; }
  float index = angle * ((float)CFF_COGGING_BINS / CFF_COGGING_TWO_PI);
  unsigned lo = (unsigned)index;
  if (lo >= CFF_COGGING_BINS) { lo = 0u; index = 0.0f; }
  unsigned hi = (lo + 1u) % CFF_COGGING_BINS;
  float f = index - (float)lo;
  float value = ((float)map_ma[lo] + f * ((float)map_ma[hi] - (float)map_ma[lo])) * 0.001f * strength;
  if (value > CFF_COGGING_MAX_COMP_A) { value = CFF_COGGING_MAX_COMP_A; }
  if (value < -CFF_COGGING_MAX_COMP_A) { value = -CFF_COGGING_MAX_COMP_A; }
  return value;
}

#endif
