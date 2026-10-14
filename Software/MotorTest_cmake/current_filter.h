#ifndef CROWER_CURRENT_FILTER_H
#define CROWER_CURRENT_FILTER_H

/* 25 kHz current-loop input conditioning, separate from raw OC protection.
 * Median(3) rejects a one-period impulse; a sustained step passes on its second
 * sample. EMA alpha=0.5 has a 2.88 kHz -3 dB corner. Median plus EMA gives
 * about 80 us low-frequency group delay at this ISR rate. L pulse measurements
 * MUST use raw samples instead.
 * Same coefficients on all phases; no artificial enforcement of Ia+Ib+Ic=0.
 */
typedef struct {
  float previous[3], older[3], output[3];
  unsigned initialized;
} ph_current_filter_t;

static inline float ph_median3(float a, float b, float c)
{
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { b = c; }
  return a > b ? a : b;
}

static inline unsigned ph_current_filter_step(ph_current_filter_t *f, const float raw[3])
{
  unsigned impulses = 0;
  for (unsigned i = 0; i < 3; ++i) {
    if (!f->initialized) {
      f->previous[i] = f->older[i] = f->output[i] = raw[i];
    } else {
      float median = ph_median3(raw[i], f->previous[i], f->older[i]);
      float delta = raw[i] - median;
      if (delta > 2.0f || delta < -2.0f) { impulses |= 1u << i; }
      f->output[i] += 0.5f * (median - f->output[i]);
      f->older[i] = f->previous[i];
      f->previous[i] = raw[i];
    }
  }
  f->initialized = 1;
  return impulses; /* diagnostic threshold only; not a current clamp */
}
#endif
