/**
  * @file    current_loop_test.c
  * @brief   40 kHz cycle-by-cycle current-loop compute test.
  *
  *  The TIM1 update interrupt is used as the 40 kHz trigger. On every event a
  *  representative FOC current loop is executed:
  *      feedback -> Clarke -> Park (sin/cos) -> 2x PI -> inverse Park
  *      -> SVPWM (min/max injection) -> 3x CCR update
  *  and the number of CPU cycles it takes is measured with the independent TIM2 cycle
  *  counter. The MCU runs at 144 MHz, so the budget for 40 kHz is
  *      144e6 / 40e3 = 3600 cycles (= 25 us).
  */
#include "current_loop_test.h"
#include "cycle_clock.h"
#include "mx_tim1.h"
#include "mx_adc1.h"
#include "stm32_hal.h"
#include "stm32c5xx_hal_adc.h"
#include "stm32c5xx_ll_adc.h"
#include <math.h>

/* Must match the PWM period used by pwm_test.c (center-aligned 40 kHz). */
#define CL_PWM_PERIOD        1799.0f
#define CL_BUDGET_CYCLES     3600u

volatile uint32_t g_cl_count      = 0u;
volatile uint32_t g_cl_min_cycles = 0xFFFFFFFFu;
volatile uint32_t g_cl_max_cycles = 0u;
volatile uint32_t g_cl_last_cycles = 0u;
volatile uint32_t g_cl_sum_cycles = 0u;
volatile uint32_t g_cl_overruns   = 0u;
volatile uint32_t g_cl_adc_timeouts = 0u;

static hal_adc_handle_t *s_adc = NULL;
static float s_theta = 0.0f;
static float s_id_int = 0.0f;
static float s_iq_int = 0.0f;

void current_loop_test_init(void)
{
  ph_clock_init();
  hal_tim_handle_t *tim = mx_tim1_gethandle();
  if (tim == NULL)
  {
    return;
  }

  s_adc = mx_adc1_gethandle();

  /* TIM1 TRGO = update event -> the ADC1 regular group (IN4/IN5/IN6) is
     hardware-triggered once per PWM period, at the same instant as this ISR. */
  (void)HAL_TIM_SetTriggerOutput(tim, HAL_TIM_TRGO_UPDATE);

  /* Arm the ADC for external-triggered conversions. */
  if (s_adc != NULL)
  {
    (void)HAL_ADC_Start(s_adc);
    LL_ADC_REG_StartConversion(ADC1);   /* set ADSTART (HAL_ADC_Start only enables) */
  }

  /* One update event per PWM period (center-aligned -> every 2 over/underflows). */
  (void)HAL_TIM_SetRepetitionCounter(tim, 1u);

  NVIC_SetPriority(TIM1_UPD_IRQn, 1u);
  NVIC_EnableIRQ(TIM1_UPD_IRQn);
  (void)HAL_TIM_Start_IT(tim);
}

void current_loop_test_run(void)
{
  hal_tim_handle_t *tim = mx_tim1_gethandle();
  uint32_t t0 = ph_clock_now();

  /* --- wait for the ADC sequence TRIGGERED BY TIM1_TRGO, then read the three
         phase currents. The wait models the ADC sampling+conversion latency. --- */
  uint32_t guard = 0u;
  while (LL_ADC_IsActiveFlag_EOS(ADC1) == 0u)
  {
    if (++guard > 3000u)
    {
      g_cl_adc_timeouts++;
      break;
    }
  }
  LL_ADC_ClearFlag_EOS(ADC1);

  float ia = (float)HAL_ADC_REG_ReadConversionData(s_adc) * (1.0f / 4096.0f) - 0.5f;
  float ib = (float)HAL_ADC_REG_ReadConversionData(s_adc) * (1.0f / 4096.0f) - 0.5f;
  (void)HAL_ADC_REG_ReadConversionData(s_adc);   /* phase C (ic = -ia-ib) */

  /* Re-arm for the next PWM period (ADSTART is cleared by HW after a sequence). */
  LL_ADC_REG_StartConversion(ADC1);

  /* --- Clarke --- */
  float alpha = ia;
  float beta  = 0.5773502692f * (ia + 2.0f * ib);

  /* --- Park (sin/cos: the heaviest part) --- */
  float s = sinf(s_theta);
  float c = cosf(s_theta);
  float id =  alpha * c + beta * s;
  float iq = -alpha * s + beta * c;

  /* --- two PI controllers --- */
  float ed = 0.05f - id;
  float eq = 0.10f - iq;
  s_id_int += ed;
  s_iq_int += eq;
  float vd = 1.20f * ed + 0.010f * s_id_int;
  float vq = 1.20f * eq + 0.010f * s_iq_int;

  /* --- inverse Park --- */
  float valpha = vd * c - vq * s;
  float vbeta  = vd * s + vq * c;

  /* --- inverse Clarke to phase voltages --- */
  float da = valpha;
  float db = -0.5f * valpha + 0.8660254f * vbeta;
  float dc = -0.5f * valpha - 0.8660254f * vbeta;

  /* --- SVPWM: min/max common-mode injection --- */
  float vmax = da;
  float vmin = da;
  if (db > vmax) { vmax = db; }
  if (dc > vmax) { vmax = dc; }
  if (db < vmin) { vmin = db; }
  if (dc < vmin) { vmin = dc; }
  float hoff = -0.5f * (vmax + vmin);

  /* --- map to duty and write the three compare units --- */
  const float half   = 0.5f * CL_PWM_PERIOD;
  const float invlim = CL_PWM_PERIOD / (2.0f * 200.0f);   /* +/-200 V full scale */

  int32_t pa = (int32_t)(half + (da + hoff) * invlim);
  int32_t pb = (int32_t)(half + (db + hoff) * invlim);
  int32_t pc = (int32_t)(half + (dc + hoff) * invlim);

  if (pa < 0) { pa = 0; } if (pa > (int32_t)CL_PWM_PERIOD) { pa = (int32_t)CL_PWM_PERIOD; }
  if (pb < 0) { pb = 0; } if (pb > (int32_t)CL_PWM_PERIOD) { pb = (int32_t)CL_PWM_PERIOD; }
  if (pc < 0) { pc = 0; } if (pc > (int32_t)CL_PWM_PERIOD) { pc = (int32_t)CL_PWM_PERIOD; }

  (void)HAL_TIM_OC_SetCompareUnitPulse(tim, HAL_TIM_OC_COMPARE_UNIT_1, (uint32_t)pa);
  (void)HAL_TIM_OC_SetCompareUnitPulse(tim, HAL_TIM_OC_COMPARE_UNIT_2, (uint32_t)pb);
  (void)HAL_TIM_OC_SetCompareUnitPulse(tim, HAL_TIM_OC_COMPARE_UNIT_3, (uint32_t)pc);

  /* advance the electrical angle for the next iteration */
  s_theta += 0.0015f;
  if (s_theta > 6.2831853f) { s_theta -= 6.2831853f; }

  /* --- measure --- */
  uint32_t dt = ph_clock_now() - t0;
  g_cl_last_cycles = dt;
  if (dt < g_cl_min_cycles) { g_cl_min_cycles = dt; }
  if (dt > g_cl_max_cycles) { g_cl_max_cycles = dt; }
  g_cl_sum_cycles += dt;
  g_cl_count++;
  if (dt > CL_BUDGET_CYCLES) { g_cl_overruns++; }
}

/* TIM1 update interrupt service routine. */
void TIM1_UPD_IRQHandler(void)
{
  HAL_TIM_IRQHandler(mx_tim1_gethandle());
}

/* Called by HAL_TIM_IRQHandler() on the update event. */
void HAL_TIM_UpdateCallback(hal_tim_handle_t *htim)
{
  (void)htim;
  current_loop_test_run();
}
