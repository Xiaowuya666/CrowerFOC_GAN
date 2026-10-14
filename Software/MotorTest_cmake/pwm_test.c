/**
  * @file    pwm_test.c
  * @brief   3-phase complementary PWM output test.
  *
  *  TIM1 is driven in center-aligned mode at ~20 kHz with complementary
  *  outputs and deadtime on the three phases:
  *      CH1(PA8)/CH1N(PA7)  -> U2 PWMH/PWML   (phase A)
  *      CH2(PA9)/CH2N(PB0)  -> U3 PWMH/PWML   (phase B)
  *      CH3(PA10)/CH3N(PB1) -> U4 PWMH/PWML   (phase C)
  *
  *  The three duties describe a slow (2 Hz electrical) sine set, 120 deg
  *  apart, so the phase relationship can be observed on a scope.
  */
#include "pwm_test.h"
#include "mx_tim1.h"
#include "stm32_hal.h"
#include <math.h>

#define PWM_CHUNK_PI      3.14159265f

/* TIM1 clock = 144 MHz (APB2 = 144 MHz, prescaler = 1).
   Center-aligned: f_pwm = f_tim / (2 * PERIOD)  =>  144e6 / (2*2880) = 25 kHz */
#define PWM_PERIOD        2879u
#define PWM_MID           (PWM_PERIOD / 2u)
#define PWM_AMP           800u           /* duty swing around 50% */
#define PWM_EDGE_MARGIN   40u            /* keep away from 0% and 100% */

#define PWM_DEADTIME      0x50u          /* ~0.55 us with DTS = 1/144 MHz   */

#define ELEC_HZ           2u             /* electrical frequency (slow, for scope) */
#define SINE_LEN          360u

static hal_tim_handle_t *s_tim = NULL;
static int32_t           s_sin[SINE_LEN];

static uint32_t duty_to_pulse(int32_t s /* -1000 .. +1000 */)
{
  int32_t p = (int32_t)PWM_MID + ((int32_t)PWM_AMP * s) / 1000;

  if (p < (int32_t)PWM_EDGE_MARGIN)
  {
    p = (int32_t)PWM_EDGE_MARGIN;
  }
  if (p > (int32_t)(PWM_PERIOD - PWM_EDGE_MARGIN))
  {
    p = (int32_t)(PWM_PERIOD - PWM_EDGE_MARGIN);
  }
  return (uint32_t)p;
}

void pwm_test_init(void)
{
  s_tim = mx_tim1_gethandle();
  if (s_tim == NULL)
  {
    return;
  }

  for (uint32_t i = 0u; i < SINE_LEN; i++)
  {
    s_sin[i] = (int32_t)(sinf(2.0f * PWM_CHUNK_PI * (float)i / (float)SINE_LEN) * 1000.0f);
  }

  /* Center-aligned 20 kHz. */
  (void)HAL_TIM_SetCounterMode(s_tim, HAL_TIM_COUNTER_CENTER_UP_DOWN);
  (void)HAL_TIM_SetPeriod(s_tim, PWM_PERIOD);
  (void)HAL_TIM_SetDeadtime(s_tim, PWM_DEADTIME, PWM_DEADTIME);

  /* PWM mode 1 on all three compare units, 50% initial duty. */
  hal_tim_oc_compare_unit_config_t cu;
  cu.mode  = HAL_TIM_OC_PWM1;
  cu.pulse = PWM_MID;
  (void)HAL_TIM_OC_SetConfigCompareUnit(s_tim, HAL_TIM_OC_COMPARE_UNIT_1, &cu);
  (void)HAL_TIM_OC_SetConfigCompareUnit(s_tim, HAL_TIM_OC_COMPARE_UNIT_2, &cu);
  (void)HAL_TIM_OC_SetConfigCompareUnit(s_tim, HAL_TIM_OC_COMPARE_UNIT_3, &cu);

  /* Preload so runtime duty updates latch on the update event. */
  (void)HAL_TIM_OC_EnableComparePreload(s_tim, HAL_TIM_OC_COMPARE_UNIT_1);
  (void)HAL_TIM_OC_EnableComparePreload(s_tim, HAL_TIM_OC_COMPARE_UNIT_2);
  (void)HAL_TIM_OC_EnableComparePreload(s_tim, HAL_TIM_OC_COMPARE_UNIT_3);

  /* Enable all six outputs (complementary pairs). */
  (void)HAL_TIM_OC_StartChannel(s_tim, HAL_TIM_CHANNEL_1);
  (void)HAL_TIM_OC_StartChannel(s_tim, HAL_TIM_CHANNEL_1N);
  (void)HAL_TIM_OC_StartChannel(s_tim, HAL_TIM_CHANNEL_2);
  (void)HAL_TIM_OC_StartChannel(s_tim, HAL_TIM_CHANNEL_2N);
  (void)HAL_TIM_OC_StartChannel(s_tim, HAL_TIM_CHANNEL_3);
  (void)HAL_TIM_OC_StartChannel(s_tim, HAL_TIM_CHANNEL_3N);

  /* Main output enable (MOE). */
  (void)HAL_TIM_BREAK_EnableMainOutput(s_tim);

  /* Run. */
  (void)HAL_TIM_Start(s_tim);
}

void pwm_test_task(void)
{
  static uint32_t last_tick = 0u;

  if (s_tim == NULL)
  {
    return;
  }

  uint32_t now = HAL_GetTick();
  if ((now - last_tick) < 1u)      /* update at ~1 kHz */
  {
    return;
  }
  last_tick = now;

  /* Phase index advances ELEC_HZ * SINE_LEN steps per second. */
  uint32_t idx_a = (uint32_t)(((uint64_t)now * (uint64_t)ELEC_HZ * SINE_LEN) / 1000u) % SINE_LEN;
  uint32_t idx_b = (idx_a + (SINE_LEN / 3u)) % SINE_LEN;
  uint32_t idx_c = (idx_a + (2u * SINE_LEN / 3u)) % SINE_LEN;

  (void)HAL_TIM_OC_SetCompareUnitPulse(s_tim, HAL_TIM_OC_COMPARE_UNIT_1, duty_to_pulse(s_sin[idx_a]));
  (void)HAL_TIM_OC_SetCompareUnitPulse(s_tim, HAL_TIM_OC_COMPARE_UNIT_2, duty_to_pulse(s_sin[idx_b]));
  (void)HAL_TIM_OC_SetCompareUnitPulse(s_tim, HAL_TIM_OC_COMPARE_UNIT_3, duty_to_pulse(s_sin[idx_c]));
}
