#ifndef CROWER_CYCLE_CLOCK_H
#define CROWER_CYCLE_CLOCK_H
#include "stm32_hal.h"

/* Reserve TIM2 as a free-running 32-bit application clock. Cube configuration:
 * APB1 /1, TIM2 input 144 MHz, PSC=0. Debuggers may clear DEMCR.TRCENA on
 * disconnect; no application timeout or WS2812 wait may depend on DWT. */
#define PH_CLOCK_HZ 144000000u

static inline void ph_clock_init(void)
{
  HAL_RCC_TIM2_EnableClock();
  if (!(TIM2->CR1 & TIM_CR1_CEN)) {
    TIM2->CR1 = 0u;
    TIM2->PSC = 0u;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CNT = 0u;
    TIM2->CR1 = TIM_CR1_CEN;
  }
}

static inline uint32_t ph_clock_now(void) { return TIM2->CNT; }
#endif
