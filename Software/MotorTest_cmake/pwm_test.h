/**
  * @file    pwm_test.h
  * @brief   3-phase complementary PWM output test (TIM1 CH1/2/3 + CH1N/2N/3N).
  */
#ifndef PWM_TEST_H
#define PWM_TEST_H

#include <stdint.h>

/* Configure TIM1 for center-aligned 3-phase PWM and start the outputs. */
void pwm_test_init(void);

/* Update the three duties (call from the main loop). */
void pwm_test_task(void);

#endif /* PWM_TEST_H */
