/**
  * @file    current_loop_test.h
  * @brief   40 kHz cycle-by-cycle current-loop compute test.
  */
#ifndef CURRENT_LOOP_TEST_H
#define CURRENT_LOOP_TEST_H

#include <stdint.h>

/* Configure the TIM1 update interrupt (40 kHz) and start it. */
void current_loop_test_init(void);

/* The representative FOC current-loop body (runs in the TIM1 update ISR). */
void current_loop_test_run(void);

/* Statistics (readable over SWD). */
extern volatile uint32_t g_cl_count;       /* number of loop iterations        */
extern volatile uint32_t g_cl_min_cycles;  /* fastest iteration (cycles)       */
extern volatile uint32_t g_cl_max_cycles;  /* slowest iteration (cycles)       */
extern volatile uint32_t g_cl_last_cycles; /* last iteration (cycles)          */
extern volatile uint32_t g_cl_sum_cycles;  /* sum (for average)                */
extern volatile uint32_t g_cl_overruns;    /* iterations exceeding the budget  */

#endif /* CURRENT_LOOP_TEST_H */
