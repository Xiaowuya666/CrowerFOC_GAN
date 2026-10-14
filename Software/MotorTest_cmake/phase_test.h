/**
  * @file    phase_test.h
  * @brief   3-phase open-loop SPWM current test with USB streaming.
  */
#ifndef PHASE_TEST_H
#define PHASE_TEST_H

#include <stdint.h>

/* Bring up TIM1 3-phase PWM + ADC1 (IN4/5/6) + CDC streaming. */
void phase_test_init(void);

/* Drain the USB TX ring buffer (call from the main loop). */
void phase_test_task(void);

/* CDC-ACM activate/deactivate hooks (called from the USBX class callbacks). */
void phase_test_on_activate(void *cdc_instance);
void phase_test_on_deactivate(void);

/* Axis state (see CFF_AXIS_* in crowerfoc_proto.h): non-zero means the motor is
   being driven, so the (interrupt-blocking) WS2812 refresh must be held off. */
extern volatile uint8_t g_ph_state;

/* Host-tunable parameters. */
extern volatile uint16_t g_ph_mod_milli;   /* modulation index x 1000      */
extern volatile uint16_t g_ph_freq_deci;   /* electrical frequency x 10    */
extern volatile uint8_t  g_ph_streaming;   /* 0/1                          */
extern volatile uint32_t g_ph_frames;      /* frames produced              */
extern volatile uint32_t g_ph_dropped;     /* frames dropped (ring full)   */

#endif /* PHASE_TEST_H */
