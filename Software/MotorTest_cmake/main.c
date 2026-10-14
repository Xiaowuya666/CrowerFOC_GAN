/**
  ******************************************************************************
  * file           : main.c
  * brief          : Main program body
  ******************************************************************************
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ws2812.h"
#include "mx_usbx_app.h"
#include "mx_usb_drd_fs.h"
#include "usb_cdc_test.h"
#include "pwm_test.h"
#include "phase_test.h"
#include "crowerfoc_proto.h"

/* Private define ------------------------------------------------------------*/
#define RAINBOW_MAX_BRIGHTNESS   40u   /* 0..255, smaller = dimmer */

/* Private functions prototype -----------------------------------------------*/
/* USBX STM32 device controller init (provided by middleware/usbx DCD) */
extern UINT _ux_dcd_stm32_initialize(ULONG dcd_io, ULONG parameter);

/**
  * brief:  The application entry point.
  * retval: none but we specify int to comply with C99 standard
  */
int main(void)
{
  if (mx_system_init() != SYSTEM_OK)
  {
    return (-1);
  }

  /* Force a real USB soft detach before reusing EP0 after debug/software reset.
     The existing 400ms boot indication keeps D+ detached long enough for the
     host to discard the old device address; USBX reconnects it below. */
  /* HAL_PCD_Stop also powers down the PHY; Start does not reinitialize it.
     Only detach D+ here so the initialized controller stays ready. */
  hal_pcd_handle_t *usb_boot = mx_usb_drd_fs_device_gethandle();
  (void)usb_boot->driver.device_disconnect((uint32_t)usb_boot->instance);

  /* WS2812 data line is on PA3 (RGB1 DI), chain of 3 LEDs */
  ws2812_init();

  /* 3-phase open-loop SPWM current test + USB streaming (TIM1 + ADC1 + CDC) */
  phase_test_init();

  /* Boot indicator: brief white flash so a reset is visually confirmed */
  ws2812_set_all(30u, 30u, 30u);
  ws2812_show();
  HAL_Delay(400);
  ws2812_clear();
  ws2812_show();

  /* USBX device stack (CDC ACM virtual COM) */
  if (app_usbx_init() != UX_SUCCESS)
  {
    /* error: steady red */
    ws2812_set_all(40u, 0u, 0u);
    ws2812_show();
    while (1) { }
  }

  /* Bind the STM32 PCD to the USBX device controller and enable the port */
  (void)_ux_dcd_stm32_initialize(0U, (ULONG)mx_usb_drd_fs_device_gethandle());

  uint16_t hue  = 0;
  uint16_t step = 0;
  uint32_t last_led = 0;

  while (1)
  {
    /* Pump the USBX device stack continuously. In standalone (no OS) mode
       the enumeration and control transfers are serviced from here, so this
       must not be throttled by the LED animation delay. */
    (void)app_usbx_process();

    /* 3-phase current test: stream frames over USB CDC. */
    phase_test_task();

    /* Animate the LEDs on a ~20 ms cadence without stalling USB tasks.
       ws2812_show() blocks interrupts for ~90 us, which would jolt the 40 kHz
       FOC ISR -> hold the refresh while the axis is actively driven. */
    uint32_t now = HAL_GetTick();
    if ((now - last_led) < 20u)
    {
      continue;
    }
    last_led = now;
    if (g_ph_state != CFF_AXIS_IDLE)
    {
      continue;
    }

    /* Breathing brightness: 0 -> 254 -> 0, scaled down */
    uint8_t bright_raw;
    uint8_t bright;
    if (step & 0x80u)
    {
      bright_raw = (uint8_t)(255u - (uint8_t)(step & 0x7Fu) * 2u);
    }
    else
    {
      bright_raw = (uint8_t)((uint8_t)(step & 0x7Fu) * 2u);
    }
    bright = (uint8_t)(((uint16_t)bright_raw * RAINBOW_MAX_BRIGHTNESS) / 255u);

    /* Rainbow: each LED gets an offset hue, the whole pattern rotates */
    for (uint16_t i = 0; i < WS2812_LED_COUNT; i++)
    {
      uint16_t led_hue = (uint16_t)((hue + i * 40u) % 360u);
      ws2812_set_hsv(i, led_hue, 255u, bright);
    }
    ws2812_show();

    hue  = (uint16_t)((hue + 5u) % 360u);
    step = (uint16_t)((step + 2u) & 0x1FFu);
  }
} /* end main */
