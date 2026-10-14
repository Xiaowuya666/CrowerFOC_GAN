/**
  ******************************************************************************
  * @file    ws2812.c
  * @brief   Bit-banged WS2812 driver using the independent TIM2 clock.
  *
  *   Data pin : PA3 (per netlist, U1-13 -> RGB1 DI)
  *   Chain    : RGB1 -> RGB2 -> RGB3
  *   Protocol : 800 kHz, GRB order, MSB first, data latched after >50 us low.
  ******************************************************************************
  */
#include "ws2812.h"
#include "stm32_hal.h"
#include "cycle_clock.h"

/* ------------------------------------------------------------------------- */
/* Hardware mapping (from netlist)                                           */
/* ------------------------------------------------------------------------- */
#define WS2812_GPIO           GPIOA
#define WS2812_GPIO_PIN       (1UL << 3U)
#define WS2812_HAL_PORT       HAL_GPIOA
#define WS2812_HAL_PIN        HAL_GPIO_PIN_3

/* ------------------------------------------------------------------------- */
/* Timing: core runs at 144 MHz -> 1 cycle = 6.944 ns                        */
/*   T0H = 0.35 us = 50 c,  T0L = 0.90 us                                  */
/*   T1H = 0.80 us = 115 c, T1L = 0.45 us                                  */
/*   bit period = 1.25 us = 180 c                                          */
/* ------------------------------------------------------------------------- */
#define WS2812_CPU_HZ         144000000u
#define WS2812_T0H_CYCLES     50u
#define WS2812_T1H_CYCLES     115u
#define WS2812_PERIOD_CYCLES  180u
#define WS2812_RESET_US       300u

static uint8_t ws2812_buf[WS2812_LED_COUNT * 3u];

/* ------------------------------------------------------------------------- */
static void ws2812_delay_us(uint32_t us)
{
  uint32_t start = ph_clock_now();
  uint32_t ticks = us * (WS2812_CPU_HZ / 1000000u);

  while ((ph_clock_now() - start) < ticks)
  {
  }
}

static void ws2812_send_byte(uint8_t value)
{
  for (int i = 7; i >= 0; i--)
  {
    uint32_t t_bit = ph_clock_now();

    WS2812_GPIO->BSRR = WS2812_GPIO_PIN;

    if (value & (1u << i))
    {
      while ((ph_clock_now() - t_bit) < WS2812_T1H_CYCLES)
      {
      }
    }
    else
    {
      while ((ph_clock_now() - t_bit) < WS2812_T0H_CYCLES)
      {
      }
    }

    WS2812_GPIO->BSRR = (uint32_t)WS2812_GPIO_PIN << 16;

    t_bit += WS2812_PERIOD_CYCLES;
    while ((int32_t)(ph_clock_now() - t_bit) < 0)
    {
    }
  }
}

/* ------------------------------------------------------------------------- */
void ws2812_init(void)
{
  hal_gpio_config_t cfg;

  cfg.mode        = HAL_GPIO_MODE_OUTPUT;
  cfg.speed       = HAL_GPIO_SPEED_FREQ_VERY_HIGH;
  cfg.pull        = HAL_GPIO_PULL_NO;
  cfg.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
  cfg.init_state  = HAL_GPIO_PIN_RESET;

  HAL_GPIO_Init(WS2812_HAL_PORT, WS2812_HAL_PIN, &cfg);

  ph_clock_init();

  ws2812_clear();
  ws2812_show();
}

void ws2812_set(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
  if (index >= WS2812_LED_COUNT)
  {
    return;
  }

  /* WS2812 uses GRB byte order */
  ws2812_buf[index * 3u + 0u] = g;
  ws2812_buf[index * 3u + 1u] = r;
  ws2812_buf[index * 3u + 2u] = b;
}

void ws2812_set_all(uint8_t r, uint8_t g, uint8_t b)
{
  for (uint16_t i = 0u; i < WS2812_LED_COUNT; i++)
  {
    ws2812_set(i, r, g, b);
  }
}

void ws2812_clear(void)
{
  for (uint16_t i = 0u; i < (WS2812_LED_COUNT * 3u); i++)
  {
    ws2812_buf[i] = 0u;
  }
}

void ws2812_show(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();

  for (uint16_t i = 0u; i < (WS2812_LED_COUNT * 3u); i++)
  {
    ws2812_send_byte(ws2812_buf[i]);
  }

  __set_PRIMASK(primask);

  ws2812_delay_us(WS2812_RESET_US);
}

void ws2812_set_hsv(uint16_t index, uint16_t h, uint8_t s, uint8_t v)
{
  uint8_t region;
  uint8_t remainder;
  uint8_t p;
  uint8_t q;
  uint8_t t;
  uint8_t r;
  uint8_t g;
  uint8_t b;

  h = h % 360u;
  region = (uint8_t)(h / 60u);
  remainder = (uint8_t)(((h % 60u) * 255u) / 60u);

  p = (uint8_t)(((uint32_t)v * (255u - s)) / 255u);
  q = (uint8_t)(((uint32_t)v * (255u - ((uint32_t)s * remainder) / 255u)) / 255u);
  t = (uint8_t)(((uint32_t)v * (255u - ((uint32_t)s * (255u - remainder)) / 255u)) / 255u);

  switch (region)
  {
    case 0u:  r = v; g = t; b = p; break;
    case 1u:  r = q; g = v; b = p; break;
    case 2u:  r = p; g = v; b = t; break;
    case 3u:  r = p; g = q; b = v; break;
    case 4u:  r = t; g = p; b = v; break;
    default:  r = v; g = p; b = q; break;
  }

  ws2812_set(index, r, g, b);
}
