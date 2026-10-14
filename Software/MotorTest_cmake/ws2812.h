/**
  ******************************************************************************
  * @file    ws2812.h
  * @brief   WS2812 / WS2812B (XL-3528RGBW-2812B) driver for STM32C562
  *
  * Hardware (from schematic netlist):
  *   WS2812 data  : PA3 (U1-13)  -> RGB1 DI
  *   LED chain    : RGB1 DO -> RGB2 DI -> RGB3 DI
  *   LED count    : 3
  ******************************************************************************
  */
#ifndef WS2812_H
#define WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define WS2812_LED_COUNT   3u

void ws2812_init(void);
void ws2812_set(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void ws2812_set_all(uint8_t r, uint8_t g, uint8_t b);
void ws2812_set_hsv(uint16_t index, uint16_t h, uint8_t s, uint8_t v);
void ws2812_clear(void);
void ws2812_show(void);

#ifdef __cplusplus
}
#endif

#endif /* WS2812_H */
