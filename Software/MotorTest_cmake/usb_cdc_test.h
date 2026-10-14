/**
  * @file    usb_cdc_test.h
  * @brief   CDC ACM throughput (TX) and interrupt-driven receive (RX) test.
  */
#ifndef USB_CDC_TEST_H
#define USB_CDC_TEST_H

#include <stdint.h>

/* Called from the USBX CDC-ACM class activate/deactivate callbacks. */
void usb_cdc_test_on_activate(void *cdc_acm_instance);
void usb_cdc_test_on_deactivate(void);

/* Call continuously from the main loop (standalone USBX). */
void usb_cdc_test_task(void);

/* Observability (readable over SWD). */
extern volatile uint32_t g_cdc_tx_bytes;     /* total bytes streamed out */
extern volatile uint32_t g_cdc_rx_bytes;     /* total bytes received from host */
extern volatile uint32_t g_cdc_rx_events;    /* number of completed RX transfers */
extern volatile uint32_t g_cdc_tx_errors;    /* TX state-machine errors */

#endif /* USB_CDC_TEST_H */
