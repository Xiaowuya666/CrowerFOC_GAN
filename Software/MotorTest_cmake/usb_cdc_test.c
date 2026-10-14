/**
  * @file    usb_cdc_test.c
  * @brief   CDC ACM throughput (TX) + interrupt-driven receive (RX) test.
  *
  *  - TX: as soon as the CDC-ACM class is activated, the device continuously
  *        streams a 0..255 ramp over the bulk IN endpoint as fast as the
  *        standalone USBX state machine allows.
  *  - RX: an OUT read is armed on the bulk OUT endpoint. The transfer is
  *        completed from the USB interrupt (via the DCD / device stack); when
  *        data arrives it is echoed back with a "ECHO" marker and counted.
  *
  *  The state machines are driven from the main loop; the actual USB traffic
  *  is interrupt driven.
  */
#include "usb_cdc_test.h"
#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"
#include <string.h>

#define TX_CHUNK        512u
#define RX_CHUNK        512u
#define ECHO_MARKER_LEN 4u

static UX_SLAVE_CLASS_CDC_ACM *s_cdc = UX_NULL;

volatile uint32_t g_cdc_tx_bytes  = 0u;
volatile uint32_t g_cdc_rx_bytes  = 0u;
volatile uint32_t g_cdc_rx_events = 0u;
volatile uint32_t g_cdc_tx_errors = 0u;

static uint8_t   s_stream[TX_CHUNK];
static uint8_t   s_rx[RX_CHUNK];
static uint8_t   s_echo[ECHO_MARKER_LEN + RX_CHUNK];

static uint8_t  *s_tx_ptr = s_stream;
static uint32_t  s_tx_len = TX_CHUNK;
static uint8_t   s_tx_busy = 0u;
static uint8_t   s_tx_is_echo = 0u;
static uint8_t   s_echo_pending = 0u;
static uint32_t  s_echo_len = 0u;

void usb_cdc_test_on_activate(void *cdc_acm_instance)
{
  s_cdc = (UX_SLAVE_CLASS_CDC_ACM *)cdc_acm_instance;
  for (uint32_t i = 0u; i < (uint32_t)sizeof(s_stream); i++)
  {
    s_stream[i] = (uint8_t)i;
  }
  s_tx_busy = 0u;
  s_tx_is_echo = 0u;
  s_echo_pending = 0u;
  s_tx_ptr = s_stream;
  s_tx_len = (uint32_t)sizeof(s_stream);
}

void usb_cdc_test_on_deactivate(void)
{
  s_cdc = UX_NULL;
}

void usb_cdc_test_task(void)
{
  ULONG actual = 0u;
  UINT  st;

  /* Snapshot: the USB ISR may run usb_cdc_test_on_deactivate() at any time and
     clear s_cdc, which would otherwise leave us calling the class with NULL. */
  UX_SLAVE_CLASS_CDC_ACM *cdc = s_cdc;

  if (cdc == UX_NULL)
  {
    return;
  }

  /* ---- RX: arm an OUT read (completes asynchronously from the USB ISR). ---- */
  st = ux_device_class_cdc_acm_read_run(cdc, s_rx, (ULONG)sizeof(s_rx), &actual);
  if (st == UX_STATE_NEXT)
  {
    g_cdc_rx_events++;
    g_cdc_rx_bytes += actual;

    if (actual > (ULONG)sizeof(s_rx))
    {
      actual = (ULONG)sizeof(s_rx);
    }
    s_echo[0] = 'E';
    s_echo[1] = 'C';
    s_echo[2] = 'H';
    s_echo[3] = 'O';
    (void)memcpy(&s_echo[ECHO_MARKER_LEN], s_rx, (size_t)actual);
    s_echo_len = ECHO_MARKER_LEN + (uint32_t)actual;
    s_echo_pending = 1u;
  }

  /* ---- TX: echo has priority, otherwise stream the ramp. ---- */
  if (s_tx_busy == 0u)
  {
    if (s_echo_pending != 0u)
    {
      s_echo_pending = 0u;
      s_tx_is_echo = 1u;
      s_tx_ptr = s_echo;
      s_tx_len = s_echo_len;
    }
    else
    {
      s_tx_is_echo = 0u;
      s_tx_ptr = s_stream;
      s_tx_len = (uint32_t)sizeof(s_stream);
    }
    s_tx_busy = 1u;
  }

  st = ux_device_class_cdc_acm_write_run(cdc, s_tx_ptr, s_tx_len, &actual);
  if (st == UX_STATE_NEXT)
  {
    if (s_tx_is_echo == 0u)
    {
      g_cdc_tx_bytes += actual;
    }
    s_tx_busy = 0u;
  }
  else if ((st == UX_STATE_ERROR) || (st == UX_STATE_EXIT))
  {
    g_cdc_tx_errors++;
    s_tx_busy = 0u;
  }
}
