#include "can_transport.h"
#include "mx_fdcan1.h"
#include "stm32_hal.h"
#include "stm32c5xx_hal_fdcan.h"
uint32_t g_can_rx,g_can_tx,g_can_dropped,g_can_restarts;
static hal_fdcan_handle_t *bus;
static uint32_t last_restart;
static unsigned configured_bitrate;
int cff_can_init(unsigned bitrate) {
  bus=mx_fdcan1_gethandle();
  if(!bus||(bitrate!=125&&bitrate!=250&&bitrate!=500&&bitrate!=1000)) return 0;
  if(configured_bitrate) (void)HAL_FDCAN_Stop(bus);
  /* 144 MHz / prescaler / 144 tq. 87.5% sample point. */
  hal_fdcan_nominal_bit_timing_t timing={0};
  timing.nominal_prescaler=1000u/bitrate;
  timing.nominal_time_seg1=125; timing.nominal_time_seg2=18; timing.nominal_jump_width=4;
  if(HAL_FDCAN_SetNominalBitTiming(bus,&timing)!=HAL_OK) return 0;
  configured_bitrate=bitrate;
  return HAL_FDCAN_Start(bus)==HAL_OK;
}
int cff_can_send(uint16_t id,const uint8_t *data,uint8_t length) {
  if(!bus||length>8||!HAL_FDCAN_GetTxFifoFreeLevel(bus)) { g_can_dropped++;return 0; }
  hal_fdcan_tx_header_t tx={0};
  /* This HAL accepts a logical 11-bit ID and shifts it into bits [28:18]
     internally when writing the message RAM. Pre-shifting here makes the
     HAL shift it a second time and corrupts the transmitted identifier. */
  tx.b.identifier=(uint32_t)id; tx.b.identifier_type=HAL_FDCAN_ID_STANDARD;
  tx.b.frame_type=HAL_FDCAN_FRAME_DATA; tx.b.data_length=(hal_fdcan_data_length_code_t)length;
  tx.b.frame_format=HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
  if(HAL_FDCAN_ReqTransmitMsgFromFIFOQ(bus,&tx,data)!=HAL_OK) {g_can_dropped++;return 0;}
  g_can_tx++;return 1;
}
void cff_can_task(cff_can_receive_t receive,cff_can_fault_t fault) {
  if(!bus) return;
  hal_fdcan_protocol_status_t status;
  if(HAL_FDCAN_GetProtocolStatus(bus,&status)==HAL_OK&&status.bus_off) {
    fault();
    uint32_t now=HAL_GetTick();
    if((uint32_t)(now-last_restart)>=1000u) {
      last_restart=now;g_can_restarts++;(void)cff_can_init(configured_bitrate);
    }
    return;
  }
  /* Bound work so unrelated bus traffic cannot starve encoder acquisition. */
  for(unsigned i=0;i<3;i++) {
    uint32_t count=0; HAL_FDCAN_GetRxFifoFillLevel(bus,HAL_FDCAN_RX_FIFO_0,&count);
    if(!count) break;
    hal_fdcan_rx_header_t rx={0};uint8_t data[64]={0};
    if(HAL_FDCAN_GetReceivedMessage(bus,HAL_FDCAN_RX_FIFO_0,&rx,data)!=HAL_OK) {g_can_dropped++;break;}
    if(rx.b.identifier_type==HAL_FDCAN_ID_STANDARD&&rx.b.frame_format==HAL_FDCAN_HEADER_FRAME_FORMAT_CAN&&rx.b.data_length<=8) {
      g_can_rx++; receive((uint16_t)(rx.b.identifier>>18),data,(uint8_t)rx.b.data_length,(uint8_t)rx.b.frame_type);
    }
  }
}
