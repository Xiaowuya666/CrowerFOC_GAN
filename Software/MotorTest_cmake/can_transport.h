#ifndef CAN_TRANSPORT_H
#define CAN_TRANSPORT_H
#include <stdint.h>
typedef void (*cff_can_receive_t)(uint16_t id,const uint8_t *data,uint8_t length,uint8_t remote);
typedef void (*cff_can_fault_t)(void);
int cff_can_init(unsigned bitrate_kbps);
void cff_can_task(cff_can_receive_t receive,cff_can_fault_t fault);
int cff_can_send(uint16_t id,const uint8_t *data,uint8_t length);
extern uint32_t g_can_rx,g_can_tx,g_can_dropped,g_can_restarts;
#endif
