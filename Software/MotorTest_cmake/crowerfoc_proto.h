/**
  * @file    crowerfoc_proto.h
  * @brief   CrowerFOC host<->device binary protocol (v1). Single source of truth.
  *
  *  Transport-agnostic; carried over USB CDC, UART and CAN identically.
  *
  *  Frame (both directions), little-endian:
  *    +--------+--------+------+-----+------------------+------+
  *    | 0xAA   | 0x55   | type | len | payload[len]     | crc8 |
  *    +--------+--------+------+-----+------------------+------+
  *    crc8 = CRC-8 (poly 0x07, init 0xFF) over [type, len, payload...]
  *
  *  Device -> host : TELEMETRY(stream), ACK, PARAM (get reply), LOG, INFO
  *  Host -> device : SET, GET, STATE, SETPOINT, SAVE, CLEAR, REBOOT
  */
#ifndef CROWERFOC_PROTO_H
#define CROWERFOC_PROTO_H

#include <stdint.h>

#define CFF_SYNC0        0xAAu
#define CFF_SYNC1        0x55u
#define CFF_MAX_PAYLOAD  64u
#define CFF_HDR_LEN      2u

/* ------------------------------------------------------------------ types */
/* device -> host */
#define CFF_MSG_TELEMETRY  0x01u
#define CFF_MSG_ACK        0x02u
#define CFF_MSG_PARAM      0x03u   /* payload: u16 id, i32/f32 value */
#define CFF_MSG_LOG        0x04u
#define CFF_MSG_INFO       0x05u   /* payload: ascii version string      */
#define CFF_MSG_CURRENT_RAW 0x06u  /* u16 seq, 3*u16 ADC, 3*f32 amperes, u16 flags */
#define CFF_CURRENT_RAW_LEN 22u    /* flags bits 0..2: median delta > 2 A */
#define CFF_MSG_CURRENT_TRACE 0x07u /* GET 0x0700..073F: u16 index, 28-byte record */
#define CFF_MSG_CONTROL_DIAG 0x08u /* u16 seq + 12*f32: targets/voltages/angles/age/KCL */
#define CFF_MSG_AXIS_DIAG 0x0Au /* u16 seq, 15*f32 incl. measured motor/output ratio */

/* host -> device */
#define CFF_MSG_SET        0x81u   /* payload: u16 id, <value>           */
#define CFF_MSG_GET        0x82u   /* payload: u16 id                    */
#define CFF_MSG_STATE      0x83u   /* payload: u8 requested_state        */
#define CFF_MSG_SETPOINT   0x84u   /* payload: f32 input                 */
#define CFF_MSG_SAVE       0x85u   /* save config to flash               */
#define CFF_MSG_CLEAR      0x86u   /* clear errors                       */
#define CFF_MSG_REBOOT     0x87u
#define CFF_MSG_HEARTBEAT  0x88u   /* empty; required at least every 750ms while driving */
#define CFF_MSG_CONFIG_PATCH 0x89u /* op0 begin, op1 + (u16,f32)*N stage, op2 apply, op3 abort; idle only */

/* ------------------------------------------------------------------ enums */
enum {
  CFF_AXIS_UNDEFINED = 0,
  CFF_AXIS_IDLE,
  CFF_AXIS_CALIBRATION,
  CFF_AXIS_CLOSED_LOOP,
  CFF_AXIS_ERROR,
  CFF_AXIS_MEASURE      /* align then measure R / L (dc step on d-axis) */
};

enum {               /* control mode  */
  CFF_CTRL_TORQUE = 0,
  CFF_CTRL_VELOCITY,
  CFF_CTRL_POSITION
};

enum {               /* input mode    */
  CFF_INPUT_INACTIVE = 0,
  CFF_INPUT_PASSTHROUGH,
  CFF_INPUT_VEL_RAMP,
  CFF_INPUT_POS_FILTER,
  CFF_INPUT_TRAP_TRAJ
};

/* telemetry.flags */
#define CFF_FLAG_CLOSED_LOOP   (1u << 0)
#define CFF_FLAG_ENCODER_OK    (1u << 1)
#define CFF_FLAG_CALIBRATED    (1u << 2)
#define CFF_FLAG_BRAKING       (1u << 3)

/* fault bits (telemetry.fault) */
#define CFF_FAULT_NONE         0x0000u
#define CFF_FAULT_OVERVOLTAGE  (1u << 0)
#define CFF_FAULT_UNDERVOLTAGE (1u << 1)
#define CFF_FAULT_OVERCURRENT  (1u << 2)
#define CFF_FAULT_FET_TEMP     (1u << 3)
#define CFF_FAULT_MOTOR_TEMP   (1u << 4)
#define CFF_FAULT_ENCODER      (1u << 5)
#define CFF_FAULT_SPINOUT      (1u << 6)
#define CFF_FAULT_DRV          (1u << 7)
#define CFF_FAULT_ADC          (1u << 8) /* conversion failure or invalid sampling */
#define CFF_FAULT_COMM         (1u << 9) /* host heartbeat expired */
#define CFF_FAULT_CALIBRATION  (1u << 10) /* electrical identification/tracking failed */
#define CFF_FAULT_OVERSPEED    (1u << 11) /* reserved: legacy firmware compatibility */
#define CFF_FAULT_POSITION     (1u << 12)
#define CFF_FAULT_DEADLINE     (1u << 13)
#define CFF_FAULT_ESTOP        (1u << 14)
#define CFF_FAULT_CONFIG       (1u << 15)

/* --------------------------------------------------- telemetry payload (36 B) */
#define CFF_TELEM_LEN   36u
/* offsets within the telemetry payload */
#define CFF_T_SEQ        0u    /* u16  frame sequence                       */
#define CFF_T_ANGLE_RAW  2u    /* u16  encoder raw 0..32767                 */
#define CFF_T_ANGLE_X100 4u    /* i16  mechanical angle deg*100 0..35999    */
#define CFF_T_ELEC_X100  6u    /* i16  electrical angle deg*100 0..35999    */
#define CFF_T_IA         8u    /* i16  phase A current, mA                  */
#define CFF_T_IB         10u   /* i16  phase B current, mA                  */
#define CFF_T_IC         12u   /* i16  phase C current, mA                  */
#define CFF_T_ID         14u   /* i16  d-axis current, mA                   */
#define CFF_T_IQ         16u   /* i16  q-axis current, mA                   */
#define CFF_T_VEL_X100   18u   /* i16  velocity, rad/s*100 (or turns/s*100) */
#define CFF_T_POS_X100   20u   /* i16 legacy saturated position rad*100; float AXIS_DIAG authoritative */
#define CFF_T_VBUS_MV    22u   /* u16  DC bus, mV                           */
#define CFF_T_DUTY_A     24u   /* u16  PWM compare A                        */
#define CFF_T_DUTY_B     26u   /* u16  PWM compare B                        */
#define CFF_T_DUTY_C     28u   /* u16  PWM compare C                        */
#define CFF_T_FAULT      30u   /* u16  fault bitfield                       */
#define CFF_T_AXIS_STATE 32u   /* u8                                        */
#define CFF_T_AXIS_MODE  33u   /* u8   control mode                         */
#define CFF_T_INPUT_MODE 34u   /* u8   input mode                           */
#define CFF_T_FLAGS      35u   /* u8                                        */

/* --------------------------------------------------- parameter ids */
#define CFF_PARAM_MOTOR_POLE_PAIRS              0x0100u
#define CFF_PARAM_MOTOR_PHASE_RESISTANCE        0x0101u
#define CFF_PARAM_MOTOR_PHASE_INDUCTANCE        0x0102u
#define CFF_PARAM_MOTOR_CURRENT_LIMIT           0x0103u
#define CFF_PARAM_MOTOR_CALIB_CURRENT           0x0104u /* read-only fixed commissioning profile */
#define CFF_PARAM_ENCODER_OFFSET                0x0200u
#define CFF_PARAM_ENCODER_DIR                   0x0201u
#define CFF_PARAM_ENCODER_BANDWIDTH             0x0202u
#define CFF_PARAM_CONTROLLER_POS_GAIN           0x0300u
#define CFF_PARAM_CONTROLLER_VEL_GAIN           0x0301u
#define CFF_PARAM_CONTROLLER_VEL_INTEGRATOR_GAIN 0x0302u
#define CFF_PARAM_CONTROLLER_CUR_GAIN_P         0x0303u
#define CFF_PARAM_CONTROLLER_CUR_GAIN_I         0x0304u
#define CFF_PARAM_LIMIT_VEL                     0x0400u
#define CFF_PARAM_LIMIT_CURRENT                 0x0401u
#define CFF_PARAM_AXIS_CONTROL_MODE             0x0500u
#define CFF_PARAM_AXIS_INPUT_MODE               0x0501u
#define CFF_PARAM_AXIS_STATE                    0x0502u
#define CFF_PARAM_AXIS_INPUT_POS                0x0503u
#define CFF_PARAM_AXIS_INPUT_VEL                0x0504u
#define CFF_PARAM_AXIS_INPUT_TORQUE             0x0505u

/* value kind, for host-side decoding of GET replies */
enum {
  CFF_VAL_U8 = 0, CFF_VAL_I16, CFF_VAL_U16, CFF_VAL_I32, CFF_VAL_F32
};

#endif /* CROWERFOC_PROTO_H */
