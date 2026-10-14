/**
  * @file    phase_test.c
  * @brief   25 kHz cycle-by-cycle loop: ADC+DMA current sampling + open-loop V/f.
  *
  *  - TIM1 center-aligned 40 kHz, 3 complementary pairs + deadtime, RCR=1.
  *  - Every PWM period (TIM1 update ISR):
  *      ADC1 regular (IN4/IN5/IN6) is converted by LPDMA1 (peripheral request)
  *      into adc_buf[3]; the ISR waits for the sequence end and reads it, then
  *      runs the open-loop V/f SPWM, the overcurrent trip, and (every 400th
  *      period) pushes a sample into a ring.
  *  - The main loop only drains the ring -> USB CDC-ACM at ~100 Hz.
  *
  *  Motor is safe on power-up: g_ph_mod_milli = 0 (no drive).
  *
  *  Host commands (A5 cmd lo hi chk): 0x01 mod x1000, 0x02 freq x10,
  *  0x03 stream on, 0x04 stream off, 0x05 reset, 0x06 clear trip.
  */
#include "phase_test.h"
#include "pwm_test.h"
#include "mx_tim1.h"
#include "mx_adc1.h"
#include "mx_adc2.h"
#include "stm32_hal.h"
#include "stm32c5xx_hal_adc.h"
#include "stm32c5xx_ll_adc.h"
#include "stm32c5xx_ll_tim.h"
#include "stm32c5xx_hal_dma.h"
#include "stm32c5xx_hal_spi.h"
#include "stm32c5xx_hal_gpio.h"
#include "mx_spi3.h"
#include "mx_gpio_default.h"
#include "crowerfoc_proto.h"
#include "current_filter.h"
#include "calibration_profile.h"
#include "cogging_compensation.h"
#include "foc_limits.h"
#include "cycle_clock.h"
#include "axis_config.h"
#include "config_store.h"
#include "motion_control.h"
#include "axis_parameters.h"
#include "mit_can_protocol.h"
#include "stm32c5xx_ll_iwdg.h"
#include "can_transport.h"
#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"
#include <math.h>
#include <string.h>

#define PH_PWM_PERIOD    2879u
#define PH_ISR_HZ        25000u
#define PH_LUT_BITS      8u
#define PH_LUT_N         (1u << PH_LUT_BITS)
#define PH_TWO_PI        6.28318530f

#define PH_VF_MILLI      30u            /* modulation(x1000) per Hz (V/f ratio) */
#define PH_RAMP_ISR      800u           /* +0.1 Hz every 800 ISRs ~= 5 Hz/s     */
#define PH_SEND_DECIM    250u           /* 25 kHz / 250 = 100 Hz frames         */

#define PH_TXRING        1024u          /* SPSC byte ring: ISR writes, main reads */

volatile uint16_t g_ph_mod_milli = 0u;      /* SAFE default: no drive */
volatile uint16_t g_ph_freq_deci = 20u;     /* 2.0 Hz                  */
volatile uint8_t  g_ph_streaming = 1u;
volatile uint32_t g_ph_frames    = 0u;
volatile uint32_t g_ph_dropped   = 0u;
volatile uint32_t g_ph_isr_last  = 0u;
volatile uint32_t g_ph_isr_max   = 0u;
volatile uint32_t g_ph_adc_wait  = 0u;
volatile uint32_t g_ph_adc_to    = 0u;
static volatile uint32_t s_adc_inj_timeout;
static volatile uint32_t s_adc_reg_timeout;
static volatile uint32_t s_adc_bad_result;
static volatile uint8_t s_adc_missed_streak;
static volatile uint8_t s_last_good_sample_mask=7u;
volatile uint8_t  g_ph_trip      = 0u;
volatile uint16_t g_ph_peak      = 0u;
volatile uint16_t g_ph_oc_count  = 0u;
volatile uint32_t g_ph_spike     = 0u;    /* count of |I| excursions > PH_SPIKE_LVL */
volatile uint16_t g_ph_spike_max = 0u;    /* largest excursion seen (ADC counts)  */
volatile uint16_t g_ph_spike_raw[3] = { 0u, 0u, 0u };
#define PH_SPIKE_LVL   200u               /* ~6.5 A (32.2 mA/count) */

/* timing confirmation: TIM1 counter captured at ISR entry / after ADC read */
volatile uint16_t g_ph_cnt_upd   = 0u;
volatile uint16_t g_ph_cnt_rd    = 0u;

/* TLE5012B SSC probe (read over SPI3, half-duplex single wire) */
volatile uint16_t g_enc_cmd      = 0x8021u;  /* Read,Lock0,UPD0,ADDR=0x02(AVAL),ND=1 */
volatile uint8_t  g_enc_mode     = 1u;       /* SPI mode 0..3 (TLE5012B = mode 1) */
volatile uint16_t g_enc_w1       = 0u;
volatile uint16_t g_enc_w2       = 0u;
volatile uint16_t g_enc_angle    = 0u;
volatile uint16_t g_ph_if_hz     = 0u;   /* open-loop FOC elec freq x10 (0 = use encoder) */
volatile uint8_t  g_enc_status   = 0u;
volatile uint32_t g_enc_count    = 0u;
volatile uint32_t g_enc_fail     = 0u;   /* failed (unrecoverable) encoder reads */

/* ---- axis configuration (RAM now; flash-backed later) -------------------- */
static const cff_config_t s_defaults = {
  .pole_pairs = 0u, /* unknown until measured or explicitly configured */
  .phase_resistance = 0.0f,
  .phase_inductance = 0.0f,
  .current_limit = 40.0f,
  .calibration_current = PH_CAL_CURRENT_A,
  .encoder_offset = 0.0f,
  .encoder_dir = -1,
  .encoder_bandwidth = 300.0f, /* rad/s, sampled SSC angle observer */
  .pos_gain = 20.0f,
  .vel_gain = 0.05f,
  .vel_integrator_gain = 0.1f,
  .current_gain_p = 0.05f,
  .current_gain_i = 100.0f,
  .limit_vel = CFF_MOTOR_MAX_RAD_S,
  .overspeed_limit = CFF_AXIS_TRIP_RAD_S,
  .control_mode = CFF_CTRL_TORQUE,
  .input_mode = CFF_INPUT_PASSTHROUGH,
  .input_pos = 0.0f, .input_vel = 0.0f, .input_torque = 0.0f,
  .torque_constant=0, .current_bandwidth=CFF_CURRENT_BANDWIDTH_DEFAULT_RAD_S, .vel_ramp=94.24778f, .torque_ramp=5,
  .input_filter_bandwidth=2, .traj_vel=CFF_AXIS_MAX_RAD_S, .traj_accel=28.27433f, .traj_decel=28.27433f, .inertia=0,
  .pos_min=-PH_TWO_PI, .pos_max=PH_TWO_PI, .pos_zero=0, .circular_range=PH_TWO_PI,
  .bus_uv=14, .bus_uv_recover=16, .bus_ov=32,
  .fet_temp_lower=70, .fet_temp_upper=90, .motor_temp_lower=80, .motor_temp_upper=100,
  .motor_ntc_beta=3450, .motor_ntc_r25=10000, .regen_current=0, .input_power=0,
  /* Keep the verified board NTC active. TEMP2's motor sensor curve is unknown,
     so leave its temperature conversion/trip disabled until configured. */
  .fet_temp_enable=1u, .motor_temp_enable=0u,
  .can_node=1, .can_bitrate_kbps=1000, .can_heartbeat_ms=100, .can_encoder_ms=1,
  .torque_vel_limit=1,
};
static cff_config_t s_cfg;
static cff_config_t s_staged_cfg;
static uint8_t s_config_pending;
static uint32_t s_staged_start;

volatile uint8_t  g_ph_state      = CFF_AXIS_IDLE;
volatile uint16_t g_ph_fault      = 0u;
volatile uint16_t g_ph_vbus       = 0u;
volatile uint8_t  g_ph_enc_ok     = 0u;

/* ---- FOC (current loop) ------------------------------------------------ */
#define PH_A_PER_LSB   0.03223f     /* 0.5 mOhm * gain 50 = 0.025 V/A -> A per ADC LSB */
#define PH_SQRT3_2     0.86602540f
#define PH_INV_SQRT3   0.57735027f
#define PH_PI_2        1.57079633f
#define PH_PI          3.14159265f
#define PH_FCPU        144000000.0f
#define PH_CAL_ISR     PH_ISR_HZ    /* calibration timing follows PWM rate */

static float   s_sin_f[256];
static volatile float s_mech = 0.0f;       /* mechanical angle, rad (from encoder) */
static float   s_if_ang = 0.0f;     /* open-loop i/f electrical angle */
static float   s_vbus = 21.2f;
static float   s_id_int = 0.0f, s_iq_int = 0.0f;
static float   s_iq_ref = 0.0f;     /* current-loop q target, A */
static float   s_i_off[3] = { 2048.0f, 2048.0f, 2048.0f };  /* phase current zero (ADC) */
static ph_current_filter_t s_current_filter;
static float s_current_raw[3];
static uint16_t s_sample_flags;
static uint32_t s_impulses = 0u;
static uint32_t s_active_adc_max = 0u, s_active_isr_max = 0u;
static uint16_t s_active_entry = 0u, s_active_end = 0u;
/* Optional one-shot 1.6 ms capture, armed only by a diagnostic host command.
   Freeze 16 PWM periods after the first raw >6.45 A excursion. No per-cycle
   buffer writes when unarmed/frozen; intended for DAP readback while idle. */
typedef struct {
  uint16_t adc[3], flags, start, end, min_ccr, direction;
  float conditioned[3];
} ph_current_trace_t;
_Static_assert(sizeof(ph_current_trace_t) == 28u, "current trace wire size");
static volatile ph_current_trace_t s_current_trace[64];
static volatile uint16_t s_trace_state = 0u, s_trace_next = 0u, s_trace_trigger = 0u;
static uint16_t s_trace_post = 0u;
static volatile float s_vel = 0.0f;        /* mechanical velocity, rad/s */
static volatile float s_pos = 0.0f;        /* mechanical position, rad (multi-turn) */
static float   s_vel_int = 0.0f;
static uint32_t s_cal_cnt = 0u;
static float    s_cal_sin = 0.0f, s_cal_cos = 0.0f;
static uint32_t s_cal_n = 0u;
static uint8_t  s_cal_ph = 0u;              /* calibration sub-phase */
static float    s_cal_a = 0.0f, s_cal_b = 0.0f;
static float    s_cal_mechacc = 0.0f;
static volatile uint8_t s_cal_finalize_pending;
static float s_bw_sum_sin[PH_CAL_BW_TEST_COUNT], s_bw_sum_cos[PH_CAL_BW_TEST_COUNT];
static uint16_t s_bw_samples[PH_CAL_BW_TEST_COUNT];
static uint8_t s_bw_step;
static uint32_t s_bw_tick;
static float s_bw_gain[PH_CAL_BW_TEST_COUNT];
static float s_bw_lag[PH_CAL_BW_TEST_COUNT];
static float s_bw_command[PH_CAL_BW_TEST_COUNT];
static float s_bw_saturation[PH_CAL_BW_TEST_COUNT];
static uint32_t s_bw_sat_base;
static float s_bw_measured_hz, s_bw_gain_1k, s_bw_lag_1k_deg;
static uint8_t s_bw_pass;
static uint8_t s_bw_valid[PH_CAL_BW_TEST_COUNT];
static float s_cal_r_low_v, s_cal_r_low_i, s_cal_voltage_error;
static float s_cal_l_samples[PH_CAL_L_REPEATS];
static float s_cal_l_pulse_i[PH_CAL_L_PULSE_MAX_TICKS];
static float s_cal_l_sum_x, s_cal_l_sum_y, s_cal_l_sum_xx, s_cal_l_sum_xy;
static uint8_t s_cal_l_pulse_ticks, s_cal_l_sample_count, s_cal_l_valid_count;
static float s_cal_l_last_a, s_cal_l_last_estimate;
static uint16_t s_cal_oc_count;
static uint32_t s_meas_t = 0u;
static float    s_meas_iss = 0.0f, s_meas_n = 0.0f, s_meas_first = 0.0f, s_meas_did = 0.0f;
static volatile uint32_t s_enc_cycle = 0u;
static volatile uint8_t s_enc_seen = 0u;
static float s_elec_used = 0.0f;
static uint8_t s_calibrated = 0u;
static int16_t s_cogging_map_ma[CFF_COGGING_BINS];
static uint8_t s_cogging_valid;
static float s_cogging_current_applied;
static uint16_t s_cal_pp = 0u;
static int8_t s_cal_dir = 1;
static float s_cal_offset = 0.0f;
static float s_cal_quality = 0.0f;
static float s_cal_forward_error = 0.0f;
static uint32_t s_cal_enc_count = 0u;
static volatile uint32_t s_command_cycle = 0u;
static float s_active_speed_peak, s_fault_speed, s_pwm_elec_used;
static uint32_t s_voltage_saturation_count;
static volatile uint16_t s_pwm_sample_q = (PH_PWM_PERIOD + 1u) / 2u;
static volatile uint8_t s_pwm_sample_mask = 7u;
static volatile uint8_t s_pwm_sample_mask_pipe[2] = {7u, 7u};
static volatile float s_pwm_applied_scale = 1.0f;
static ph_angle_observer_t s_angle_observer;
static volatile float s_observer_mech, s_vel_raw;
/* Optional gearbox-output encoder (FPC1 / TLE5012B, independent PB6 CS). */
static volatile float s_enc2_pos, s_enc2_vel;
static volatile uint8_t s_enc2_ok;
static volatile uint32_t s_enc2_fail;
static uint8_t s_enc2_enabled;
static int8_t s_enc2_dir=1;
static float s_enc2_offset;
static volatile float s_gear_ratio=NAN;
static uint16_t s_enc_mod1, s_enc_mod2;
static volatile float s_enc_delay_us;
static uint8_t s_enc_config_valid;
static uint32_t s_zero_remaining, s_zero_acc[3];
static uint32_t s_send_decim = PH_SEND_DECIM;
static cff_motion_t s_motion;
static uint32_t s_motion_div, s_pwm_count;
static volatile float s_fet_temp=NAN, s_motor_temp=NAN, s_thermal_scale=0;
static float s_bus_current, s_power;
static uint16_t s_disarm_reason;
static uint8_t s_config_loaded, s_command_source, s_can_enabled;
static uint8_t s_can_feedback_active;
static uint8_t s_save_status;
static uint16_t s_fet_adc,s_bus_adc;
static volatile cff_mit_command_t s_mit_command;
static volatile float s_can_position_zero;
static uint8_t s_can_auto_arm=1u, s_can_inhibit;
static volatile float s_can_current;
static uint8_t s_axis_mail[62];
static float s_enc_age_used, s_vq_out, s_id_target, s_iq_target, s_voltage_limit;
#define PH_HOST_TIMEOUT_CYCLES 108000000u /* 750ms; independent of host GUI timers */
#define PH_ENC_PERIOD_CYCLES 72000u /* 500 us at 144 MHz; main-loop best effort */
#define PH_ENC_MAX_AGE_CYCLES 720000u /* stop drive after 5 ms without valid angle */

static float ph_wrap(float a)
{
  while (a >= PH_TWO_PI) { a -= PH_TWO_PI; }
  while (a < 0.0f) { a += PH_TWO_PI; }
  return a;
}

/* Parameter protocol transports the stored motor-side SI values. The host
   applies the single public rpm/degree and gearbox conversion. */
static float ph_axis_param_to_motor(uint16_t id,float v)
{
  (void)id;
  return v;
}
static float ph_motor_param_to_axis(uint16_t id,float v)
{
  (void)id;
  return v;
}

static float ph_encoder_elec(void)
{
  return ph_wrap(s_mech * ((float)s_cfg.encoder_dir * (float)s_cfg.pole_pairs)
                 + s_cfg.encoder_offset);
}

/* SSC CRC-8/J1850: command and data bytes, init FF, poly 1D, xorout FF.
   This is different from the host protocol CRC (poly 07, no xorout). */
static uint8_t ph_enc_crc(uint16_t command, uint16_t data)
{
  const uint8_t b[4] = { (uint8_t)(command >> 8), (uint8_t)command,
                         (uint8_t)(data >> 8), (uint8_t)data };
  uint8_t crc = 0xFFu;
  for (uint32_t i = 0u; i < 4u; i++) {
    crc ^= b[i];
    for (uint32_t j = 0u; j < 8u; j++) {
      crc = (uint8_t)((crc & 0x80u) ? ((crc << 1) ^ 0x1Du) : (crc << 1));
    }
  }
  return (uint8_t)~crc;
}

static float    s_vd_out = 0.0f, s_id_out = 0.0f, s_iq_out = 0.0f;

static float ph_sin(float a)
{
  a = ph_wrap(a);
  a *= (64.0f / PH_PI_2);           /* 256 entries over 2*pi */
  int32_t i = (int32_t)a;
  float fr = a - (float)i;
  i &= 255;
  int32_t j = (i + 1) & 255;
  return s_sin_f[i] + (s_sin_f[j] - s_sin_f[i]) * fr;
}
static float ph_cos(float a) { return ph_sin(a + PH_PI_2); }

#define PH_OC_ADC   1241u   /* ~40 A raw phase peak; independent of filtering */
#define PH_OC_N     5u

static hal_adc_handle_t *s_adc = NULL;
static UX_SLAVE_CLASS_CDC_ACM *s_cdc = NULL;
static hal_dma_handle_t s_dma;
static hal_spi_handle_t *s_spi = NULL;
static volatile uint16_t s_adc_dma[4] = { 2048u, 2048u, 2048u, 2048u };  /* [3] = dummy */

/* TIM1 UPDATE -> ADC1 injected scan -> JDR1/2/3. Conversion apertures are
   hardware-timed even if USB briefly masks the control interrupt. */
static uint8_t ph_adc_read_injected(volatile uint16_t *dst)
{
  uint32_t start = ph_clock_now();
  while (!LL_ADC_IsActiveFlag_JEOS(ADC1)) {
    if ((uint32_t)(ph_clock_now() - start) > 600u) {
      g_ph_adc_to++; s_adc_inj_timeout++;
      return 0u; /* retain last complete scan for a single missed trigger */
    }
  }
  dst[0] = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
  dst[1] = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
  dst[2] = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3);
  LL_ADC_ClearFlag(ADC1, LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JEOC);
  if (dst[0] > 4095u || dst[1] > 4095u || dst[2] > 4095u) {
    g_ph_adc_to++; s_adc_bad_result++; g_ph_fault |= CFF_FAULT_ADC;
    return 0u;
  }
  return 1u;
}

static int16_t  s_sin[PH_LUT_N];
static float    s_ang = 0.0f;
static uint32_t s_f_act = 0u;
static uint32_t s_ramp_div = 0u;
static uint32_t s_decim = 0u;
static uint16_t s_seq = 0u;

static volatile uint8_t  s_txring[PH_TXRING];
static volatile uint16_t s_tx_head = 0u;
static volatile uint16_t s_tx_tail = 0u;

static uint8_t  s_txbuf[544];
static uint8_t  s_tx_busy = 0u;
static uint32_t s_tx_len = 0u;
/* ISR prepares payloads; only main encodes CRC and writes the USB ring.
   This also removes the old ISR/main multi-producer race on s_tx_head. */
static uint8_t s_telem_mail[CFF_TELEM_LEN], s_raw_mail[CFF_CURRENT_RAW_LEN];
static uint8_t s_control_mail[50]; /* u16 seq, 12 floats, same PWM sample */
static volatile uint8_t s_telem_pending = 0u;
static volatile uint8_t s_telem_due = 0u;
static volatile uint16_t s_telem_duty[3];

static uint8_t  s_rxbuf[64];
static uint8_t  s_cmd[64];
static uint8_t  s_cmd_len = 0u;

/* ------------------------------------------------------------------------- */

static uint8_t ph_crc8(const uint8_t *d, uint32_t n)
{
  uint8_t crc = 0xFFu;
  for (uint32_t i = 0u; i < n; i++)
  {
    crc ^= d[i];
    for (uint8_t b = 0u; b < 8u; b++)
    {
      crc = (uint8_t)((crc & 0x80u) ? ((crc << 1) ^ 0x07u) : (crc << 1));
    }
  }
  return crc;
}

/* Append one complete framed message to the TX ring (SPSC: ISR writes head). */
static void ph_tx_frame(uint8_t type, const uint8_t *pl, uint8_t len)
{
  uint16_t need = (uint16_t)(4u + (uint16_t)len + 1u);
  uint16_t head = s_tx_head;
  uint16_t used = (uint16_t)((head - s_tx_tail) & (PH_TXRING - 1u));
  if ((uint16_t)(used + need) >= PH_TXRING) { g_ph_dropped++; return; }

  uint8_t tmp[4u + CFF_MAX_PAYLOAD + 1u];
  tmp[0] = CFF_SYNC0; tmp[1] = CFF_SYNC1; tmp[2] = type; tmp[3] = len;
  for (uint16_t i = 0u; i < len; i++) { tmp[4u + i] = pl[i]; }
  tmp[4u + len] = ph_crc8(&tmp[2], (uint32_t)len + 2u);

  for (uint16_t i = 0u; i < need; i++)
  {
    s_txring[head] = tmp[i];
    head = (uint16_t)((head + 1u) & (PH_TXRING - 1u));
  }
  s_tx_head = head;
}

/* Build and queue one telemetry frame (called from the 40 kHz ISR). */
static int16_t ph_milliamp_saturate(float amps)
{
  float v = amps * 1000.0f;
  if (v > 32767.0f) { return 32767; }
  if (v < -32768.0f) { return -32768; }
  return (int16_t)v;
}

static void ph_tx_telem(uint16_t ia_raw, uint16_t ib_raw, uint16_t ic_raw,
                        uint16_t da, uint16_t db, uint16_t dc)
{
  uint8_t q[CFF_TELEM_LEN];
  uint16_t raw = g_enc_angle;                                             /* 0..32767 */
  uint16_t a100 = (uint16_t)(((uint32_t)raw * 36000u) >> 15);            /* mech deg*100 */
  float elecf = ph_wrap(s_elec_used); /* actual Park angle, including i/f/calibration */
  uint16_t e100 = (uint16_t)(elecf * (36000.0f / PH_TWO_PI));

  const uint16_t rr[3] = { ia_raw, ib_raw, ic_raw };
  int16_t imA[3];
  for (uint32_t i = 0u; i < 3u; i++)
  {
    imA[i] = ph_milliamp_saturate(s_current_filter.output[i]);
  }
  uint8_t flags = (uint8_t)(((g_ph_state == CFF_AXIS_CLOSED_LOOP) ? CFF_FLAG_CLOSED_LOOP : 0u) |
                            (g_ph_enc_ok ? CFF_FLAG_ENCODER_OK : 0u) |
                            (s_calibrated ? CFF_FLAG_CALIBRATED : 0u));

  #define W16(o, v) do { q[o] = (uint8_t)((uint16_t)(v) & 0xFFu); q[(o) + 1u] = (uint8_t)(((uint16_t)(v) >> 8) & 0xFFu); } while (0)
  W16(CFF_T_SEQ, s_seq);
  W16(CFF_T_ANGLE_RAW, raw);
  W16(CFF_T_ANGLE_X100, a100);
  W16(CFF_T_ELEC_X100, e100);
  W16(CFF_T_IA, (uint16_t)imA[0]);
  W16(CFF_T_IB, (uint16_t)imA[1]);
  W16(CFF_T_IC, (uint16_t)imA[2]);
  W16(CFF_T_ID, ph_milliamp_saturate(s_id_out));
  W16(CFF_T_IQ, ph_milliamp_saturate(s_iq_out));
  float axis_pos=s_enc2_enabled?(s_enc2_dir*s_enc2_pos-s_enc2_offset):(s_cfg.encoder_dir*s_pos-s_cfg.pos_zero);
  float axis_vel=s_enc2_enabled?(s_enc2_dir*s_enc2_vel):(s_cfg.encoder_dir*s_vel);
  W16(CFF_T_VEL_X100, cff_i16_saturate(axis_vel*100));
  W16(CFF_T_POS_X100, cff_i16_saturate(axis_pos*100));
  W16(CFF_T_VBUS_MV, g_ph_vbus);
  W16(CFF_T_DUTY_A, da);
  W16(CFF_T_DUTY_B, db);
  W16(CFF_T_DUTY_C, dc);
  W16(CFF_T_FAULT, g_ph_fault);
  q[CFF_T_AXIS_STATE] = g_ph_state;
  q[CFF_T_AXIS_MODE] = s_cfg.control_mode;
  q[CFF_T_INPUT_MODE] = s_cfg.input_mode;
  q[CFF_T_FLAGS] = flags;
  #undef W16

  uint8_t raw_pl[CFF_CURRENT_RAW_LEN];
  raw_pl[0] = (uint8_t)s_seq; raw_pl[1] = (uint8_t)(s_seq >> 8);
  for (uint32_t i = 0u; i < 3u; i++) {
    raw_pl[2u + i*2u] = (uint8_t)rr[i];
    raw_pl[3u + i*2u] = (uint8_t)(rr[i] >> 8);
    memcpy(&raw_pl[8u + i*4u], &s_current_raw[i], 4u);
  }
  raw_pl[20] = (uint8_t)s_sample_flags; raw_pl[21] = (uint8_t)(s_sample_flags >> 8);
  if (s_telem_pending) { g_ph_dropped++; }
  memcpy(s_telem_mail, q, CFF_TELEM_LEN);
  memcpy(s_raw_mail, raw_pl, CFF_CURRENT_RAW_LEN);
  uint8_t control_active = g_ph_state != CFF_AXIS_IDLE;
  const float axis[15]={ axis_pos, axis_vel,s_current_filter.output[0],s_current_filter.output[1],s_current_filter.output[2],
    control_active?s_id_out:0,control_active?s_iq_out:0,s_fet_temp,s_motor_temp,s_bus_current,s_power,(float)s_motion.traj.done,
    (float)s_command_source,(float)(!s_can_inhibit),s_gear_ratio};
  s_axis_mail[0]=(uint8_t)s_seq;s_axis_mail[1]=(uint8_t)(s_seq>>8);memcpy(s_axis_mail+2,axis,sizeof(axis));
  const float control[12] = { control_active ? s_id_target : 0.0f,
    control_active ? s_iq_target : 0.0f, control_active ? s_vd_out : 0.0f,
    control_active ? s_vq_out : 0.0f,
    s_elec_used, s_pwm_elec_used, s_observer_mech, s_vel_raw,
    s_enc_age_used * 1000000.0f, s_voltage_limit,
    s_current_raw[0] + s_current_raw[1] + s_current_raw[2], s_enc_delay_us };
  s_control_mail[0] = (uint8_t)s_seq; s_control_mail[1] = (uint8_t)(s_seq >> 8);
  memcpy(&s_control_mail[2], control, sizeof(control));
  __DMB(); s_telem_pending = 1u;
  s_seq++;
  g_ph_frames++;
}

/* ---- parameters (GET/SET). Values are transported as f32. ---------------- */
static float ph_get_param(uint16_t id, uint8_t *ok)
{
  *ok = 1u;
  if (id >= 0x0224u && id < 0x0224u + PH_CAL_BW_TEST_COUNT) return s_bw_gain[id-0x0224u];
  if (id >= 0x0234u && id < 0x0234u + PH_CAL_BW_TEST_COUNT) return s_bw_lag[id-0x0234u];
  if (id >= 0x0244u && id < 0x0244u + PH_CAL_BW_TEST_COUNT) return s_bw_command[id-0x0244u];
  if (id >= 0x0254u && id < 0x0254u + PH_CAL_BW_TEST_COUNT) return 100.0f*s_bw_saturation[id-0x0254u];
  if (id >= 0x0264u && id < 0x0264u + PH_CAL_BW_TEST_COUNT) return (float)s_bw_valid[id-0x0264u];
  const cff_param_desc_t *desc=cff_param_find(id);
  if(desc) return ph_motor_param_to_axis(id,cff_param_read(&s_cfg,desc));
  switch (id)
  {
    case CFF_PARAM_MOTOR_POLE_PAIRS:               return (float)s_cfg.pole_pairs;
    case CFF_PARAM_MOTOR_PHASE_RESISTANCE:         return s_cfg.phase_resistance;
    case CFF_PARAM_MOTOR_PHASE_INDUCTANCE:         return s_cfg.phase_inductance;
    case CFF_PARAM_MOTOR_CURRENT_LIMIT:            return s_cfg.current_limit;
    case CFF_PARAM_MOTOR_CALIB_CURRENT:            return s_cfg.calibration_current;
    case CFF_PARAM_ENCODER_OFFSET:                 return s_cfg.encoder_offset;
    case CFF_PARAM_ENCODER_DIR:                    return (float)s_cfg.encoder_dir;
    case CFF_PARAM_ENCODER_BANDWIDTH:              return s_cfg.encoder_bandwidth;
    case CFF_PARAM_CONTROLLER_POS_GAIN:            return s_cfg.pos_gain;
    case CFF_PARAM_CONTROLLER_VEL_GAIN:            return s_cfg.vel_gain;
    case CFF_PARAM_CONTROLLER_VEL_INTEGRATOR_GAIN: return s_cfg.vel_integrator_gain;
    case CFF_PARAM_CONTROLLER_CUR_GAIN_P:          return s_cfg.current_gain_p;
    case CFF_PARAM_CONTROLLER_CUR_GAIN_I:          return s_cfg.current_gain_i;
    case CFF_PARAM_LIMIT_VEL:                      return s_cfg.limit_vel;
    case 0x0402u: return s_cfg.overspeed_limit;
    case 0x0416u: return s_gear_ratio;
    case CFF_PARAM_LIMIT_CURRENT:                  return s_cfg.current_limit;
    case CFF_PARAM_AXIS_CONTROL_MODE:              return (float)s_cfg.control_mode;
    case CFF_PARAM_AXIS_INPUT_MODE:                return (float)s_cfg.input_mode;
    case CFF_PARAM_AXIS_STATE:                     return (float)g_ph_state;
    case CFF_PARAM_AXIS_INPUT_POS:                 return s_cfg.input_pos;
    case CFF_PARAM_AXIS_INPUT_VEL:                 return s_cfg.input_vel;
    case CFF_PARAM_AXIS_INPUT_TORQUE:              return s_cfg.input_torque;
    case 0x0203u: return ph_wrap(s_elec_used);
    case 0x0204u: return ph_encoder_elec();
    case 0x0205u: { float d = ph_wrap(s_elec_used - ph_encoder_elec());
                    return (d > PH_PI) ? d - PH_TWO_PI : d; }
    case 0x0206u: return s_enc_seen ? (float)(ph_clock_now() - s_enc_cycle) / 144000.0f : -1.0f;
    case 0x0207u: return s_cal_quality;
    case 0x0208u: return (float)s_cal_ph;
    case 0x0209u: return (float)g_enc_w2;
    case 0x020Au: return (float)g_enc_fail;
    case 0x020Bu: return ph_cal_progress(s_cal_ph, s_cal_cnt);
    case 0x020Cu: return (float)PH_CAL_TOTAL_TICKS / (float)PH_ISR_HZ;
    case 0x020Du: return s_cal_voltage_error;
    case 0x020Eu: return s_cal_r_low_i;
    case 0x020Fu: return s_cal_r_low_v;
    case 0x0210u: return s_enc_config_valid ? (float)s_enc_mod1 : -1.0f;
    case 0x0211u: return s_enc_config_valid ? (float)s_enc_mod2 : -1.0f;
    case 0x0212u: return s_enc_delay_us;
    case 0x0213u: return s_observer_mech;
    case 0x0214u: return s_vel_raw;
    case 0x0215u: return (float)s_cogging_valid;
    case 0x0216u: return s_cogging_current_applied;
    case 0x0217u: return (float)s_pwm_sample_mask;
    case 0x0218u: return s_pwm_applied_scale;
    case 0x0219u: return (float)s_pwm_sample_q;
    case 0x021Au: return (float)s_cal_l_valid_count;
    case 0x021Bu: return s_cal_l_last_a;
    case 0x021Cu: return s_cal_l_last_estimate;
    case 0x0220u: return s_bw_measured_hz;
    case 0x0221u: return s_bw_gain_1k;
    case 0x0222u: return s_bw_lag_1k_deg;
    case 0x0223u: return (float)s_bw_pass;
    case 0x062Au: return 2.0f; /* frequency-response protocol v2 */
    case 0x0625u: return (float)s_adc_inj_timeout;
    case 0x0626u: return (float)s_adc_reg_timeout;
    case 0x0627u: return (float)s_adc_bad_result;
    case 0x0600u:                                   return (float)g_ph_spike;
    case 0x0601u:                                   return (float)g_ph_spike_max;
    case 0x0602u:                                   return (float)g_ph_spike_raw[0];
    case 0x0603u:                                   return (float)g_ph_spike_raw[1];
    case 0x0604u:                                   return (float)g_ph_spike_raw[2];
    case 0x0610u: return (float)g_ph_adc_to;
    case 0x0611u: return (float)g_ph_adc_wait;
    case 0x0612u: return (float)g_ph_isr_max;
    case 0x0613u: return (float)g_ph_cnt_upd;
    case 0x0614u: return (float)g_ph_cnt_rd;
    case 0x0615u: return s_i_off[0];
    case 0x0616u: return s_i_off[1];
    case 0x0617u: return s_i_off[2];
    case 0x0618u: return (float)s_impulses;
    case 0x0619u: return (float)s_active_adc_max;
    case 0x061Au: return (float)s_active_isr_max;
    case 0x061Bu: return (float)s_active_entry;
    case 0x061Cu: return (float)s_active_end;
    case 0x061Du: return (float)s_trace_state;
    case 0x061Eu: return (float)s_trace_trigger;
    case 0x061Fu: return (float)s_trace_next;
    case 0x0620u: return 1.0f; /* hardware injected scan */
    case 0x0621u: return s_active_speed_peak;
    case 0x0622u: return s_fault_speed;
    case 0x0623u: return (float)s_voltage_saturation_count;
    case 0x0624u: return s_pwm_elec_used;
    case 0x0630u: return (float)PH_ISR_HZ / (float)s_send_decim;
    case 0x0810: return s_fet_temp;
    case 0x0811: return s_motor_temp;
    case 0x0812: return s_thermal_scale;
    case 0x0813: return s_bus_current;
    case 0x0814: return s_power;
    case 0x0815: return (float)s_disarm_reason;
    case 0x0816: return (float)s_motion.traj.done;
    case 0x0820: return (float)s_config_loaded;
    case 0x0821: return (float)g_config_sequence;
    case 0x0822: return (float)s_save_status;
    case 0x0823: return 2.0f; /* extension protocol revision */
    case 0x0824: return (float)s_fet_adc;
    case 0x0825: return (float)s_bus_adc;
    case 0x0802: return (float)(!s_can_inhibit);
    case 0x0803: return (float)s_can_auto_arm;
    case 0x0804: return CFF_MIT_CURRENT_FULL_SCALE_A;
    case 0x0805: return (CFF_MIT_I_MAX-CFF_MIT_I_MIN)/4095.0f; /* A/count */
    case 0x0830: return (float)g_can_rx;
    case 0x0831: return (float)g_can_tx;
    case 0x0832: return (float)g_can_dropped;
    case 0x0833: return (float)g_can_restarts;
    case 0x0834: return (float)s_command_source;
    case 0x0835: return s_can_current;
    case 0x0840: return (float)s_enc2_enabled;
    case 0x0841: return (float)s_enc2_dir;
    case 0x0842: return s_enc2_offset;
    case 0x0843: return (float)s_enc2_ok;
    case 0x0844: return (float)s_enc2_fail;
    case 0x0845: return s_enc2_pos;
    case 0x0846: return s_enc2_vel;
    default: *ok = 0u; return 0.0f;
  }
}

static void ph_safe_stop(uint16_t fault)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (fault && g_ph_state != CFF_AXIS_IDLE) { s_fault_speed = s_vel; }
  g_ph_fault |= fault;
  if(fault) s_disarm_reason|=fault;
  g_ph_state = CFF_AXIS_IDLE;
  g_ph_mod_milli = 0u; g_ph_if_hz = 0u;
  s_iq_ref = 0.0f; s_can_current=0.0f; s_cfg.input_torque = 0.0f; s_cfg.input_vel = 0.0f;
  s_motion.initialized=0; s_can_enabled=0; s_can_inhibit=1u; s_command_source=0;
  s_config_pending=0; s_power=s_bus_current=0;
  s_id_int = 0.0f; s_iq_int = 0.0f; s_vel_int = 0.0f;
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  __set_PRIMASK(primask);
}

static void ph_request_state(uint8_t requested)
{
  if (requested == CFF_AXIS_IDLE) { ph_safe_stop(0u); return; }
  if (g_ph_state != CFF_AXIS_IDLE || g_ph_fault) { return; }
  if(s_config_pending) return;
  if (requested != CFF_AXIS_CALIBRATION && requested != CFF_AXIS_CLOSED_LOOP) { return; }
  if (s_zero_remaining || !s_enc_config_valid) { ph_safe_stop(CFF_FAULT_ENCODER); return; }
  if(requested==CFF_AXIS_CLOSED_LOOP && s_enc2_enabled && !s_enc2_ok) { ph_safe_stop(CFF_FAULT_ENCODER); return; }
  if((s_cfg.fet_temp_enable&&!isfinite(s_fet_temp))||s_thermal_scale<=0) { ph_safe_stop(CFF_FAULT_FET_TEMP); return; }
  if(requested==CFF_AXIS_CLOSED_LOOP && !g_ph_if_hz && !s_calibrated) {
    ph_safe_stop(CFF_FAULT_CALIBRATION); return;
  }
  if(requested==CFF_AXIS_CLOSED_LOOP && s_cfg.control_mode==CFF_CTRL_POSITION &&
     (!s_enc2_enabled||!s_enc2_ok||!isfinite(s_gear_ratio))) { ph_safe_stop(CFF_FAULT_ENCODER); return; }
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_id_int = s_iq_int = s_vel_int = 0.0f;
  s_iq_ref = 0.0f; s_cfg.input_torque = 0.0f; s_cfg.input_vel = 0.0f;
  float joint_pos=s_enc2_enabled ? (s_enc2_dir*s_enc2_pos-s_enc2_offset) :
                  (float)s_cfg.encoder_dir*s_pos-s_cfg.pos_zero;
  float joint_vel=s_enc2_enabled ? s_enc2_dir*s_enc2_vel :
                  (float)s_cfg.encoder_dir*s_vel;
  s_cfg.input_pos=joint_pos;
  cff_motion_reset(&s_motion,joint_pos,joint_vel); s_motion_div=0;
  s_active_speed_peak = 0.0f; s_fault_speed = 0.0f;
  s_voltage_saturation_count = 0u;
  if (requested == CFF_AXIS_CLOSED_LOOP) {
    s_trace_state = 1u; s_trace_next = 0u; s_trace_post = 0u; s_trace_trigger = 0u;
  }
  if (requested == CFF_AXIS_CALIBRATION) {
    s_cal_finalize_pending = 0u;
    s_bw_step=0u; s_bw_tick=0u; s_bw_measured_hz=0.0f;
    s_bw_gain_1k=0.0f; s_bw_lag_1k_deg=0.0f; s_bw_pass=0u;
    memset(s_bw_sum_sin,0,sizeof(s_bw_sum_sin));
    memset(s_bw_sum_cos,0,sizeof(s_bw_sum_cos));
    memset(s_bw_samples,0,sizeof(s_bw_samples));
    memset(s_bw_gain,0,sizeof(s_bw_gain));
    s_calibrated = 0u; s_cal_quality = 0.0f; s_cal_pp = 0u;
    s_cogging_valid=0u;s_cogging_current_applied=0.0f;
    memset(s_cogging_map_ma,0,sizeof(s_cogging_map_ma));
    s_cal_enc_count = g_enc_count; g_ph_if_hz = 0u;
    s_cal_cnt = 0u; s_cal_ph = 0u; s_cal_n = 0u;
    s_cal_a = s_cal_b = s_cal_sin = s_cal_cos = 0.0f;
    s_cal_r_low_v = s_cal_r_low_i = s_cal_voltage_error = 0.0f;
    s_cal_oc_count = 0u;
    s_cal_l_pulse_ticks = PH_CAL_L_PULSE_MAX_TICKS; s_cal_l_sample_count = 0u; s_cal_l_valid_count = 0u;
    s_cal_l_last_a = s_cal_l_last_estimate = 0.0f;
    memset(s_cal_l_samples, 0, sizeof(s_cal_l_samples));
    memset(s_cal_l_pulse_i, 0, sizeof(s_cal_l_pulse_i));
    s_cal_l_sum_x = s_cal_l_sum_y = s_cal_l_sum_xx = s_cal_l_sum_xy = 0.0f;
  }
  s_command_cycle = ph_clock_now();
  s_command_source=1u;
  g_ph_state = requested;
  __set_PRIMASK(primask);
}

/* Status: 0 success, 1 bad/range/unsupported, 2 busy, 3 hardware failure. */
static uint8_t ph_set_param(uint16_t id,float v)
{
  if(!isfinite(v)) return 1;
  if(s_config_pending) return 2;
  if(id==0x061Eu) {
    if(v!=1.0f) return 1;
    if(g_ph_state!=CFF_AXIS_IDLE || g_ph_fault || !s_calibrated ||
       !s_enc_config_valid || s_zero_remaining ||
       !(s_cfg.phase_resistance>0.0f) || !(s_cfg.phase_inductance>0.0f)) return 2;
    memset(s_bw_sum_sin,0,sizeof(s_bw_sum_sin));
    memset(s_bw_sum_cos,0,sizeof(s_bw_sum_cos));
    memset(s_bw_samples,0,sizeof(s_bw_samples));
    memset(s_bw_gain,0,sizeof(s_bw_gain));
    memset(s_bw_lag,0,sizeof(s_bw_lag));
    memset(s_bw_saturation,0,sizeof(s_bw_saturation));
    memset(s_bw_valid,0,sizeof(s_bw_valid));
    for(unsigned i=0u;i<PH_CAL_BW_TEST_COUNT;i++)
      s_bw_command[i]=ph_cal_bw_command(s_cfg.phase_resistance,s_cfg.phase_inductance,
                                        ph_cal_bw_frequency(i));
    s_bw_step=0u; s_bw_tick=0u; s_cal_cnt=0u; s_cal_ph=6u;
    s_id_int=0.0f; s_iq_int=0.0f; s_command_cycle=ph_clock_now(); s_command_source=1u;
    g_ph_state=CFF_AXIS_CALIBRATION;
    return 0;
  }
  if(id==CFF_PARAM_AXIS_STATE) {
    if(v!=1&&v!=2&&v!=3) return 1;
    ph_request_state((uint8_t)v); return g_ph_state==(uint8_t)v?0:2;
  }
  int target=id==0x0503||id==0x0504||id==0x0505;
  if((g_ph_state!=CFF_AXIS_IDLE||g_ph_mod_milli) && (!target||s_command_source!=1u)) return 2;
  if(id==0x0802) {
    if(v!=0&&v!=1) return 1;
    if(v==0) ph_safe_stop(0);
    else if(g_ph_fault||!s_calibrated) return 2;
    else {s_can_inhibit=0;s_can_enabled=1;}
    return 0;
  }
  if(id==0x0840u) { if(v!=0&&v!=1)return 1; s_enc2_enabled=(uint8_t)v; s_motion.initialized=0; return 0; }
  if(id==0x0841u) { if(v!=1&&v!=-1)return 1; s_enc2_dir=(int8_t)v; s_motion.initialized=0; return 0; }
  if(id==0x0842u) { if(fabsf(v)>1000000.0f)return 1; s_enc2_offset=v; s_motion.initialized=0; return 0; }
  if(id==0x0503u && s_cfg.control_mode==CFF_CTRL_POSITION) {
    float current=s_enc2_enabled ? (s_enc2_dir*s_enc2_pos-s_enc2_offset) :
                                  (s_cfg.encoder_dir*s_pos-s_cfg.pos_zero);
    if(fabsf(v-current)>CFF_POSITION_STEP_OUTPUT_RAD) return 1;
  }
  if(id==0x0630) {
    if(v!=100&&v!=500&&v!=1000) return 1;
    s_send_decim=PH_ISR_HZ/(uint32_t)v; return 0;
  }
  if(id==0x061D&&v==1) {
    s_trace_state=0;s_trace_next=s_trace_post=s_trace_trigger=0;__DMB();s_trace_state=1;return 0;
  }
  const cff_param_desc_t *desc=cff_param_find(id);
  if(!desc) return 1;
  cff_config_t candidate=s_cfg;
  float stored=ph_axis_param_to_motor(id,v);
  if(!cff_param_write(&candidate,desc,stored)) return 1;
  if(id==0x0200) candidate.encoder_offset=ph_wrap(v);
  if(id==0x0500 && candidate.control_mode==CFF_CTRL_POSITION) candidate.input_mode=CFF_INPUT_TRAP_TRAJ;
  else if(id==0x0500&&!cff_input_compatible(candidate.control_mode,candidate.input_mode)) candidate.input_mode=1;
  if(candidate.pos_limits_enable&&target&&id==0x0503&&
     (candidate.input_pos<candidate.pos_min||candidate.input_pos>candidate.pos_max)) return 1;
  if(!cff_config_valid(&candidate)||candidate.can_node<1||candidate.can_node>7) return 1;
  if(id==0x0107) {
    if(candidate.phase_resistance<=0||candidate.phase_inductance<=0) return 1;
    candidate.current_gain_p=candidate.phase_inductance*v;
    candidate.current_gain_i=candidate.phase_resistance*v;
  }
  if(id==0x0801&&!cff_can_init(candidate.can_bitrate_kbps)) return 3;
  uint32_t mask=__get_PRIMASK();__disable_irq();s_cfg=candidate;
  if(id==0x0100||id==0x0200||id==0x0201) s_calibrated=0;
  if(target) s_command_cycle=ph_clock_now();
  __set_PRIMASK(mask);return 0;
}

static void ph_ack(uint8_t command,uint8_t status,uint16_t id) {
  uint8_t p[4]={command,status,(uint8_t)id,(uint8_t)(id>>8)};
  ph_tx_frame(CFF_MSG_ACK,p,4);
}

static void ph_clear_faults(void) {
  if(g_ph_state!=CFF_AXIS_IDLE||g_ph_mod_milli) return;
  uint32_t mask=__get_PRIMASK();__disable_irq();
  g_ph_trip=g_ph_oc_count=0;g_ph_fault=0;g_ph_spike=g_ph_spike_max=0;
  __set_PRIMASK(mask);
}

static void ph_save_config(void) {
  if(g_ph_state!=CFF_AXIS_IDLE||g_ph_mod_milli||s_config_pending) {ph_ack(CFF_MSG_SAVE,2,0);return;}
  /* Single-bank erase can stall instructions. Gate off and mask only TIM1;
     SysTick stays enabled for HAL timeouts. Re-zero before allowing a drive. */
  TIM1->BDTR&=~TIM_BDTR_MOE; NVIC_DisableIRQ(TIM1_UPD_IRQn);
  int ok=cff_store_save(&s_cfg,s_calibrated,s_cal_quality,s_cogging_map_ma,s_cogging_valid,
                        s_enc2_enabled,s_enc2_dir,s_enc2_offset);
  LL_ADC_ClearFlag(ADC1,LL_ADC_FLAG_JEOS|LL_ADC_FLAG_JEOC);
  LL_TIM_ClearFlag_UPDATE(TIM1);NVIC_ClearPendingIRQ(TIM1_UPD_IRQn);
  memset(s_zero_acc,0,sizeof(s_zero_acc));s_zero_remaining=408;
  s_current_filter.initialized=0;s_enc_seen=0;
  NVIC_EnableIRQ(TIM1_UPD_IRQn);
  s_save_status=ok?1:2;ph_ack(CFF_MSG_SAVE,ok?0:3,0);
}

static void ph_reply_param(uint16_t id, float v)
{
  uint8_t q[6];
  q[0] = (uint8_t)(id & 0xFFu); q[1] = (uint8_t)(id >> 8);
  memcpy(&q[2], &v, 4u);
  ph_tx_frame(CFF_MSG_PARAM, q, 6u);
}

static void ph_handle_cmd(uint8_t type, const uint8_t *pl, uint8_t len)
{
  switch (type)
  {
    /* ---- legacy bench commands (V/f test) ---- */
    case 0x01u: if (len == 2u) {
      uint16_t mod = (uint16_t)(pl[0] | ((uint16_t)pl[1] << 8));
      if (mod == 0u || (mod <= 200u && g_ph_state == CFF_AXIS_IDLE && !g_ph_fault)) {
        g_ph_mod_milli = mod; s_command_cycle = ph_clock_now();
      }
    } break;
    case 0x02u: if (len >= 2u) { g_ph_freq_deci = (uint16_t)(pl[0] | ((uint16_t)pl[1] << 8)); } break;
    case 0x03u: g_ph_streaming = 1u; break;
    case 0x04u: g_ph_streaming = 0u; break;
    case 0x05u: s_seq = 0u; g_ph_frames = 0u; g_ph_dropped = 0u; break;
    case 0x06u: if (g_ph_state == CFF_AXIS_IDLE && !g_ph_mod_milli) { g_ph_trip = 0u; g_ph_oc_count = 0u; } break;
    case 0x09u: if (len == 2u) {
      uint16_t hz = (uint16_t)(pl[0] | ((uint16_t)pl[1] << 8));
      if (hz == 0u || (g_ph_state == CFF_AXIS_IDLE && hz <= 2000u)) { g_ph_if_hz = hz; }
    } break;
    /* ---- CrowerFOC protocol ---- */
    case CFF_MSG_CONFIG_PATCH: {
      uint8_t status=1;
      if(g_ph_state!=CFF_AXIS_IDLE||g_ph_mod_milli) {ph_ack(type,2,0);break;}
      if(len==1&&pl[0]==0) {s_staged_cfg=s_cfg;s_config_pending=1;s_staged_start=HAL_GetTick();status=0;}
      else if(len==1&&pl[0]==3) {s_config_pending=0;status=0;}
      else if(len>1&&pl[0]==1&&(len-1u)%6u==0&&s_config_pending) {
      cff_config_t next=s_staged_cfg;status=0;
        for(unsigned i=1;i<len;i+=6) {
          uint16_t id=(uint16_t)(pl[i]|(uint16_t)pl[i+1]<<8);float value;memcpy(&value,pl+i+2,4);
          const cff_param_desc_t *desc=cff_param_find(id);
          float stored=ph_axis_param_to_motor(id,value);
          if(!desc||id==0x0503||id==0x0504||id==0x0505||!cff_param_write(&next,desc,stored)) {status=1;break;}
          if(id==0x0200) next.encoder_offset=ph_wrap(value);
        }
        if(status==0) {s_staged_cfg=next;s_staged_start=HAL_GetTick();}
      } else if(len==1&&pl[0]==2&&s_config_pending) {
        if(cff_config_valid(&s_staged_cfg)) {
          if(s_staged_cfg.can_bitrate_kbps!=s_cfg.can_bitrate_kbps&&!cff_can_init(s_staged_cfg.can_bitrate_kbps)) status=3;
          else {
            uint32_t mask=__get_PRIMASK();__disable_irq();s_cfg=s_staged_cfg;
            s_calibrated=0;s_can_inhibit=1;s_motion.initialized=0;__set_PRIMASK(mask);
            s_config_pending=0;status=0;
          }
        }
      }
      ph_ack(type,status,0);break;
    }
    case CFF_MSG_STATE:
      if(len!=1) {ph_ack(type,1,0);break;}
      if(s_command_source==2&&pl[0]!=1) {ph_ack(type,2,0);break;}
      ph_request_state(pl[0]);ph_ack(type,g_ph_state==pl[0]?0:2,0);break;
    case CFF_MSG_HEARTBEAT: if (len == 0u && s_command_source==1) { s_command_cycle = ph_clock_now(); } break;
    case CFF_MSG_SETPOINT: if (len == 4u) {
                             float v; memcpy(&v, pl, 4u);
                             if (!isfinite(v)||s_command_source==2) { ph_ack(type,1,0);break; }
                             uint16_t id=s_cfg.control_mode==2?0x0503:(s_cfg.control_mode==1?0x0504:0x0505);
                             ph_ack(type,ph_set_param(id,v),id);
                           } break;
    case CFF_MSG_SET: if(len==6) {
      float v;memcpy(&v,pl+2,4);uint16_t id=(uint16_t)(pl[0]|(uint16_t)pl[1]<<8);
      ph_ack(type,ph_set_param(id,v),id);
    } else ph_ack(type,1,0); break;
    case CFF_MSG_GET:      if (len >= 2u) { uint8_t ok; uint16_t id = (uint16_t)(pl[0] | ((uint16_t)pl[1] << 8));
                             if (id >= 0x0700u && id <= 0x073Fu && s_trace_state == 3u) {
                               uint16_t index = id - 0x0700u;
                               uint8_t q[30]; q[0] = (uint8_t)index; q[1] = 0u;
                               memcpy(&q[2], (const void *)&s_current_trace[index], 28u);
                               ph_tx_frame(CFF_MSG_CURRENT_TRACE, q, sizeof(q)); break;
                             }
                             float v = ph_get_param(id, &ok); if (ok != 0u) { ph_reply_param(id, v); } else ph_ack(type,1,id); } break;
    case CFF_MSG_CLEAR:    if (g_ph_state != CFF_AXIS_IDLE || g_ph_mod_milli) { break; }
                           ph_clear_faults();ph_ack(type,0,0);break;
    case CFF_MSG_SAVE: if(len==0) ph_save_config(); else ph_ack(type,1,0); break;
    case CFF_MSG_REBOOT:   if (g_ph_state == CFF_AXIS_IDLE && !g_ph_mod_milli) { NVIC_SystemReset(); } break;
    default: break;
  }
}

/* Host->device: AA 55 | type | len | <payload> | crc8(type..payload) */
static void ph_parse_rx(const uint8_t *b, uint32_t n)
{
  for (uint32_t i = 0u; i < n; i++)
  {
    uint8_t c = b[i];
    if (s_cmd_len == 0u)
    {
      if (c == CFF_SYNC0) { s_cmd[0] = c; s_cmd_len = 1u; }
    }
    else if (s_cmd_len == 1u)
    {
      if (c == CFF_SYNC1) { s_cmd[1] = c; s_cmd_len = 2u; }
      else if (c != CFF_SYNC0) { s_cmd_len = 0u; }
    }
    else
    {
      s_cmd[s_cmd_len++] = c;
      if (s_cmd_len >= 4u)
      {
        uint32_t total = (uint32_t)s_cmd[3] + 5u;   /* 2 sync + type + len + payload + crc */
        if (total > (uint32_t)sizeof(s_cmd)) { s_cmd_len = 0u; }
        else if (s_cmd_len >= total)
        {
          if (ph_crc8(&s_cmd[2], (uint32_t)s_cmd[3] + 2u) == s_cmd[total - 1u])
          {
            ph_handle_cmd(s_cmd[2], &s_cmd[4], s_cmd[3]);
          }
          s_cmd_len = 0u;
        }
      }
    }
  }
}

/* One independent single-channel conversion (software trigger). */
static uint16_t ph_adc_read_ch(uint32_t ch)
{
  /* Clear old sequence flags BEFORE triggering, then wait for this sequence.
     Do not use stale DR contents after a timeout as a new current sample. */
  uint32_t start = ph_clock_now();
  while (LL_ADC_REG_IsConversionOngoing(ADC1)) {
    if ((uint32_t)(ph_clock_now() - start) > 500u) {
      g_ph_adc_to++; s_adc_reg_timeout++; g_ph_fault |= CFF_FAULT_ADC; return 0xFFFFu;
    }
  }
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, ch);
  LL_ADC_ClearFlag(ADC1, LL_ADC_FLAG_EOC | LL_ADC_FLAG_EOS | LL_ADC_FLAG_OVR);
  LL_ADC_REG_StartConversion(ADC1);
  start = ph_clock_now();
  while (LL_ADC_IsActiveFlag_EOS(ADC1) == 0u) {
    if ((uint32_t)(ph_clock_now() - start) > 500u) {
      g_ph_adc_to++; s_adc_reg_timeout++; g_ph_fault |= CFF_FAULT_ADC; return 0xFFFFu;
    }
  }
  uint16_t v = (uint16_t)LL_ADC_REG_ReadConversionData12(ADC1);
  if ((v > 4095u) || LL_ADC_IsActiveFlag_OVR(ADC1)) {
    g_ph_adc_to++; s_adc_bad_result++; g_ph_fault |= CFF_FAULT_ADC; return 0xFFFFu;
  }
  return v;
}

/* Three phase currents (each the "first" of its sequence -> no crosstalk). */
static void ph_adc_read3(volatile uint16_t *dst)
{
  dst[0] = ph_adc_read_ch(LL_ADC_CHANNEL_4);
  dst[1] = ph_adc_read_ch(LL_ADC_CHANNEL_5);
  dst[2] = ph_adc_read_ch(LL_ADC_CHANNEL_6);
}

/* ADC2 is owned by the main task for bus and temperature conversions.
   Current ADC1 remains exclusively PWM-triggered. Clear stale flags first. */
static uint16_t ph_adc2_read(uint32_t channel) {
  LL_ADC_REG_SetSequencerRanks(ADC2,LL_ADC_REG_RANK_1,channel);
  LL_ADC_ClearFlag(ADC2,LL_ADC_FLAG_EOC|LL_ADC_FLAG_EOS|LL_ADC_FLAG_OVR);
  LL_ADC_REG_StartConversion(ADC2);uint32_t started=ph_clock_now();
  while(!LL_ADC_IsActiveFlag_EOS(ADC2)) {
    if((uint32_t)(ph_clock_now()-started)>14400u) return 0xFFFF;
  }
  uint16_t result=(uint16_t)LL_ADC_REG_ReadConversionData12(ADC2);
  if(LL_ADC_IsActiveFlag_OVR(ADC2)) return 0xFFFF;
  return result;
}
static void ph_slow_sensors(void) {
  static uint32_t last_bus,last_temp;
  uint32_t now=HAL_GetTick();
  if((uint32_t)(now-last_bus)>=1) {
    last_bus=now;uint16_t raw=ph_adc2_read(LL_ADC_CHANNEL_0);
    s_bus_adc=raw;
    if(raw>4095) {ph_safe_stop(CFF_FAULT_ADC);return;}
    float v=(float)raw*(36.3f/4095.0f);
    s_vbus=v>1?v:1;g_ph_vbus=(uint16_t)(v*1000);
    if(v<s_cfg.bus_uv) ph_safe_stop(CFF_FAULT_UNDERVOLTAGE);
    else if(v>s_cfg.bus_uv_recover) g_ph_fault&=(uint16_t)~CFF_FAULT_UNDERVOLTAGE;
    if(v>=s_cfg.bus_ov) ph_safe_stop(CFF_FAULT_OVERVOLTAGE);
  }
  if((uint32_t)(now-last_temp)>=20) {
    last_temp=now;
    uint16_t raw=0;
    float fet=NAN,motor=NAN,scale=1;
    if(s_cfg.fet_temp_enable) {
      raw=ph_adc2_read(LL_ADC_CHANNEL_1);s_fet_adc=raw;
      fet=cff_ntc_temperature(raw,10000,3450);
      if(isfinite(fet)&&fet>-30&&fet<150) {
        scale=cff_temperature_derating(fet,s_cfg.fet_temp_lower,s_cfg.fet_temp_upper);
      } else {scale=0;ph_safe_stop(CFF_FAULT_FET_TEMP);}
    }
    s_fet_temp=fet;
    if(s_cfg.motor_temp_enable) {
      raw=ph_adc2_read(LL_ADC_CHANNEL_2);motor=cff_ntc_temperature(raw,s_cfg.motor_ntc_r25,s_cfg.motor_ntc_beta);
      if(!isfinite(motor)||motor<-30||motor>200) {scale=0;ph_safe_stop(CFF_FAULT_MOTOR_TEMP);}
      else {scale=fminf(scale,cff_temperature_derating(motor,s_cfg.motor_temp_lower,s_cfg.motor_temp_upper));
        if(motor>=s_cfg.motor_temp_upper) ph_safe_stop(CFF_FAULT_MOTOR_TEMP);}
    }
    s_motor_temp=motor;s_thermal_scale=scale;
    if(isfinite(fet)&&fet>=s_cfg.fet_temp_upper) ph_safe_stop(CFF_FAULT_FET_TEMP);
  }
}

/* ---- FOC core: Clarke/Park -> Id/Iq PI -> inverse Park -> SVPWM -------- */
static void ph_foc_run(float elec, float id_ref, float iq_ref,
                       float iA, float iB, float iC, uint16_t *pulse)
{
  float output_elec = elec;
  if (g_ph_state == CFF_AXIS_CLOSED_LOOP && g_ph_if_hz == 0u) {
    float age = (float)(ph_clock_now() - s_enc_cycle) / PH_FCPU + s_enc_delay_us * 0.000001f;
    s_enc_age_used = age;
    float omega = s_vel * (float)s_cfg.encoder_dir * (float)s_cfg.pole_pairs;
    /* Conditioned currents lag ~50 us. PWM preload takes one period and
       its mean actuation time is another half-period later. */
    output_elec = ph_wrap(elec + omega * (age + 0.0000375f));
    elec = ph_wrap(elec + omega * (age - 0.0000500f));
  }
  elec = ph_wrap(elec);
  s_elec_used = elec;
  s_pwm_elec_used = ph_wrap(output_elec);
  float ialpha, ibeta;
  ph_clarke_three(iA, iB, iC, &ialpha, &ibeta);
  float cs = ph_cos(elec), sn = ph_sin(elec);
  float idm =  ialpha * cs + ibeta * sn;
  float iqm = -ialpha * sn + ibeta * cs;

  float lim = (g_ph_state == CFF_AXIS_CALIBRATION ? PH_CAL_CURRENT_LIMIT_A : s_cfg.current_limit)*s_thermal_scale;
  float idc = id_ref, iqc = iq_ref;
  if (idc >  lim) { idc =  lim; } else if (idc < -lim) { idc = -lim; }
  if (iqc >  lim) { iqc =  lim; } else if (iqc < -lim) { iqc = -lim; }
  s_id_target = idc; s_iq_target = iqc;

  float dt = 1.0f / (float)PH_ISR_HZ;
  float kp = s_cfg.current_gain_p, ki = s_cfg.current_gain_i;
  float ed = idc - idm;
  float eq = iqc - iqm;
  float vd, vq;
  float vmax = ph_svpwm_voltage_limit(s_vbus, PH_PWM_PERIOD + 1u);
  if (g_ph_state == CFF_AXIS_CALIBRATION && vmax > PH_CAL_MAX_VOLTAGE) { vmax = PH_CAL_MAX_VOLTAGE; }
  float ffd=0,ffq=0;
  if(g_ph_state==CFF_AXIS_CLOSED_LOOP&&g_ph_if_hz==0&&s_cfg.model_ff_enable) {
    float omega=s_vel*(float)s_cfg.encoder_dir*(float)s_cfg.pole_pairs;
    float flux=s_cfg.torque_constant/(1.5f*(float)s_cfg.pole_pairs);
    ffd=s_cfg.phase_resistance*idc-omega*s_cfg.phase_inductance*iqm;
    ffq=s_cfg.phase_resistance*iqc+omega*(s_cfg.phase_inductance*idm+flux);
  }
  ph_current_pi_ff_step(ed,eq,kp,ki,dt,vmax,ffd,ffq,&s_id_int,&s_iq_int,&vd,&vq);
  if (vd*vd + vq*vq >= 0.999f * vmax*vmax) { s_voltage_saturation_count++; }

  s_id_out = idm; s_iq_out = iqm;
  float power=1.5f*(vd*idm+vq*iqm);
  s_power+=0.01f*(power-s_power); s_bus_current=s_power/fmaxf(s_vbus,1);

  float output_cs = ph_cos(output_elec), output_sn = ph_sin(output_elec);
  float valpha = vd * output_cs - vq * output_sn;
  float vbeta  = vd * output_sn + vq * output_cs;
  float va = valpha;
  float vb = -0.5f * valpha + PH_SQRT3_2 * vbeta;
  float vc = -0.5f * valpha - PH_SQRT3_2 * vbeta;
  unsigned da, db, dc, sample_q, sample_mask;
  float applied = ph_svpwm_duty_sample(va,vb,vc,s_vbus,PH_PWM_PERIOD+1u,0u,
                                       &da,&db,&dc,&sample_q,&sample_mask);
  s_pwm_sample_q=(uint16_t)sample_q;
  s_pwm_sample_mask=(uint8_t)sample_mask;
  s_pwm_applied_scale=applied;
  /* Back-calculate the current PI integrators from the voltage the clipped
     phase compares can actually apply. This avoids windup at the hexagon rail. */
  float pva=((float)da/(float)(PH_PWM_PERIOD+1u)-0.5f)*s_vbus;
  float pvb=((float)db/(float)(PH_PWM_PERIOD+1u)-0.5f)*s_vbus;
  float pvc=((float)dc/(float)(PH_PWM_PERIOD+1u)-0.5f)*s_vbus;
  float pa=(2.0f*pva-pvb-pvc)/3.0f, pb=(pvb-pvc)*0.57735026919f;
  float actual_vd=pa*output_cs+pb*output_sn;
  float actual_vq=-pa*output_sn+pb*output_cs;
  s_id_int=actual_vd-kp*ed-ffd;
  s_iq_int=actual_vq-kp*eq-ffq;
  s_vd_out=actual_vd; s_vq_out=actual_vq; s_voltage_limit=vmax;
  pulse[0] = (uint16_t)da; pulse[1] = (uint16_t)db; pulse[2] = (uint16_t)dc;
}

/* ---- 40 kHz cycle: ADC+DMA read, FOC/SVPWM, OC trip, decimated frame ----- */
static float ph_cal_l_trimmed_mean(void)
{
  float sorted[PH_CAL_L_REPEATS];
  unsigned n=s_cal_l_valid_count;
  if(n>PH_CAL_L_REPEATS) n=PH_CAL_L_REPEATS;
  for(unsigned i=0u;i<n;i++) {
    float value=s_cal_l_samples[i]; unsigned j=i;
    while(j>0u && sorted[j-1u]>value) { sorted[j]=sorted[j-1u]; j--; }
    sorted[j]=value;
  }
  unsigned trim=n/4u, first=trim, last=n-trim;
  if(n<8u || last<=first) return 0.0f;
  float sum=0.0f;
  for(unsigned i=first;i<last;i++) sum+=sorted[i];
  return sum/(float)(last-first);
}

static void ph_isr(void)
{
  uint32_t t0 = ph_clock_now();
  g_ph_cnt_upd = (uint16_t)TIM1->CNT;   /* counter position of the sample */

  /* TIM1 update triggers all three injected samples before ISR dispatch. */
  uint8_t adc_valid=ph_adc_read_injected(s_adc_dma);
  if(adc_valid) { s_adc_missed_streak=0u; s_last_good_sample_mask=s_pwm_sample_mask; }
  else if(s_adc_missed_streak<255u) { s_adc_missed_streak++; }
  if(s_adc_missed_streak>=3u) ph_safe_stop(CFF_FAULT_ADC);
  g_ph_adc_wait = ph_clock_now() - t0;
  g_ph_cnt_rd = (uint16_t)TIM1->CNT;    /* counter position after the read */
  if ((g_ph_state != CFF_AXIS_IDLE || g_ph_mod_milli != 0u) &&
      (uint32_t)(ph_clock_now() - s_command_cycle) > (s_command_source==2?14400000u:PH_HOST_TIMEOUT_CYCLES)) {
    ph_safe_stop(CFF_FAULT_COMM);
  }
  uint8_t active_sample = (TIM1->BDTR & TIM_BDTR_MOE) != 0u;
  if (g_ph_state == CFF_AXIS_CLOSED_LOOP) {
    float measured_speed=s_enc2_enabled?s_enc2_vel:s_vel;
    float speed = fabsf(measured_speed);
    if (speed > s_active_speed_peak) { s_active_speed_peak = speed; }
    if(s_enc2_enabled&&!s_enc2_ok) ph_safe_stop(CFF_FAULT_ENCODER);
  }
  if (active_sample) {
    if (g_ph_adc_wait > s_active_adc_max) { s_active_adc_max = g_ph_adc_wait; }
    s_active_entry = g_ph_cnt_upd; s_active_end = g_ph_cnt_rd;
  }

  uint16_t ia = s_adc_dma[0];   /* IN4 = phase A */
  uint16_t ib = s_adc_dma[1];   /* IN5 = phase B */
  uint16_t ic = s_adc_dma[2];   /* IN6 = phase C */
  unsigned sample_mask = adc_valid?s_pwm_sample_mask:s_last_good_sample_mask;
  if (!active_sample) { sample_mask = 7u; }

  /* Measure final zero in the SAME hardware-triggered injected sequence used
     in operation, with gates off. The first eight scans are discarded. */
  if (adc_valid && s_zero_remaining && !active_sample && !(g_ph_fault & CFF_FAULT_ADC)) {
    if (s_zero_remaining <= 400u) {
      s_zero_acc[0] += ia; s_zero_acc[1] += ib; s_zero_acc[2] += ic;
    }
    if (--s_zero_remaining == 0u) {
      for (unsigned i = 0u; i < 3u; i++) { s_i_off[i] = (float)s_zero_acc[i] / 400.0f; }
      s_current_filter.initialized = 0u;
    }
  }

  /* --- overcurrent trip (every period): use only shunts proven quiet by the
     PWM scheduler, then reconstruct the third inline phase with KCL. --- */
  float raw_current[3] = {
    ((float)ia-s_i_off[0])*PH_A_PER_LSB,
    ((float)ib-s_i_off[1])*PH_A_PER_LSB,
    ((float)ic-s_i_off[2])*PH_A_PER_LSB
  };
  if (active_sample && __builtin_popcount(sample_mask & 7u) < 2) {
    ph_safe_stop(CFF_FAULT_ADC);
  }
  (void)ph_reconstruct_two_currents(raw_current,sample_mask & 7u);
  float peak_a=fmaxf(fabsf(raw_current[0]),fmaxf(fabsf(raw_current[1]),fabsf(raw_current[2])));
  uint16_t pk=(uint16_t)fminf(65535.0f,peak_a/PH_A_PER_LSB);
  g_ph_peak = pk;
  if (pk > g_ph_spike_max) { g_ph_spike_max = pk; }
  if (pk > PH_SPIKE_LVL)
  {
    g_ph_spike++;
    g_ph_spike_raw[0] = ia; g_ph_spike_raw[1] = ib; g_ph_spike_raw[2] = ic;
  }
  if (peak_a > (float)PH_OC_ADC*PH_A_PER_LSB)
  {
    if (g_ph_oc_count < 60000u) { g_ph_oc_count++; }
    if ((g_ph_trip == 0u) && (g_ph_oc_count >= PH_OC_N)) { g_ph_trip = 1u; }
  }
  else
  {
    g_ph_oc_count = 0u;
  }
  if (g_ph_trip != 0u) {
    ph_safe_stop(CFF_FAULT_OVERCURRENT);
  }
  /* Keep all raw ADC ranks for diagnostics; only reconstructed/quiet ranks
     enter the conditioned-current path. */
  for (uint32_t i = 0u; i < 3u; i++) {
    s_current_raw[i] = raw_current[i];
  }
  if (g_ph_fault & CFF_FAULT_ADC) {
    s_current_filter.initialized = 0u;
  } else {
    s_sample_flags = (uint16_t)(ph_current_filter_step(&s_current_filter, s_current_raw) | 0x100u |
                               ((sample_mask&7u)<<12) | (adc_valid?0u:0x200u));
    if (s_sample_flags & 7u) { s_impulses++; }
  }
  /* From here onward ia/ib/ic are conditioned amperes, not ADC counts. */
  float iA = s_current_filter.output[0], iB = s_current_filter.output[1], iC = s_current_filter.output[2];
  if (s_trace_state == 1u || s_trace_state == 2u) {
    uint16_t index = s_trace_next;
    volatile ph_current_trace_t *trace = &s_current_trace[index];
    for (uint32_t i = 0u; i < 3u; i++) {
      trace->adc[i] = s_adc_dma[i]; trace->conditioned[i] = s_current_filter.output[i];
    }
    trace->flags = s_sample_flags;
    trace->start = g_ph_cnt_upd; trace->end = g_ph_cnt_rd;
    uint32_t min_ccr = TIM1->CCR1;
    if (TIM1->CCR2 < min_ccr) { min_ccr = TIM1->CCR2; }
    if (TIM1->CCR3 < min_ccr) { min_ccr = TIM1->CCR3; }
    trace->min_ccr = (uint16_t)min_ccr;
    trace->direction = (TIM1->CR1 & TIM_CR1_DIR) != 0u;
    s_trace_next = (uint16_t)((index + 1u) & 63u);
    if (s_trace_state == 1u && pk > PH_SPIKE_LVL) {
      s_trace_trigger = index; s_trace_post = 16u; s_trace_state = 2u;
    } else if (s_trace_state == 2u && --s_trace_post == 0u) {
      __DMB(); s_trace_state = 3u;
    }
  }
  if (g_ph_state == CFF_AXIS_CALIBRATION) {
    if ((float)pk * PH_A_PER_LSB > PH_CAL_RAW_TRIP_A) { s_cal_oc_count++; }
    else { s_cal_oc_count = 0u; }
    if (s_cal_oc_count >= PH_OC_N) { ph_safe_stop(CFF_FAULT_OVERCURRENT); }
  }

  if ((g_ph_state == CFF_AXIS_CALIBRATION) ||
      (g_ph_state == CFF_AXIS_MEASURE) ||
      ((g_ph_state == CFF_AXIS_CLOSED_LOOP) && (g_ph_if_hz == 0u))) {
    if ((!s_enc_seen) || (!g_ph_enc_ok) ||
        ((uint32_t)(ph_clock_now() - s_enc_cycle) > PH_ENC_MAX_AGE_CYCLES)) {
      g_ph_fault |= CFF_FAULT_ENCODER;
      g_ph_state = CFF_AXIS_IDLE;
      s_calibrated = 0u;
    }
  }
  if (g_ph_fault != 0u) { g_ph_state = CFF_AXIS_IDLE; g_ph_mod_milli = 0u; }

  if ((g_ph_state == CFF_AXIS_CLOSED_LOOP) && (g_ph_if_hz == 0u) &&
      (s_cfg.pole_pairs == 0u)) {
    g_ph_fault |= CFF_FAULT_SPINOUT; g_ph_state = CFF_AXIS_IDLE;
  }

  /* --- control: FOC (closed loop) / calibration / bench V/f / idle --- */
  uint16_t pulse[3];
  if (g_ph_state == CFF_AXIS_CLOSED_LOOP)
  {
    if(++s_motion_div>=40u) {
      s_motion_div=0;
      float position=s_enc2_enabled?(s_enc2_dir*s_enc2_pos-s_enc2_offset):
                     ((float)s_cfg.encoder_dir*s_pos-s_cfg.pos_zero);
      float velocity=s_enc2_enabled?(s_enc2_dir*s_enc2_vel):((float)s_cfg.encoder_dir*s_vel);
      float limit=s_cfg.current_limit*s_thermal_scale;
      if(!s_motion.initialized) cff_motion_reset(&s_motion,position,velocity);
      if(s_cfg.pos_limits_enable&&(position<s_cfg.pos_min-0.05f||position>s_cfg.pos_max+0.05f)) ph_safe_stop(CFF_FAULT_POSITION);
      if(s_command_source==2) {
        cff_mit_command_t m={s_mit_command.position_rad,s_mit_command.velocity_rad_s,
                             s_mit_command.kp_a_per_rad,s_mit_command.kd_a_per_rad_s,
                             s_mit_command.iq_ff_A};
        float can_position=position-s_can_position_zero;
        float requested_current=m.kp_a_per_rad*(m.position_rad-can_position)+
                                m.kd_a_per_rad_s*(m.velocity_rad_s-velocity)+m.iq_ff_A;
        s_can_current=cff_clamp(requested_current,-limit,limit);
        s_iq_ref=s_can_current;
      } else {
        cff_config_t motion_cfg=s_cfg;
        /* With no healthy output encoder, velocity mode regulates the motor
           encoder directly. In output-axis mode keep its speed ceiling. */
        if(s_enc2_enabled && motion_cfg.limit_vel>CFF_AXIS_MAX_RAD_S)
          motion_cfg.limit_vel=CFF_AXIS_MAX_RAD_S;
        s_iq_ref=cff_motion_step(&s_motion,&motion_cfg,position,velocity,limit,0.001f);
      }
      /* Cogging compensation disabled: the learned map can inject a periodic
         torque/current command and obscure current-loop noise diagnostics. */
      s_cogging_current_applied = 0.0f;
      /* A non-regenerative supply must not receive sustained braking torque.
         This is an estimate-based soft limit, not a physical energy sink. */
      if(s_cfg.regen_current==0&&fabsf(velocity)>0.5f&&s_iq_ref*velocity<0) s_iq_ref=0;
      if(s_cfg.input_power>0&&s_power>s_cfg.input_power) s_iq_ref*=cff_clamp(s_cfg.input_power/fmaxf(s_power,1),0,1);
      if(s_bus_current < -s_cfg.regen_current-0.05f && s_iq_ref*velocity<0) s_iq_ref*=0.5f;
      s_iq_ref=cff_clamp(s_iq_ref,-limit,limit);
    }
    float elec;
    if (g_ph_if_hz != 0u)             /* open-loop i/f (no encoder) */
    {
      s_if_ang += PH_TWO_PI * ((float)g_ph_if_hz * 0.1f) / (float)PH_ISR_HZ;
      if (s_if_ang >= PH_TWO_PI) { s_if_ang -= PH_TWO_PI; }
      elec = s_if_ang;
    }
    else
    {
      elec = ph_wrap(s_observer_mech * (float)s_cfg.encoder_dir * (float)s_cfg.pole_pairs + s_cfg.encoder_offset);
    }
    ph_foc_run(elec, 0.0f, s_iq_ref, iA, iB, iC, pulse);
  }
  else if (g_ph_state == CFF_AXIS_CALIBRATION)
  {
    uint32_t t = s_cal_cnt++;
    if (s_cal_ph == 0u)                    /* ---- R (dc, current loop) ---- */
    {
      ph_foc_run(0.0f, ph_cal_r_target(t), 0.0f, iA, iB, iC, pulse);
      uint32_t local = t % PH_CAL_R_LEVEL_TICKS;
      if (local >= 12000u) { s_cal_a += s_vd_out; s_cal_b += s_id_out; s_cal_n++; }
      if (local == PH_CAL_R_LEVEL_TICKS - 1u)
      {
        float idm = s_cal_n ? (s_cal_b / (float)s_cal_n) : 0.0f;
        float vdm = s_cal_n ? (s_cal_a / (float)s_cal_n) : 0.0f;
        if (t < PH_CAL_R_LEVEL_TICKS) {
          s_cal_r_low_v = vdm; s_cal_r_low_i = idm;
        } else {
          float resistance = 0.0f;
          if (!ph_cal_fit_r(s_cal_r_low_v, s_cal_r_low_i, vdm, idm,
                            &resistance, &s_cal_voltage_error)) {
            ph_safe_stop(CFF_FAULT_CALIBRATION);
          } else {
            s_cfg.phase_resistance = resistance;
            /* Use a fixed 16-cycle pulse so each trial contains enough
               current span for a discrete RL fit; 1 V and the measured R
               keep the steady current below the 6 A calibration clamp. */
            s_cal_l_pulse_ticks = PH_CAL_L_PULSE_MAX_TICKS;
            s_cal_l_valid_count = 0u;
            s_cal_ph = 1u; s_cal_cnt = 0u;
          }
        }
        s_cal_a = 0.0f; s_cal_b = 0.0f; s_cal_n = 0u;
      }
    }
    else if (s_cal_ph == 1u) /* L: repeated alternating pulses, discrete RL fit, trimmed mean */
    {
      if (t < PH_CAL_L_START) {
        ph_foc_run(0.0f, 0.0f, 0.0f, iA, iB, iC, pulse);
      } else {
        const float vtest = 1.0f;
        uint32_t elapsed = t - PH_CAL_L_START;
        uint32_t burst = elapsed / PH_CAL_L_CYCLE_TICKS;
        uint32_t pulse_tick = elapsed % PH_CAL_L_CYCLE_TICKS;
        if (burst < PH_CAL_L_REPEATS && pulse_tick < s_cal_l_pulse_ticks) {
          int polarity = (burst & 1u) ? -1 : 1;
          unsigned da,db,dc,sample_q,sample_mask;
          float vphase=(float)polarity*vtest;
          (void)ph_svpwm_duty_sample(vphase,-0.5f*vphase,-0.5f*vphase,s_vbus,
                                     PH_PWM_PERIOD+1u,0u,&da,&db,&dc,&sample_q,&sample_mask);
          s_pwm_sample_q=(uint16_t)sample_q;
          s_pwm_sample_mask=(uint8_t)sample_mask;
          pulse[0]=(uint16_t)da; pulse[1]=(uint16_t)db; pulse[2]=(uint16_t)dc;
          if (pulse_tick == 0u) {
            s_cal_l_sample_count = 0u;
            s_cal_l_sum_x = s_cal_l_sum_y = s_cal_l_sum_xx = s_cal_l_sum_xy = 0.0f;
          } else {
            /* raw_current[0] already uses the active PWM quiet-phase mask and
               reconstructs phase A from B/C when A is in its switching edge. */
            float signed_i = (float)polarity * raw_current[0];
            unsigned n = s_cal_l_sample_count;
            if (n < PH_CAL_L_PULSE_MAX_TICKS) {
              if (n > 0u) {
                float x=s_cal_l_pulse_i[n-1u], y=signed_i;
                s_cal_l_sum_x += x; s_cal_l_sum_y += y;
                s_cal_l_sum_xx += x*x; s_cal_l_sum_xy += x*y;
              }
              s_cal_l_pulse_i[n]=signed_i;
              s_cal_l_sample_count++;
            }
            if (pulse_tick == (uint32_t)s_cal_l_pulse_ticks - 1u) {
              float pairs = s_cal_l_sample_count > 1u ? (float)(s_cal_l_sample_count-1u) : 0.0f;
              float denom = pairs*s_cal_l_sum_xx - s_cal_l_sum_x*s_cal_l_sum_x;
              float a = denom > 0.0f ? (pairs*s_cal_l_sum_xy - s_cal_l_sum_x*s_cal_l_sum_y) / denom : 0.0f;
              /* For RL: I[k+1]=a*I[k]+b, a=exp(-R/(L*fs)).
                 Fitting a rather than V/L makes the result insensitive to
                 the inverter's nearly constant dead-time/device drop. */
              float estimate = (a > 0.0f && a < 1.0f && s_cfg.phase_resistance > 0.0f) ?
                               -s_cfg.phase_resistance/((float)PH_ISR_HZ*logf(a)) : 0.0f;
              s_cal_l_last_a=a; s_cal_l_last_estimate=estimate;
              if (pairs >= 8.0f && a > 0.05f && a < 0.9995f && isfinite(estimate) && estimate > 1.0e-7f && estimate < 0.1f && s_cal_l_valid_count < PH_CAL_L_REPEATS) {
                s_cal_l_samples[s_cal_l_valid_count++] = estimate;
              }
            }
          }
        } else {
          ph_foc_run(0.0f, 0.0f, 0.0f, iA, iB, iC, pulse);
        }
        if (t >= PH_CAL_L_TICKS - 1u) {
          float estimate=ph_cal_l_trimmed_mean();
          if (!(estimate > 1.0e-7f) || !isfinite(estimate) || s_cal_l_valid_count < 12u) {
            ph_safe_stop(CFF_FAULT_CALIBRATION);
          } else {
            s_cfg.phase_inductance=estimate;
            s_cal_ph = 2u; s_cal_cnt = 0u; s_if_ang = 0.0f;
            s_id_int = 0.0f; s_iq_int = 0.0f;
            uint16_t half = (uint16_t)((PH_PWM_PERIOD + 1u) / 2u);
            pulse[0] = half; pulse[1] = half; pulse[2] = half;
          }
        }
      }
    }
    else if (s_cal_ph == 2u)               /* ---- spin-lock (ramp 0->2 Hz) ---- */
    {
      float f = 2.0f * (float)t / 24000.0f;
      if (f > 2.0f) { f = 2.0f; }
      s_if_ang += PH_TWO_PI * f / (float)PH_ISR_HZ;
      if (s_if_ang >= PH_TWO_PI) { s_if_ang -= PH_TWO_PI; }
      float ramp = (float)t / 8000.0f;
      if (ramp > 1.0f) { ramp = 1.0f; }
      ph_foc_run(s_if_ang, PH_CAL_CURRENT_A * ramp, 0.0f, iA, iB, iC, pulse);
      if (t >= PH_CAL_LOCK_TICKS - 1u) { s_cal_ph = 3u; s_cal_cnt = 0u; s_cal_a = s_mech; s_cal_mechacc = 0.0f; }
    }
    else if (s_cal_ph == 3u)               /* ---- pole-pairs (16 electrical turns; suppress endpoint cogging error) ---- */
    {
      const float f = 2.0f;
      s_if_ang += PH_TWO_PI * f / (float)PH_ISR_HZ;
      if (s_if_ang >= PH_TWO_PI) { s_if_ang -= PH_TWO_PI; }
      ph_foc_run(s_if_ang, s_cfg.calibration_current, 0.0f, iA, iB, iC, pulse);
      if ((t & 0x3Fu) == 0u)
      {
        float dm = s_mech - s_cal_a;
        if (dm >  PH_PI) { dm -= PH_TWO_PI; }
        if (dm < -PH_PI) { dm += PH_TWO_PI; }
        s_cal_mechacc += dm; s_cal_a = s_mech;
      }
      if (t >= PH_CAL_PP_TICKS - 1u)
      {
        float elec_adv = (float)PH_CAL_PP_TICKS / (float)PH_ISR_HZ * PH_TWO_PI * f;
        float mech_adv = s_cal_mechacc;
        if (mech_adv < 0.0f) { s_cal_dir = -1; mech_adv = -mech_adv; }
        else                 { s_cal_dir = 1; }
        if (mech_adv > 0.05f)
        {
          uint16_t spp = (uint16_t)((elec_adv / mech_adv) + 0.5f);
          if ((spp >= 1u) && (spp <= 64u)) { s_cal_pp = spp; }
        }
        if (s_cal_pp == 0u) {
          g_ph_fault |= CFF_FAULT_SPINOUT; g_ph_state = CFF_AXIS_IDLE;
        }
        s_cal_ph = 4u; s_cal_cnt = 0u; s_cal_sin = 0.0f; s_cal_cos = 0.0f; s_cal_n = 0u;
      }
    }
    else if (s_cal_ph == 4u) /* static positive Id: rotor d-axis = commanded angle */
    {
      ph_foc_run(s_if_ang, s_cfg.calibration_current, 0.0f, iA, iB, iC, pulse);
      /* 0.5 s settle, then 0.3 s of NEW encoder readings. */
      if ((t >= PH_CAL_STATIC_SETTLE) && (g_enc_count != s_cal_enc_count)) {
        s_cal_enc_count = g_enc_count;
        float d = s_if_ang - (float)s_cal_dir * s_mech * (float)s_cal_pp;
        s_cal_sin += sinf(d); s_cal_cos += cosf(d); s_cal_n++;
      }
      if (t >= PH_CAL_STATIC_TICKS - 1u) {
        s_cal_quality = s_cal_n ? sqrtf(s_cal_sin*s_cal_sin + s_cal_cos*s_cal_cos) / (float)s_cal_n : 0.0f;
        if ((s_cal_n < 100u) || (s_cal_quality < 0.98f)) {
          g_ph_fault |= CFF_FAULT_SPINOUT; g_ph_state = CFF_AXIS_IDLE;
        } else {
          s_cal_offset = ph_wrap(atan2f(s_cal_sin, s_cal_cos));
          s_cal_ph = 5u; s_cal_cnt = 0u;
          s_cal_sin = 0.0f; s_cal_cos = 0.0f; s_cal_n = 0u;
        }
      }
    }
    else if (s_cal_ph == 5u) /* verify mapping with forward/reverse d-axis sweep */
    {
      const float f = (t < PH_CAL_SWEEP_LEG_TICKS) ? 1.0f : -1.0f; /* 2 electrical turns each way */
      s_if_ang = ph_wrap(s_if_ang + PH_TWO_PI * f / (float)PH_ISR_HZ);
      ph_foc_run(s_if_ang, s_cfg.calibration_current, 0.0f, iA, iB, iC, pulse);
      /* Ignore acceleration/reversal settling; verify each direction separately. */
      uint32_t leg_t = t % PH_CAL_SWEEP_LEG_TICKS;
      if ((leg_t >= PH_CAL_SWEEP_SETTLE) && (g_enc_count != s_cal_enc_count)) {
        s_cal_enc_count = g_enc_count;
        float d = s_if_ang - ((float)s_cal_dir * s_mech * (float)s_cal_pp + s_cal_offset);
        s_cal_sin += sinf(d); s_cal_cos += cosf(d); s_cal_n++;
      }
      if (t == PH_CAL_SWEEP_LEG_TICKS - 1u) {
        s_cal_quality = s_cal_n ? sqrtf(s_cal_sin*s_cal_sin + s_cal_cos*s_cal_cos) / (float)s_cal_n : 0.0f;
        float error = atan2f(s_cal_sin, s_cal_cos);
        if ((s_cal_n < 100u) || (s_cal_quality < 0.95f) || (fabsf(error) > 1.0f)) {
          g_ph_fault |= CFF_FAULT_SPINOUT; g_ph_state = CFF_AXIS_IDLE;
        } else {
          s_cal_forward_error = error;
          s_cal_sin = 0.0f; s_cal_cos = 0.0f; s_cal_n = 0u;
        }
      } else if (t >= PH_CAL_SWEEP_TICKS - 1u) {
        /* The final circular statistics and sin/cos/atan2 sequence exceed
           one 25 kHz period on the STM32C562. Stop PWM here and finish the
           non-time-critical math from phase_test_task(). */
        s_cal_finalize_pending = 1u;
        __DMB();
        ph_safe_stop(0u);
      }
    }
    else if (s_cal_ph == 6u) /* 5 A d-axis frequency response; ideally stationary, alignment error may cause slight motion. */
    {
      if (s_bw_step >= PH_CAL_BW_TEST_COUNT) {
        s_cal_finalize_pending=2u; __DMB(); ph_safe_stop(0u);
      } else {
        uint32_t bt=s_bw_tick++;
      if(bt==0u) s_bw_sat_base=s_voltage_saturation_count;
        const unsigned frequency=ph_cal_bw_frequency(s_bw_step);
        const unsigned settle=ph_cal_bw_settle_ticks(PH_CAL_ISR,s_bw_step);
        const unsigned measure=ph_cal_bw_measure_ticks(PH_CAL_ISR,s_bw_step);
        float phase=PH_TWO_PI*(float)frequency*(float)bt/(float)PH_CAL_ISR;
        float sn=ph_sin(phase), cs=ph_cos(phase);
        float elec=ph_wrap(s_observer_mech*(float)s_cfg.encoder_dir*(float)s_cfg.pole_pairs+s_cfg.encoder_offset);
        ph_foc_run(elec,s_bw_command[s_bw_step]*sn,0.0f,iA,iB,iC,pulse);
        if(bt>=settle) {
          s_bw_sum_sin[s_bw_step]+=s_id_out*sn;
          s_bw_sum_cos[s_bw_step]+=s_id_out*cs;
          if(s_bw_samples[s_bw_step]<UINT16_MAX) s_bw_samples[s_bw_step]++;
        }
        if(bt+1u>=settle+measure) {
          unsigned n=s_bw_samples[s_bw_step];
          s_bw_gain[s_bw_step]=ph_cal_bw_gain(s_bw_sum_sin[s_bw_step],s_bw_sum_cos[s_bw_step],n,
                                               s_bw_command[s_bw_step]);
          s_bw_lag[s_bw_step]=ph_cal_bw_lag_deg(s_bw_sum_sin[s_bw_step],s_bw_sum_cos[s_bw_step]);
          s_bw_saturation[s_bw_step]=(float)(s_voltage_saturation_count-s_bw_sat_base)/(float)measure;
          s_bw_valid[s_bw_step]=(uint8_t)(s_bw_command[s_bw_step]>=0.5f && s_bw_saturation[s_bw_step]<=0.01f &&
                                isfinite(s_bw_gain[s_bw_step]) && s_bw_gain[s_bw_step]<2.0f);
          s_bw_step++; s_bw_tick=0u; s_id_int=0.0f; s_iq_int=0.0f;
        }
      }
    }
    else
    {
      uint16_t half = (uint16_t)((PH_PWM_PERIOD + 1u) / 2u);
      pulse[0] = half; pulse[1] = half; pulse[2] = half;
      g_ph_state = CFF_AXIS_IDLE;
    }
  }
  else if (g_ph_state == CFF_AXIS_MEASURE)
  {
    uint32_t t = s_meas_t++;
    if (t < 20000u)                       /* 0.5 s: align to electrical 0 */
    {
      ph_foc_run(0.0f, s_cfg.calibration_current, 0.0f, iA, iB, iC, pulse);
    }
    else if (t < 40000u)                  /* R: current loop holds id = 0.5 A */
    {
      ph_foc_run(0.0f, 0.5f, 0.0f, iA, iB, iC, pulse);
      if (t > 35000u) { s_meas_iss += s_vd_out; s_meas_first += s_id_out; s_meas_n += 1.0f; }
    }
    else if (t == 40000u)
    {
      float vd = (s_meas_n > 0.0f) ? (s_meas_iss / s_meas_n) : 0.0f;
      float idm = (s_meas_n > 0.0f) ? (s_meas_first / s_meas_n) : 0.0f;
      s_cfg.phase_resistance = (idm > 0.05f) ? (vd / idm) : 0.0f;   /* R = Vd / Id */
      s_meas_iss = 0.0f; s_meas_first = 0.0f; s_meas_n = 0.0f;
      uint16_t half = (uint16_t)((PH_PWM_PERIOD + 1u) / 2u);
      pulse[0] = half; pulse[1] = half; pulse[2] = half;
    }
    else if (t < 44000u)                  /* discharge to zero current */
    {
      ph_foc_run(0.0f, 0.0f, 0.0f, iA, iB, iC, pulse);
    }
    else if (t <= 44004u)                 /* L: open-loop d-axis voltage step */
    {
      const float vtest = 1.0f;
      float idA = ((float)ia - s_i_off[0]) * PH_A_PER_LSB;
      if (t == 44000u) { s_meas_first = idA; }
      if (t == 44004u) { s_meas_did = (idA - s_meas_first) / (4.0f / (float)PH_ISR_HZ); }
      int32_t da = (int32_t)((0.5f + vtest / s_vbus) * (float)(PH_PWM_PERIOD + 1u));
      int32_t db = (int32_t)((0.5f - 0.5f * vtest / s_vbus) * (float)(PH_PWM_PERIOD + 1u));
      pulse[0] = (uint16_t)da; pulse[1] = (uint16_t)db; pulse[2] = (uint16_t)db;
      if (t == 44004u)
      {
        s_cfg.phase_inductance = (s_meas_did > 20.0f) ? (vtest / s_meas_did) : 0.0f;  /* L = V / (di/dt) */
        if (s_cfg.phase_inductance > 1e-7f) { s_cfg.current_gain_p = s_cfg.phase_inductance * 1000.0f; }
        if (s_cfg.phase_resistance > 1e-4f) { s_cfg.current_gain_i = s_cfg.phase_resistance * 1000.0f; }
        s_meas_t = 0u;
        s_id_int = 0.0f; s_iq_int = 0.0f;
        g_ph_state = CFF_AXIS_IDLE;
      }
    }
    else
    {
      uint16_t half = (uint16_t)((PH_PWM_PERIOD + 1u) / 2u);
      pulse[0] = half; pulse[1] = half; pulse[2] = half;
    }
  }
  else if (g_ph_mod_milli > 0u)
  {
    if (++s_ramp_div >= PH_RAMP_ISR)
    {
      s_ramp_div = 0u;
      uint32_t target = (uint32_t)g_ph_freq_deci;
      if (s_f_act < target) { s_f_act++; }
      else if (s_f_act > target) { s_f_act--; }
    }
    float f_hz = (float)s_f_act * 0.1f;
    int32_t mod = (int32_t)((float)PH_VF_MILLI * f_hz);
    if (mod > (int32_t)g_ph_mod_milli) { mod = (int32_t)g_ph_mod_milli; }
    if (mod < 0) { mod = 0; }
    float dth = PH_TWO_PI * f_hz / (float)PH_ISR_HZ;
    s_ang += dth;
    if (s_ang >= PH_TWO_PI) { s_ang -= PH_TWO_PI; }
    uint32_t base = ((uint32_t)(s_ang * (float)PH_LUT_N / PH_TWO_PI)) & (PH_LUT_N - 1u);
    for (uint32_t ph = 0u; ph < 3u; ph++)
    {
      uint32_t idx = (base + (ph * (PH_LUT_N / 3u))) & (PH_LUT_N - 1u);
      int32_t duty = 500 + (mod * (int32_t)s_sin[idx]) / 65534;
      if (duty < 20)  { duty = 20; }
      if (duty > 980) { duty = 980; }
      pulse[ph] = (uint32_t)((duty * ((int32_t)PH_PWM_PERIOD + 1)) / 1000);
      if (pulse[ph] < PH_PWM_SAMPLE_MARGIN) { pulse[ph] = PH_PWM_SAMPLE_MARGIN; }
      if (pulse[ph] > PH_PWM_PERIOD + 1u - PH_PWM_SAMPLE_MARGIN) { pulse[ph] = PH_PWM_PERIOD + 1u - PH_PWM_SAMPLE_MARGIN; }
    }
  }
  else
  {
    uint16_t half = (uint16_t)((PH_PWM_PERIOD + 1u) / 2u);
    pulse[0] = half; pulse[1] = half; pulse[2] = half;
  }
  if (g_ph_fault != 0u) {
    uint16_t half = (uint16_t)((PH_PWM_PERIOD + 1u) / 2u);
    pulse[0] = half; pulse[1] = half; pulse[2] = half;
  }
  TIM1->CCR1 = pulse[0];
  TIM1->CCR2 = pulse[1];
  TIM1->CCR3 = pulse[2];
  TIM1->CCR4 = s_pwm_sample_q;
  if ((g_ph_fault != 0u) ||
      ((g_ph_state == CFF_AXIS_IDLE) && (g_ph_mod_milli == 0u))) {
    TIM1->BDTR &= ~TIM_BDTR_MOE; /* idle means gates OFF, not 50% common-mode PWM */
  } else {
    TIM1->BDTR |= TIM_BDTR_MOE;
  }

  /* --- decimate to the USB stream (100 Hz) --- */
  if ((g_ph_streaming != 0u) && (++s_decim >= s_send_decim))
  {
    s_decim = 0u;
    /* Defer float formatting and diagnostic snapshots to the main loop so
       telemetry cannot consume the 40 kHz current-control deadline. */
    s_telem_duty[0]=pulse[0];s_telem_duty[1]=pulse[1];s_telem_duty[2]=pulse[2];
    __DMB();s_telem_due=1u;
  }

  uint32_t dt = ph_clock_now() - t0;
  g_ph_isr_last = dt;
  s_pwm_count++;
  if(active_sample && dt >= (144000000u / PH_ISR_HZ)) ph_safe_stop(CFF_FAULT_DEADLINE);
  if (dt > g_ph_isr_max) { g_ph_isr_max = dt; }
  if (active_sample && dt > s_active_isr_max) { s_active_isr_max = dt; }
}

/* ---- TLE5012B single-wire SSC read (best-effort probe, half-duplex) ----- */
static uint8_t ph_enc_transfer(uint16_t command, uint16_t *value, uint16_t *safety,
                               uint32_t *sample_cycle, uint32_t cs_pin)
{
  uint8_t cmd[2];
  cmd[0] = (uint8_t)(command >> 8); cmd[1] = (uint8_t)command;
  uint16_t w1 = 0u, w2 = 0u;
  uint8_t  err = 0xFFu;

  for (uint8_t attempt = 0u; attempt < 3u; attempt++)
  {
    uint8_t data[4] = { 0u, 0u, 0u, 0u };
    err = 0u;
    HAL_GPIO_WritePin(HAL_GPIOB, cs_pin, HAL_GPIO_PIN_RESET);
    for (volatile uint32_t d = 0u; d < 16u; d++) { }              /* Cs setup >= 105 ns */
    if (HAL_SPI_Transmit(s_spi, cmd, 2u, 1u) != HAL_OK) { err |= 0x1u; }
    for (volatile uint32_t d = 0u; d < 32u; d++) { } /* direction turnaround >=130 ns */
    *sample_cycle = ph_clock_now(); /* register acquisition, NOT end of USB/main processing */
    if (HAL_SPI_Receive(s_spi, data, 4u, 1u) != HAL_OK) { err |= 0x2u; }
    for (volatile uint32_t d = 0u; d < 16u; d++) { }              /* Cs hold  >= 105 ns */
    HAL_GPIO_WritePin(HAL_GPIOB, cs_pin, HAL_GPIO_PIN_SET);
    w1 = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    w2 = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
    if (ph_enc_crc(command, w1) != (uint8_t)w2) { err |= 0x8u; }
    /* valid = transport + CRC, no system/interface error, valid angle (bits14..12). */
    if ((err == 0u) && ((w2 & 0x7000u) == 0x7000u)) { break; }
    for (volatile uint32_t d = 0u; d < 200u; d++) { }             /* CS inactive >= 600 ns */
  }
  *value = w1; *safety = w2;
  return (err == 0u && (w2 & 0x7000u) != 0x7000u) ? 0x10u : err;
}

static void ph_enc_read(void)
{
  if (s_spi == NULL) { return; }
  static uint8_t mode_applied = 0xFFu;
  if (g_enc_mode != mode_applied) {
    (void)HAL_SPI_SetClockPolarity(s_spi, ((g_enc_mode & 0x2u) != 0u) ?
                                  HAL_SPI_CLOCK_POLARITY_HIGH : HAL_SPI_CLOCK_POLARITY_LOW);
    (void)HAL_SPI_SetClockPhase(s_spi, ((g_enc_mode & 0x1u) != 0u) ?
                               HAL_SPI_CLOCK_PHASE_2_EDGE : HAL_SPI_CLOCK_PHASE_1_EDGE);
    mode_applied = g_enc_mode;
  }
  uint16_t w1 = 0u, w2 = 0u;
  uint32_t measured_cycle = 0u;
  if (!s_enc_config_valid && g_ph_state == CFF_AXIS_IDLE) {
    /* LOCK=1010 for configuration-register access; read only. */
    uint16_t mod1, mod2, safety;
    if (ph_enc_transfer(0xD061u, &mod1, &safety, &measured_cycle, SPI3_CS_ENC_PIN) == 0u &&
        ph_enc_transfer(0xD081u, &mod2, &safety, &measured_cycle, SPI3_CS_ENC_PIN) == 0u) {
      float delay = ph_tle_delay_us(mod1, mod2);
      if (delay >= 0.0f) {
        s_enc_mod1 = mod1; s_enc_mod2 = mod2; s_enc_delay_us = delay;
        __DMB(); s_enc_config_valid = 1u;
      }
    }
  }
  uint8_t err = ph_enc_transfer(g_enc_cmd, &w1, &w2, &measured_cycle, SPI3_CS_ENC_PIN);

  g_enc_status = err;
  g_enc_w1 = w1; g_enc_w2 = w2; /* retain rejected words for diagnosis too */
  if ((err == 0u) && ((w2 & 0x7000u) == 0x7000u))
  {
    float measured = (float)(w1 & 0x7FFFu) * (PH_TWO_PI / 32768.0f);
    float position = s_pos, raw_velocity = 0.0f;
    float dt = 0.0f;
    {
      static uint32_t last_cyc = 0u;
      static float last_mech = 0.0f;
      if (last_cyc != 0u)
      {
        dt = (float)(measured_cycle - last_cyc) / PH_FCPU;
        float dm = measured - last_mech;
        if (dm >  PH_PI) { dm -= PH_TWO_PI; }
        if (dm < -PH_PI) { dm += PH_TWO_PI; }
        position += dm;
        if (dt > 1e-6f) { raw_velocity = dm / dt; }
      }
      last_cyc = measured_cycle; last_mech = measured;
    }
    ph_angle_observer_step(&s_angle_observer, measured, dt, s_cfg.encoder_bandwidth);
    /* FOC must never combine a NEW angle with an OLD timestamp or velocity.
       Keep only the publication stores atomic; SSC transactions stay interruptible. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_enc_angle = (uint16_t)(w1 & 0x7FFFu);
    s_mech = measured; s_pos = position;
    s_observer_mech = s_angle_observer.angle;
    s_vel_raw = raw_velocity; s_vel = s_angle_observer.velocity;
    s_enc_cycle = measured_cycle; s_enc_seen = 1u; g_ph_enc_ok = 1u;
    __DMB();
    __set_PRIMASK(primask);
  }
  else
  {
    g_ph_enc_ok = 0u;
    g_enc_fail++;
  }

  {
    static uint8_t efail = 0u;
    if (g_ph_enc_ok == 0u) { if (efail < 255u) { efail++; } } else { efail = 0u; }
    if ((efail >= 3u) && (g_ph_state == CFF_AXIS_CLOSED_LOOP))
    {
      g_ph_fault |= CFF_FAULT_ENCODER;   /* encoder lost while driving */
      g_ph_state = CFF_AXIS_IDLE;
    }
  }
  g_enc_count++;
}

static void ph_enc2_read(void)
{
  static uint32_t last_cycle;
  static float last_angle;
  static float last_motor_pos,last_output_pos;
  static float accum_motor,accum_output,correlation;
  static uint8_t ratio_tracking;
  if (s_spi == NULL) return;
  uint16_t angle=0,safety=0; uint32_t cycle=0;
  uint8_t err=ph_enc_transfer(g_enc_cmd,&angle,&safety,&cycle,HAL_GPIO_PIN_6);
  if(err || (safety&0x7000u)!=0x7000u) { s_enc2_ok=0; s_enc2_fail++; return; }
  float a=(float)(angle&0x7FFFu)*(PH_TWO_PI/32768.0f);
  float p=s_enc2_pos, vel=0.0f;
  if(last_cycle) {
    float d=a-last_angle; if(d>PH_PI)d-=PH_TWO_PI; if(d<-PH_PI)d+=PH_TWO_PI;
    p+=d; float dt=(float)(cycle-last_cycle)/PH_FCPU; if(dt>1e-6f)vel=d/dt;
  }
  last_cycle=cycle; last_angle=a;
  s_enc2_pos=p; s_enc2_vel=vel; s_enc2_ok=1;
  if(g_ph_enc_ok) {
    if(!ratio_tracking) {
      last_motor_pos=s_pos; last_output_pos=p; ratio_tracking=1u;
      return;
    }
    float dm=(float)s_cfg.encoder_dir*(s_pos-last_motor_pos), dout=p-last_output_pos;
    accum_motor+=fabsf(dm);
    accum_output+=fabsf(dout);
    correlation+=dm*dout;
    last_motor_pos=s_pos; last_output_pos=p;
    if(accum_output>=0.5f && accum_motor>0.01f) {
      s_gear_ratio=accum_motor/accum_output;
      if(fabsf(correlation)>0.01f) s_enc2_dir=(correlation>=0.0f)?1:-1;
    }
  }
}

/* Measure ADC zero with the motor undriven -> per-phase current offsets. */
static void ph_cal_current_offset(void)
{
  if (s_adc == NULL) { return; }
  const uint32_t n = 400u;
  uint32_t acc[3] = { 0u, 0u, 0u };
  for (uint32_t k = 0u; k < n; k++)
  {
    ph_adc_read3(s_adc_dma);
    acc[0] += s_adc_dma[0]; acc[1] += s_adc_dma[1]; acc[2] += s_adc_dma[2];
  }
  s_i_off[0] = (float)acc[0] / (float)n;   /* A = IN4 = slot 0 */
  s_i_off[1] = (float)acc[1] / (float)n;   /* B = IN5 = slot 1 */
  s_i_off[2] = (float)acc[2] / (float)n;   /* C = IN6 = slot 2 */
}

/* ------------------------------------------------------------------------- */

void phase_test_init(void)
{
  ph_clock_init();
  s_cfg=s_defaults;
  s_config_loaded=cff_store_load(&s_cfg,&s_calibrated,&s_cal_quality,s_cogging_map_ma,&s_cogging_valid,
                                 &s_enc2_enabled,&s_enc2_dir,&s_enc2_offset);
  /* Ignore and invalidate any legacy learned map in RAM; it is no longer
     applied and a subsequent parameter save will persist the cleared map. */
  s_cogging_valid=0u;s_cogging_current_applied=0.0f;
  memset(s_cogging_map_ma,0,sizeof(s_cogging_map_ma));
  /* Upgrade known old direct-drive defaults/full-scale speed pairs only. */
  if(s_config_loaded) (void)cff_config_migrate_speed_units(&s_cfg);
  /* Upgrade an existing calibrated 1 kHz PI profile to the measured 1 kHz
     closed-loop target. Preserve all other user-selected bandwidth settings. */
  if(s_config_loaded && s_calibrated &&
     fabsf(s_cfg.current_bandwidth-6283.18530718f)<1.0f &&
     s_cfg.phase_resistance>0.0f && s_cfg.phase_inductance>0.0f) {
    s_cfg.current_bandwidth=CFF_CURRENT_BANDWIDTH_DEFAULT_RAD_S;
    s_cfg.current_gain_p=s_cfg.phase_inductance*s_cfg.current_bandwidth;
    s_cfg.current_gain_i=s_cfg.phase_resistance*s_cfg.current_bandwidth;
  }
  pwm_test_init();
  /* Hold both outputs low in idle. Stop switching before current-zero measurement. */
  TIM1->BDTR |= TIM_BDTR_OSSI | TIM_BDTR_OSSR;
  TIM1->BDTR &= ~TIM_BDTR_MOE;

  hal_tim_handle_t *tim = mx_tim1_gethandle();
  if (tim == NULL) { return; }

  s_adc = mx_adc1_gethandle();
  s_spi = mx_spi3_gethandle();
  /* FPC1 output encoder chip select. It shares SSC clock/data with the motor encoder. */
  HAL_RCC_GPIOB_EnableClock();
  hal_gpio_config_t enc2_cs={0}; enc2_cs.mode=HAL_GPIO_MODE_OUTPUT;
  enc2_cs.speed=HAL_GPIO_SPEED_FREQ_VERY_HIGH; enc2_cs.pull=HAL_GPIO_PULL_UP;
  enc2_cs.output_type=HAL_GPIO_OUTPUT_PUSHPULL; enc2_cs.init_state=HAL_GPIO_PIN_SET;
  if(HAL_GPIO_Init(HAL_GPIOB,HAL_GPIO_PIN_6,&enc2_cs)!=HAL_OK) g_ph_fault|=CFF_FAULT_ENCODER;

  /* ADC2: bus voltage on IN0 (PA0). Generated code left the channel unset. */
  hal_adc_handle_t *adc2 = mx_adc2_gethandle();
  if (adc2 != NULL)
  {
    hal_adc_channel_config_t cc2;
    cc2.group          = HAL_ADC_GROUP_REGULAR;
    cc2.sampling_time  = HAL_ADC_SAMPLING_TIME_48CYCLES;
    cc2.input_mode     = HAL_ADC_IN_SINGLE_ENDED;
    cc2.sequencer_rank = 1u;
    (void)HAL_ADC_SetConfigChannel(adc2, HAL_ADC_CHANNEL_0, &cc2);
    (void)HAL_ADC_SetConfigChannel(adc2, HAL_ADC_CHANNEL_1, &cc2);
    (void)HAL_ADC_SetConfigChannel(adc2, HAL_ADC_CHANNEL_2, &cc2);
    (void)HAL_ADC_Start(adc2);
    if (HAL_ADC_Calibrate(adc2) != HAL_OK) { g_ph_fault |= CFF_FAULT_ADC; }
  }

  for (uint32_t i = 0u; i < PH_LUT_N; i++)
  {
    s_sin[i] = (int16_t)(sinf(PH_TWO_PI * (float)i / (float)PH_LUT_N) * 32767.0f);
  }
  for (uint32_t i = 0u; i < 256u; i++)
  {
    s_sin_f[i] = sinf(PH_TWO_PI * (float)i / 256.0f);
  }

  if (s_adc != NULL)
  {
    hal_adc_reg_config_t rc;
    HAL_ADC_REG_GetConfig(s_adc, &rc);
    rc.trigger_src       = HAL_ADC_REG_TRIG_SOFTWARE;
    rc.sequencer_length  = 1u;   /* one channel per conversion (see ph_adc_read3) */
    rc.sequencer_discont = HAL_ADC_REG_SEQ_DISCONT_DISABLE;
    rc.continuous        = HAL_ADC_REG_CONV_SINGLE;
    (void)HAL_ADC_REG_SetConfig(s_adc, &rc);

    hal_adc_channel_config_t cc;
    cc.group          = HAL_ADC_GROUP_REGULAR;
    cc.sampling_time  = HAL_ADC_SAMPLING_TIME_13CYCLES;
    cc.input_mode     = HAL_ADC_IN_SINGLE_ENDED;
    /* 4 -> 5 -> 6 then a dummy 6, so the real phases are never the last conversion */
    cc.sequencer_rank = 1u;
    (void)HAL_ADC_SetConfigChannel(s_adc, HAL_ADC_CHANNEL_4, &cc);
    cc.sequencer_rank = 2u;
    (void)HAL_ADC_SetConfigChannel(s_adc, HAL_ADC_CHANNEL_5, &cc);
    cc.sequencer_rank = 3u;
    (void)HAL_ADC_SetConfigChannel(s_adc, HAL_ADC_CHANNEL_6, &cc);
    cc.sequencer_rank = 4u;
    (void)HAL_ADC_SetConfigChannel(s_adc, HAL_ADC_CHANNEL_6, &cc);   /* dummy */

    HAL_RCC_LPDMA1_EnableClock();       /* USE_HAL_DMA_CLK_ENABLE_MODEL = NO */
    (void)HAL_DMA_Init(&s_dma, HAL_LPDMA1_CH0);
    hal_dma_direct_xfer_config_t dc;
    dc.request         = HAL_LPDMA1_REQUEST_ADC1;
    dc.direction       = HAL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    dc.src_inc         = HAL_DMA_SRC_ADDR_FIXED;
    dc.dest_inc        = HAL_DMA_DEST_ADDR_INCREMENTED;
    dc.src_data_width  = HAL_DMA_SRC_DATA_WIDTH_HALFWORD;
    dc.dest_data_width = HAL_DMA_DEST_DATA_WIDTH_HALFWORD;
    dc.priority        = HAL_DMA_PRIORITY_HIGH;
    (void)HAL_DMA_SetConfigPeriphDirectXfer(&s_dma, &dc);
    /* DMA not wired to the ADC now: conversions are polled in ph_adc_read3(). */

    (void)HAL_ADC_Start(s_adc);
    if (HAL_ADC_Calibrate(s_adc) != HAL_OK) { g_ph_fault |= CFF_FAULT_ADC; }
  }

  HAL_Delay(5u); /* INA240 output settles with gates off before offset collection */
  ph_cal_current_offset();   /* motor undriven here -> ADC zero */

  if (HAL_ADC_Stop(s_adc) != HAL_OK) { g_ph_fault |= CFF_FAULT_ADC; }
  hal_adc_inj_config_t injected = {
    .trigger_src = HAL_ADC_INJ_TRIG_TIM1_TRGO2,
    .trigger_edge = HAL_ADC_INJ_TRIG_EDGE_RISING_FALLING,
    .sequencer_length = 3u,
    .sequencer_discont = HAL_ADC_INJ_SEQ_DISCONT_DISABLE
  };
  if (HAL_ADC_INJ_SetConfig(s_adc, &injected) != HAL_OK) { g_ph_fault |= CFF_FAULT_ADC; }
  hal_adc_channel_config_t injected_channel = {
    .group = HAL_ADC_GROUP_INJECTED, .sampling_time = HAL_ADC_SAMPLING_TIME_13CYCLES,
    .input_mode = HAL_ADC_IN_SINGLE_ENDED, .sequencer_rank = 1u
  };
  if (HAL_ADC_SetConfigChannel(s_adc, HAL_ADC_CHANNEL_4, &injected_channel) != HAL_OK) { g_ph_fault |= CFF_FAULT_ADC; }
  injected_channel.sequencer_rank = 2u;
  if (HAL_ADC_SetConfigChannel(s_adc, HAL_ADC_CHANNEL_5, &injected_channel) != HAL_OK) { g_ph_fault |= CFF_FAULT_ADC; }
  injected_channel.sequencer_rank = 3u;
  if (HAL_ADC_SetConfigChannel(s_adc, HAL_ADC_CHANNEL_6, &injected_channel) != HAL_OK) { g_ph_fault |= CFF_FAULT_ADC; }

  (void)HAL_TIM_SetRepetitionCounter(tim, 1u);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);
  /* CH4 is an internal compare only. Capture both OC4REF edges: the PWM timer
     can issue its repetition-count update at either extremum after reset, and
     both scans finish before the following extremum. */
  LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_SetCompareCH4(TIM1, (PH_PWM_PERIOD + 1u) / 2u);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_OC4);
  if (HAL_ADC_Start(s_adc) != HAL_OK) { g_ph_fault |= CFF_FAULT_ADC; }
  LL_ADC_ClearFlag(ADC1, LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JEOC);
  if (HAL_ADC_INJ_StartConv(s_adc) != HAL_OK) { g_ph_fault |= CFF_FAULT_ADC; }
  /* Discard the update pending from timer initialization. It predates the ADC
     trigger setup and must not be mistaken for a completed injected scan. */
  LL_TIM_ClearFlag_UPDATE(TIM1);
  NVIC_ClearPendingIRQ(TIM1_UPD_IRQn);
  NVIC_SetPriority(TIM1_UPD_IRQn, 0u);
  NVIC_EnableIRQ(TIM1_UPD_IRQn);
  s_zero_remaining = 408u;
  (void)HAL_TIM_Start_IT(tim);
  if(!cff_can_init(s_cfg.can_bitrate_kbps)) g_ph_fault|=CFF_FAULT_COMM;
}

void phase_test_on_activate(void *cdc_instance)
{
  s_cdc = (UX_SLAVE_CLASS_CDC_ACM *)cdc_instance;
  s_tx_head = 0u; s_tx_tail = 0u; s_tx_busy = 0u;
}

void phase_test_on_deactivate(void)
{
  if(s_command_source!=2) {
    uint16_t fault=(g_ph_state!=CFF_AXIS_IDLE||g_ph_mod_milli)?CFF_FAULT_COMM:0;
    if(fault) ph_safe_stop(fault);
  }
  s_cdc = NULL;
}

static void ph_can_fault(void) {
  s_can_feedback_active=0u;
  if(s_command_source==2) ph_safe_stop(CFF_FAULT_COMM);
}
static float ph_can_axis_position(void) {
  return s_enc2_enabled?(s_enc2_dir*s_enc2_pos-s_enc2_offset):
         ((float)s_cfg.encoder_dir*s_pos-s_cfg.pos_zero);
}
static float ph_can_axis_velocity(void) {
  return s_enc2_enabled?(s_enc2_dir*s_enc2_vel):((float)s_cfg.encoder_dir*s_vel);
}
static void ph_can_receive(uint16_t id,const uint8_t *p,uint8_t len,uint8_t remote) {
  if(remote||id!=s_cfg.can_node||len!=8u) return;
  s_can_feedback_active=1u;
  uint8_t opcode=0u;
  if(cff_mit_special(p,len,&opcode)) {
    if(opcode==CFF_MIT_CMD_DISABLE) {
      ph_safe_stop(0u);
      return;
    }
    if(opcode==CFF_MIT_CMD_ZERO) {
      if(g_ph_state==CFF_AXIS_IDLE) s_can_position_zero=ph_can_axis_position();
      return;
    }
    if(opcode==CFF_MIT_CMD_ENABLE) {
      if(s_can_inhibit||g_ph_fault||!s_calibrated||!s_can_auto_arm||g_ph_mod_milli) return;
      cff_mit_command_t hold={0};
      hold.position_rad=ph_can_axis_position()-s_can_position_zero;
      uint32_t mask=__get_PRIMASK();__disable_irq();s_mit_command=hold;__set_PRIMASK(mask);
      if(g_ph_state==CFF_AXIS_IDLE) ph_request_state(CFF_AXIS_CLOSED_LOOP);
      if(g_ph_state==CFF_AXIS_CLOSED_LOOP) { s_command_source=2u;s_can_enabled=1u;s_command_cycle=ph_clock_now(); }
      return;
    }
  }
  cff_mit_command_t command;
  if(!cff_mit_decode(p,len,&command)) return;
  if(s_command_source==1&&g_ph_state!=CFF_AXIS_IDLE) return;
  if(s_can_inhibit||g_ph_fault||!s_can_auto_arm) return;
  if(g_ph_state==CFF_AXIS_IDLE && s_can_auto_arm && s_calibrated && !g_ph_mod_milli) {
    ph_request_state(CFF_AXIS_CLOSED_LOOP);
    if(g_ph_state!=CFF_AXIS_CLOSED_LOOP) return;
    s_command_source=2;s_can_enabled=1;
  }
  if(g_ph_state==CFF_AXIS_CLOSED_LOOP&&s_command_source==2) {
    uint32_t mask=__get_PRIMASK();__disable_irq();s_mit_command=command;__set_PRIMASK(mask);
    s_command_cycle=ph_clock_now();
  }
}
static void ph_watchdog_task(void) {
  static uint32_t last_count,last_tick;static uint8_t started;
  uint32_t now=HAL_GetTick();
  if(!started&&now>1000) {
    /* LSI nominal 32 kHz, prescaler 32: ~2 seconds, no interrupt dependency. */
    LL_IWDG_Enable(IWDG);LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG,LL_IWDG_PRESCALER_32);LL_IWDG_SetReloadCounter(IWDG,1999);
    started=1;last_tick=now;last_count=s_pwm_count;
  }
  if(started&&s_pwm_count!=last_count) {
    LL_IWDG_ReloadCounter(IWDG);last_count=s_pwm_count;last_tick=now;
  } else if(started&&(uint32_t)(now-last_tick)>50) ph_safe_stop(CFF_FAULT_DEADLINE);
}

static void ph_cal_finalize_task(void)
{
  if (!s_cal_finalize_pending) { return; }
  __DMB();
  uint8_t pending=s_cal_finalize_pending;
  s_cal_finalize_pending = 0u;

  if (pending==2u) {
    float valid_gain[PH_CAL_BW_TEST_COUNT];
    for(unsigned i=0u;i<PH_CAL_BW_TEST_COUNT;i++)
      valid_gain[i]=s_bw_valid[i] ? s_bw_gain[i] : NAN;
    s_bw_measured_hz=ph_cal_bw_cutoff_hz(valid_gain);
    s_bw_gain_1k=s_bw_gain[8];
    s_bw_lag_1k_deg=ph_cal_bw_lag_deg(s_bw_sum_sin[8],s_bw_sum_cos[8]);
    unsigned valid_count=0u;
    for(unsigned i=0u;i<PH_CAL_BW_TEST_COUNT;i++) if(s_bw_valid[i]) {
      valid_count++;
    }
    s_bw_pass=(valid_count>=8u && s_bw_valid[8] && s_bw_gain_1k>=0.70710678f && s_bw_lag_1k_deg<=135.0f) ? 1u : 0u;
    s_cal_ph=7u; s_cal_cnt=0u; s_calibrated=1u;
    s_id_int=0.0f; s_iq_int=0.0f;
    return;
  }

  s_cal_quality = s_cal_n ? sqrtf(s_cal_sin*s_cal_sin + s_cal_cos*s_cal_cos) / (float)s_cal_n : 0.0f;
  float error = atan2f(s_cal_sin, s_cal_cos);
  float separation = ph_wrap(error - s_cal_forward_error + PH_PI) - PH_PI;
  if ((s_cal_n < 100u) || (s_cal_quality < 0.95f) || (fabsf(error) > 1.0f) ||
      (fabsf(separation) > 1.0f)) {
    ph_safe_stop(CFF_FAULT_SPINOUT);
    return;
  }

  /* Equal-weight circular mean cancels friction/load-angle bias. */
  float correction = atan2f(sinf(error) + sinf(s_cal_forward_error),
                            cosf(error) + cosf(s_cal_forward_error));
  s_cfg.pole_pairs = s_cal_pp; s_cfg.encoder_dir = s_cal_dir;
  s_cfg.encoder_offset = ph_wrap(s_cal_offset + correction);
  /* Slightly over-design the PI target to compensate the measured closed-loop
     bandwidth shortfall. Frequency response remains a separate operator action. */
  s_cfg.current_bandwidth=PH_TWO_PI*(float)PH_CAL_CURRENT_BW_TARGET_HZ;
  s_cfg.current_gain_p = s_cfg.phase_inductance * s_cfg.current_bandwidth;
  s_cfg.current_gain_i = s_cfg.phase_resistance * s_cfg.current_bandwidth;
  s_cal_ph = 7u; s_cal_cnt = 0u;
  s_cogging_valid = 0u;
  memset(s_cogging_map_ma, 0, sizeof(s_cogging_map_ma));
  s_calibrated = 1u;
  s_id_int = 0.0f; s_iq_int = 0.0f;
  s_command_cycle=ph_clock_now(); s_command_source=1u;
  g_ph_state=CFF_AXIS_IDLE;
}

void phase_test_task(void)
{
  ph_cal_finalize_task();
  if(s_config_pending&&(uint32_t)(HAL_GetTick()-s_staged_start)>3000u) s_config_pending=0;
  ph_slow_sensors();ph_watchdog_task();
  cff_can_task(ph_can_receive,ph_can_fault);
  static uint32_t last_can;
  uint32_t now=HAL_GetTick();
  if(s_can_feedback_active&&(uint32_t)(now-last_can)>=1u) {
    last_can=now;uint8_t feedback[8];
    cff_mit_feedback(feedback,s_cfg.can_node,ph_can_axis_position()-s_can_position_zero,
      ph_can_axis_velocity(),g_ph_state==CFF_AXIS_CLOSED_LOOP?s_iq_out:0,
      s_cfg.motor_temp_enable?s_motor_temp:NAN,g_ph_fault?1u:0u);
    (void)cff_can_send(s_cfg.can_node,feedback,8);
  }
  static uint32_t enc_last = 0u;
  uint32_t enc_now = ph_clock_now();
  if ((uint32_t)(enc_now - enc_last) >= PH_ENC_PERIOD_CYCLES) {
    enc_last = enc_now; ph_enc_read(); ph_enc2_read();
  }

  UX_SLAVE_CLASS_CDC_ACM *cdc = s_cdc;
  ULONG actual = 0u;
  UINT st;
  if(s_telem_due) {
    uint16_t duties[3];uint32_t primask=__get_PRIMASK();__disable_irq();
    s_telem_due=0u;duties[0]=s_telem_duty[0];duties[1]=s_telem_duty[1];duties[2]=s_telem_duty[2];
    __set_PRIMASK(primask);
    ph_tx_telem(s_adc_dma[0],s_adc_dma[1],s_adc_dma[2],duties[0],duties[1],duties[2]);
  }
  if (cdc == NULL) { return; }

  if (s_telem_pending) {
    uint8_t q[CFF_TELEM_LEN], raw[CFF_CURRENT_RAW_LEN], control[50], axis[62];
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(q, s_telem_mail, sizeof(q));
    memcpy(raw, s_raw_mail, sizeof(raw));
    memcpy(control, s_control_mail, sizeof(control));
    memcpy(axis,s_axis_mail,sizeof(axis));
    s_telem_pending = 0u;
    __set_PRIMASK(primask);
    ph_tx_frame(CFF_MSG_AXIS_DIAG,axis,sizeof(axis));
    ph_tx_frame(CFF_MSG_CURRENT_RAW, raw, sizeof(raw));
    ph_tx_frame(CFF_MSG_CONTROL_DIAG, control, sizeof(control));
    ph_tx_frame(CFF_MSG_TELEMETRY, q, sizeof(q));
  }

  st = ux_device_class_cdc_acm_read_run(cdc, s_rxbuf, (ULONG)sizeof(s_rxbuf), &actual);
  if ((st == UX_STATE_NEXT) && (actual > 0u)) { ph_parse_rx(s_rxbuf, actual); }

  if (s_tx_busy == 0u)
  {
    uint16_t avail = (uint16_t)((s_tx_head - s_tx_tail) & (PH_TXRING - 1u));
    if (avail > 0u)
    {
      uint32_t n = 0u;
      while ((avail > 0u) && (n < sizeof(s_txbuf)))
      {
        s_txbuf[n++] = s_txring[s_tx_tail];
        s_tx_tail = (uint16_t)((s_tx_tail + 1u) & (PH_TXRING - 1u));
        avail--;
      }
      s_tx_len = n;
      s_tx_busy = 1u;
    }
  }

  if (s_tx_busy != 0u)
  {
    st = ux_device_class_cdc_acm_write_run(cdc, s_txbuf, s_tx_len, &actual);
    if ((st == UX_STATE_NEXT) || (st == UX_STATE_ERROR) || (st == UX_STATE_EXIT))
    {
      s_tx_busy = 0u;
    }
  }
}

void TIM1_UPD_IRQHandler(void)
{
  /* Only UIE is enabled on this vector. Start ADC before generic HAL dispatch
     latency consumes the quiet zero-vector interval around CNT=0. */
  if ((TIM1->SR & TIM_SR_UIF) && (TIM1->DIER & TIM_DIER_UIE)) {
    TIM1->SR = ~TIM_SR_UIF;
    ph_isr();
  }
}

void HAL_TIM_UpdateCallback(hal_tim_handle_t *htim)
{
  (void)htim;
  ph_isr();
}
