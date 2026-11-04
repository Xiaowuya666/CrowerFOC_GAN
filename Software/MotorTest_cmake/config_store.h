#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H
#include "axis_config.h"
#include "cogging_compensation.h"
int cff_store_load(cff_config_t *config, uint8_t *calibrated, float *quality,
                   int16_t *cogging_map_ma, uint8_t *cogging_valid,
                   uint8_t *encoder2_enabled, int8_t *encoder2_dir, float *encoder2_offset, float *encoder2_ratio, float *encoder2_quality);
int cff_store_save(const cff_config_t *config, uint8_t calibrated, float quality,
                   const int16_t *cogging_map_ma, uint8_t cogging_valid,
                   uint8_t encoder2_enabled, int8_t encoder2_dir, float encoder2_offset, float encoder2_ratio, float encoder2_quality);
extern uint32_t g_config_sequence;
#endif
