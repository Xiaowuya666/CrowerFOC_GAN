#ifndef CONFIG_RECORD_H
#define CONFIG_RECORD_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "axis_config.h"
#include "cogging_compensation.h"
#define CFF_CONFIG_MAGIC 0x43464632u
#define CFF_CONFIG_VERSION 4u
#define CFF_CONFIG_COMMIT 0xC0FF17EDu
typedef struct {
  uint32_t magic, version, sequence, size;
  cff_config_t config;
  int16_t cogging_map_ma[CFF_COGGING_BINS];
  uint32_t cogging_valid;
  float calibration_quality;
  uint32_t calibrated;
  uint32_t encoder2_enabled;
  int32_t encoder2_dir;
  float encoder2_offset;
  uint32_t crc;
} cff_config_record_t;
/* Version 3 is the deployed record before output-encoder settings. */
typedef struct {
  uint32_t magic, version, sequence, size;
  cff_config_t config;
  int16_t cogging_map_ma[CFF_COGGING_BINS];
  uint32_t cogging_valid;
  float calibration_quality;
  uint32_t calibrated;
  uint32_t crc;
} cff_config_record_v3_t;
/* Version 2 is the deployed format; keep it readable so adding the learned
   cogging map never discards the user's electrical calibration/configuration. */
typedef struct {
  uint32_t magic, version, sequence, size;
  cff_config_t config;
  float calibration_quality;
  uint32_t calibrated;
  uint32_t crc;
} cff_config_record_v2_t;
static inline uint32_t cff_crc32(const void *data, size_t size) {
  const uint8_t *b=data; uint32_t crc=~0u;
  while(size--) { crc^=*b++; for(unsigned i=0;i<8;i++) crc=(crc>>1)^((0u-(crc&1u))&0xEDB88320u); }
  return ~crc;
}
static inline int cff_record_valid(const cff_config_record_t *r, uint32_t commit) {
  return commit==CFF_CONFIG_COMMIT && r->magic==CFF_CONFIG_MAGIC &&
    r->version==CFF_CONFIG_VERSION && r->size==sizeof(cff_config_t) &&
    r->crc==cff_crc32(r,offsetof(cff_config_record_t,crc)) && cff_config_valid(&r->config) &&
    r->calibrated<=1u && r->cogging_valid<=1u && (!r->cogging_valid||r->calibrated) && isfinite(r->calibration_quality) &&
    r->encoder2_enabled<=1u && (r->encoder2_dir==1||r->encoder2_dir==-1) && isfinite(r->encoder2_offset) &&
    (!r->calibrated || (r->config.pole_pairs>=1 && r->config.phase_resistance>0 &&
      r->config.phase_inductance>0 && r->calibration_quality>=0.95f && r->calibration_quality<=1.001f));
}
static inline int cff_record_v2_valid(const cff_config_record_v2_t *r,uint32_t commit) {
  return commit==CFF_CONFIG_COMMIT && r->magic==CFF_CONFIG_MAGIC && r->version==2u &&
    r->size==sizeof(cff_config_t) && r->crc==cff_crc32(r,offsetof(cff_config_record_v2_t,crc)) &&
    cff_config_valid(&r->config) &&
    r->calibrated<=1u && isfinite(r->calibration_quality) &&
    (!r->calibrated || (r->config.pole_pairs>=1 && r->config.phase_resistance>0 &&
      r->config.phase_inductance>0 && r->calibration_quality>=0.95f && r->calibration_quality<=1.001f));
}
static inline int cff_record_v3_valid(const cff_config_record_v3_t *r,uint32_t commit) {
  return commit==CFF_CONFIG_COMMIT && r->magic==CFF_CONFIG_MAGIC && r->version==3u &&
    r->size==sizeof(cff_config_t) && r->crc==cff_crc32(r,offsetof(cff_config_record_v3_t,crc)) &&
    cff_config_valid(&r->config) &&
    r->calibrated<=1u && r->cogging_valid<=1u &&
    (!r->cogging_valid||r->calibrated) && isfinite(r->calibration_quality) &&
    (!r->calibrated || (r->config.pole_pairs>=1 && r->config.phase_resistance>0 &&
      r->config.phase_inductance>0 && r->calibration_quality>=0.95f && r->calibration_quality<=1.001f));
}
static inline int cff_record_newer(uint32_t a,uint32_t b) { return (int32_t)(a-b)>0; }
#endif
