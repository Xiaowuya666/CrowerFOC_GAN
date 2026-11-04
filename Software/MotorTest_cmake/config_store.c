#include "config_store.h"
#include "config_record.h"
#include "stm32_hal.h"
#include "stm32c5xx_hal_flash.h"
#include "stm32c5xx_hal_flash_itf.h"
/* The linker excludes both 8 KiB pages from executable ROM. No option bytes
   are modified. HAL EraseByAddr handles actual bank topology. */
#define SLOT0 0x0807C000u
#define SLOT1 0x0807E000u
#define PAGE_BYTES 8192u
#define COMMIT_OFFSET 512u
_Static_assert(sizeof(cff_config_record_t)<=COMMIT_OFFSET,"configuration record too large");
static void sanitize_legacy_axis_config(cff_config_t *c) {
  if(c->limit_vel>CFF_MOTOR_MAX_RAD_S)c->limit_vel=CFF_MOTOR_MAX_RAD_S;
  if(c->overspeed_limit>CFF_AXIS_TRIP_RAD_S)c->overspeed_limit=CFF_AXIS_TRIP_RAD_S;
  if(c->traj_vel>CFF_AXIS_MAX_RAD_S)c->traj_vel=CFF_AXIS_MAX_RAD_S;
  c->input_pos=c->input_vel=c->input_torque=0.0f;
  c->pos_limits_enable=0u; c->pos_zero=0.0f;
  c->pos_min=-6.2831853f;c->pos_max=6.2831853f;c->circular_enable=0u;c->circular_range=6.2831853f;
}
static int active_slot=-1;
uint32_t g_config_sequence;
static int valid_slot(unsigned slot,uint32_t *sequence,cff_config_t *config,
                      uint8_t *calibrated,float *quality,int16_t *cog_map,uint8_t *cog_valid,
                      uint8_t *enc2_enabled,int8_t *enc2_dir,float *enc2_offset,float *enc2_ratio,float *enc2_quality) {
  *enc2_ratio=0;*enc2_quality=0;
  uint32_t addr=slot?SLOT1:SLOT0, header[4];
  uint32_t commit=*(const uint32_t*)(addr+COMMIT_OFFSET);
  memcpy(header,(const void*)addr,sizeof(header));
  if(header[1]==CFF_CONFIG_VERSION) {
    cff_config_record_t r; memcpy(&r,(const void*)addr,sizeof(r));
    if(!cff_record_valid(&r,commit)) return 0;
    *sequence=r.sequence;*config=r.config;
    *calibrated=(uint8_t)r.calibrated;*quality=r.calibration_quality;
    memcpy(cog_map,r.cogging_map_ma,sizeof(r.cogging_map_ma));*cog_valid=(uint8_t)r.cogging_valid;
    *enc2_enabled=(uint8_t)r.encoder2_enabled;*enc2_dir=(int8_t)r.encoder2_dir;*enc2_offset=r.encoder2_offset;
    *enc2_ratio=r.encoder2_ratio;*enc2_quality=r.encoder2_quality;
    return 1;
  }
  if(header[1]==4u) {
    cff_config_record_v4_t r;memcpy(&r,(const void*)addr,sizeof(r));
    if(!cff_record_v4_valid(&r,commit))return 0;
    *sequence=r.sequence;*config=r.config;
    *calibrated=(uint8_t)r.calibrated;*quality=r.calibration_quality;
    memcpy(cog_map,r.cogging_map_ma,sizeof(r.cogging_map_ma));*cog_valid=(uint8_t)r.cogging_valid;
    *enc2_enabled=(uint8_t)r.encoder2_enabled;*enc2_dir=(int8_t)r.encoder2_dir;*enc2_offset=r.encoder2_offset;
    return 1;
  }
  if(header[1]==3u) {
    cff_config_record_v3_t r; memcpy(&r,(const void*)addr,sizeof(r));
    if(!cff_record_v3_valid(&r,commit)) return 0;
    *sequence=r.sequence;*config=r.config;sanitize_legacy_axis_config(config);
    if(!cff_config_valid(config))return 0;
    *calibrated=(uint8_t)r.calibrated;*quality=r.calibration_quality;
    memcpy(cog_map,r.cogging_map_ma,sizeof(r.cogging_map_ma));*cog_valid=(uint8_t)r.cogging_valid;
    *enc2_enabled=0u;*enc2_dir=1;*enc2_offset=0.0f;
    return 1;
  }
  if(header[1]==2u) {
    cff_config_record_v2_t r; memcpy(&r,(const void*)addr,sizeof(r));
    if(!cff_record_v2_valid(&r,commit)) return 0;
    *sequence=r.sequence;*config=r.config;sanitize_legacy_axis_config(config);
    if(!cff_config_valid(config))return 0;
    *calibrated=(uint8_t)r.calibrated;*quality=r.calibration_quality;
    memset(cog_map,0,sizeof(int16_t)*CFF_COGGING_BINS);*cog_valid=0u;
    *enc2_enabled=0u;*enc2_dir=1;*enc2_offset=0.0f;
    return 1;
  }
  return 0;
}
int cff_store_load(cff_config_t *config,uint8_t *calibrated,float *quality,
                   int16_t *cog_map,uint8_t *cog_valid,uint8_t *enc2_enabled,
                   int8_t *enc2_dir,float *enc2_offset,float *enc2_ratio,float *enc2_quality) {
  if(FLASH_SIZE!=0x80000u) return 0;
  uint32_t seq[2]={0,0}; cff_config_t cfg[2]; uint8_t cal[2]={0,0},cv[2]={0,0},e2[2]={0,0};
  int8_t e2dir[2]={1,1};float e2off[2]={0,0},e2ratio[2]={0,0},e2quality[2]={0,0};
  float q[2]={0,0}; int16_t maps[2][CFF_COGGING_BINS];
  int va=valid_slot(0,&seq[0],&cfg[0],&cal[0],&q[0],maps[0],&cv[0],&e2[0],&e2dir[0],&e2off[0],&e2ratio[0],&e2quality[0]);
  int vb=valid_slot(1,&seq[1],&cfg[1],&cal[1],&q[1],maps[1],&cv[1],&e2[1],&e2dir[1],&e2off[1],&e2ratio[1],&e2quality[1]);
  if(!va&&!vb) return 0;
  active_slot=vb&&(!va||cff_record_newer(seq[1],seq[0]))?1:0;
  *config=cfg[active_slot];*calibrated=cal[active_slot];*quality=q[active_slot];
  memcpy(cog_map,maps[active_slot],sizeof(maps[active_slot]));*cog_valid=cv[active_slot];
  *enc2_enabled=e2[active_slot];*enc2_dir=e2dir[active_slot];*enc2_offset=e2off[active_slot];
  *enc2_ratio=e2ratio[active_slot];*enc2_quality=e2quality[active_slot];
  config->input_pos=config->input_vel=config->input_torque=0;
  g_config_sequence=seq[active_slot];
  return 1;
}
int cff_store_save(const cff_config_t *config,uint8_t calibrated,float quality,
                   const int16_t *cog_map,uint8_t cog_valid,uint8_t enc2_enabled,
                   int8_t enc2_dir,float enc2_offset,float enc2_ratio,float enc2_quality) {
  if(FLASH_SIZE!=0x80000u||!cff_config_valid(config)) return 0;
  if(cog_valid>1u||(!calibrated&&cog_valid)||enc2_enabled>1u||(enc2_dir!=1&&enc2_dir!=-1)||!isfinite(enc2_offset)) return 0;
  hal_flash_handle_t flash={0};
  uint32_t image[COMMIT_OFFSET/4]; memset(image,0xFF,sizeof(image));
  cff_config_record_t r={0};
  r.magic=CFF_CONFIG_MAGIC; r.version=CFF_CONFIG_VERSION; r.size=sizeof(*config);
  r.sequence=g_config_sequence+1u; r.config=*config;
  memcpy(r.cogging_map_ma,cog_map,sizeof(r.cogging_map_ma));r.cogging_valid=cog_valid;
  r.config.input_pos=r.config.input_vel=r.config.input_torque=0;
  r.calibrated=calibrated; r.calibration_quality=quality;
  r.encoder2_enabled=enc2_enabled;r.encoder2_dir=enc2_dir;r.encoder2_offset=enc2_offset;
  r.encoder2_ratio=isfinite(enc2_ratio)?enc2_ratio:0;
  r.encoder2_quality=r.encoder2_ratio?enc2_quality:0;
  r.crc=cff_crc32(&r,offsetof(cff_config_record_t,crc));
  if(!cff_record_valid(&r,CFF_CONFIG_COMMIT)) return 0;
  memcpy(image,&r,sizeof(r));
  unsigned next=active_slot==0?1u:0u;
  uint32_t addr=next?SLOT1:SLOT0;
  uint32_t commit[4]={CFF_CONFIG_COMMIT,r.sequence,~r.sequence,CFF_CONFIG_MAGIC};
  int ok=0;
  if(HAL_FLASH_Init(&flash,HAL_FLASH)!=HAL_OK) return 0;
  if(HAL_FLASH_ITF_Unlock(HAL_FLASH)==HAL_OK) {
    if(HAL_FLASH_EraseByAddr(&flash,addr,PAGE_BYTES,1000u)==HAL_OK &&
       HAL_FLASH_ProgramByAddr(&flash,addr,image,sizeof(image),1000u)==HAL_OK &&
       memcmp((const void*)addr,image,sizeof(image))==0 &&
       HAL_FLASH_ProgramByAddr(&flash,addr+COMMIT_OFFSET,commit,sizeof(commit),1000u)==HAL_OK) {
      uint32_t seq;cff_config_t cfg;uint8_t cal,cv,e2;int8_t e2dir;float q,e2off,e2ratio,e2quality;int16_t map[CFF_COGGING_BINS];
      ok=valid_slot(next,&seq,&cfg,&cal,&q,map,&cv,&e2,&e2dir,&e2off,&e2ratio,&e2quality);
    }
    (void)HAL_FLASH_ITF_Lock(HAL_FLASH);
  }
  (void)HAL_FLASH_DeInit(&flash);
  if(ok) { active_slot=(int)next; g_config_sequence=r.sequence; }
  return ok;
}
