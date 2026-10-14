#include "../../Software/MotorTest_cmake/motion_control.h"
#include "../../Software/MotorTest_cmake/config_record.h"
#include "../../Software/MotorTest_cmake/axis_parameters.h"
#include "../../Software/MotorTest_cmake/mit_can_protocol.h"
#include "../../Software/MotorTest_cmake/foc_limits.h"
extern const cff_config_t test_defaults;
#define CHECK(x,n) do {if(!(x)) return n;}while(0)
int test_entry(void) {
  cff_config_t c=test_defaults;
  CHECK(cff_config_valid(&c)&&c.current_limit==40&&c.pole_pairs==0&&c.phase_resistance==0,1);
  c.limit_vel=CFF_MOTOR_MAX_RAD_S+1;c.overspeed_limit=100;c.traj_vel=100;
  CHECK(cff_config_migrate_speed_units(&c)&&fabsf(c.limit_vel-CFF_MOTOR_MAX_RAD_S)<.001f&&
        fabsf(c.overspeed_limit-CFF_AXIS_TRIP_RAD_S)<.001f&&
        fabsf(c.traj_vel-CFF_AXIS_MAX_RAD_S)<.001f,56);
  c=test_defaults;c.traj_vel=100;
  CHECK(cff_config_migrate_speed_units(&c)&&fabsf(c.traj_vel-CFF_AXIS_MAX_RAD_S)<.001f,58);
  c=test_defaults;c.limit_vel=12;c.overspeed_limit=9;
  CHECK(!cff_config_migrate_speed_units(&c)&&c.limit_vel==12&&c.overspeed_limit==9,57);
  c=test_defaults;
  c.overspeed_limit=20;CHECK(cff_config_valid(&c),2);
  c.overspeed_limit=NAN;CHECK(!cff_config_valid(&c),3);c=test_defaults;
  c.torque_units_nm=1;CHECK(!cff_config_valid(&c),4);c=test_defaults;
  c.pos_limits_enable=1;c.circular_enable=1;CHECK(!cff_config_valid(&c),5);c=test_defaults;
  c.motor_temp_enable=1;c.motor_ntc_beta=0;CHECK(!cff_config_valid(&c),6);c=test_defaults;
  c.motor_ntc_beta=3450;c.motor_ntc_r25=10000;
  CHECK(cff_config_migrate_motor_ptc(&c)&&fabsf(c.motor_ntc_beta-9.9130435f)<.001f&&c.motor_ntc_r25==620,63);
  CHECK(cff_config_valid(&c),64);
  CHECK(!cff_param_write(&c,cff_param_find(0x0800),1.5f),7);
  CHECK(cff_param_write(&c,cff_param_find(0x0402),20)&&cff_config_valid(&c),8);
  c=test_defaults;
  cff_config_record_t r={0};r.magic=CFF_CONFIG_MAGIC;r.version=CFF_CONFIG_VERSION;r.encoder2_dir=1;
  r.sequence=0xFFFFFFFE;r.size=sizeof(c);r.config=c;r.calibration_quality=0;
  r.crc=cff_crc32(&r,offsetof(cff_config_record_t,crc));
  CHECK(cff_record_valid(&r,CFF_CONFIG_COMMIT),10);
  CHECK(!cff_record_valid(&r,0xFFFFFFFF),11);
  CHECK(cff_record_newer(0,0xFFFFFFFF)&&!cff_record_newer(0xFFFFFFFF,0),12);
  for(unsigned i=0;i<sizeof(r);i++) {
    cff_config_record_t damaged=r;((uint8_t*)&damaged)[i]^=1;
    /* Tail padding is not part of the record checksum or payload. */
    if(i<offsetof(cff_config_record_t,crc)+sizeof(r.crc)) CHECK(!cff_record_valid(&damaged,CFF_CONFIG_COMMIT),13);
  }
  r.calibrated=1;r.crc=cff_crc32(&r,offsetof(cff_config_record_t,crc));
  CHECK(!cff_record_valid(&r,CFF_CONFIG_COMMIT),14);
  cff_config_record_v2_t old={0};old.magic=CFF_CONFIG_MAGIC;old.version=2u;
  old.sequence=77u;old.size=sizeof(c);old.config=test_defaults;old.config.pole_pairs=21u;
  old.config.phase_resistance=1.0f;old.config.phase_inductance=.001f;old.calibration_quality=.99f;
  old.calibrated=1u;old.crc=cff_crc32(&old,offsetof(cff_config_record_v2_t,crc));
  CHECK(cff_record_v2_valid(&old,CFF_CONFIG_COMMIT),61);
  old.config.current_limit=0;old.crc=cff_crc32(&old,offsetof(cff_config_record_v2_t,crc));
  CHECK(!cff_record_v2_valid(&old,CFF_CONFIG_COMMIT),62);
  cff_traj_t t;cff_traj_plan(&t,0,0,1,2,5,4);
  float prev=0;
  for(unsigned i=0;i<4000;i++) {
    cff_traj_step(&t,.001f);
    CHECK(isfinite(t.p)&&fabsf(t.v)<=2.001f&&fabsf(t.p-prev)<=.0021f,20);prev=t.p;
  }
  CHECK(t.done&&t.p==1&&t.v==0,21);
  cff_traj_plan(&t,1,1,-2,2,5,4);
  for(unsigned i=0;i<6000;i++) {cff_traj_step(&t,(i%2)? .0007f:.0013f);CHECK(fabsf(t.v)<=2.001f,22);}
  CHECK(t.done&&t.p==-2&&t.v==0,23);
  cff_traj_plan(&t,0,0,0,2,5,4);cff_traj_step(&t,.001f);CHECK(t.done,24);
  c.control_mode=1;c.input_mode=2;c.input_vel=2;c.vel_ramp=1;
  cff_motion_t m;cff_motion_reset(&m,0,0);
  float iq=cff_motion_step(&m,&c,0,0,1,.001f);
  CHECK(fabsf(m.vel-.001f)<.00001f&&fabsf(iq)<=1,30);
  c.input_mode=1;c.input_vel=100;c.vel_gain=1;c.vel_integrator_gain=10;
  for(unsigned i=0;i<1000;i++) {iq=cff_motion_step(&m,&c,0,0,1,.001f);CHECK(fabsf(iq)<=1,31);}
  CHECK(fabsf(m.integral)<.001f,32);
  c.control_mode=0;c.input_torque=40;c.vel_gain=.1f;c.limit_vel=10;
  CHECK(cff_motion_step(&m,&c,0,10,40,.001f)<=.0001f,33);
  CHECK(cff_motion_step(&m,&c,0,20,40,.001f)<0,34);
  c=test_defaults;c.control_mode=2;c.input_mode=4;c.input_pos=1;
  cff_motion_reset(&m,0,0);
  float position_peak=0;
  for(unsigned i=0;i<5000;i++) {iq=cff_motion_step(&m,&c,m.pos,m.vel,1,.001f);CHECK(fabsf(iq)<=1,35);if(fabsf(m.vel)>position_peak)position_peak=fabsf(m.vel);}
  CHECK(position_peak<=CFF_POSITION_MAX_RAD_S+.001f,59);
  CHECK(m.traj.done&&fabsf(m.pos-1)<.00001f,36);
  c.circular_enable=1;c.circular_range=6.2831853f;
  CHECK(fabsf(cff_position_error(.1f,6.2f,&c)-.1831853f)<.0001f,37);
  CHECK(fabsf(cff_ptc_temperature((uint16_t)(620.0f*4095.0f/10620.0f),620,9.9130435f)-25)<.02f,40);
  CHECK(fabsf(cff_ptc_temperature((uint16_t)(848.0f*4095.0f/10848.0f),620,9.9130435f)-48)<.1f,65);
  CHECK(!isfinite(cff_ptc_temperature(0,620,9.9130435f))&&!isfinite(cff_ptc_temperature(4095,620,9.9130435f)),41);
  CHECK(cff_temperature_derating(80,70,90)==.5f&&cff_temperature_derating(95,70,90)==0,42);
  uint8_t p[8]={0x80,0,0x80,0,0,0,0x08,0};cff_mit_command_t command;
  CHECK(cff_mit_decode(p,8,&command)&&fabsf(command.iq_ff_A)<.02f,50);
  CHECK(!cff_mit_decode(p,7,&command),51);
  uint8_t opcode=0;for(unsigned i=0;i<7u;i++)p[i]=0xFFu;p[7]=CFF_MIT_CMD_ENABLE;
  CHECK(cff_mit_special(p,8,&opcode)&&opcode==CFF_MIT_CMD_ENABLE,52);
  p[7]=0u;CHECK(!cff_mit_special(p,8,&opcode),53);
  cff_mit_feedback(p,3,0,0,40,NAN,0);
  CHECK(p[0]==3u&&p[5]==0xFFu&&p[6]==0xFFu,54);
  float id=0,iqi=0,vd,vq;
  ph_current_pi_ff_step(0,1,.2f,100,.000025f,2,10,10,&id,&iqi,&vd,&vq);
  CHECK(isfinite(id)&&isfinite(iqi)&&sqrtf(vd*vd+vq*vq)<2.0001f,60);
  return 0;
}
