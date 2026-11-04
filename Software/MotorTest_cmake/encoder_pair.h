#ifndef CFF_ENCODER_PAIR_H
#define CFF_ENCODER_PAIR_H
#include <math.h>
#include <string.h>
typedef struct {
  unsigned count;
  float origin_motor,origin_output,mx,my,xx,yy,xy,minx,maxx,miny,maxy;
} cff_encoder_pair_t;
static inline void cff_pair_reset(cff_encoder_pair_t *p) { memset(p,0,sizeof(*p)); }
static inline void cff_pair_sample(cff_encoder_pair_t *p,float motor,float output) {
  if(!isfinite(motor)||!isfinite(output))return;
  if(!p->count){p->origin_motor=motor;p->origin_output=output;}
  float x=motor-p->origin_motor,y=output-p->origin_output;
  p->count++;float dx=x-p->mx,dy=y-p->my;
  p->mx+=dx/(float)p->count;p->my+=dy/(float)p->count;
  p->xx+=dx*(x-p->mx);p->yy+=dy*(y-p->my);p->xy+=dx*(y-p->my);
  p->minx=fminf(p->minx,x);p->maxx=fmaxf(p->maxx,x);
  p->miny=fminf(p->miny,y);p->maxy=fmaxf(p->maxy,y);
}
static inline int cff_pair_finish(const cff_encoder_pair_t *p,float *ratio,
                                int *relative_dir,float *quality) {
  *quality=(p->xx>1e-9f&&p->yy>1e-9f)?p->xy*p->xy/(p->xx*p->yy):0;
  if(p->count<100 || p->maxx-p->minx<1.0f || p->maxy-p->miny<0.02f ||
     !isfinite(*quality)||*quality<0.995f)return 0;
  float slope=p->xy/p->xx,r=1.0f/fabsf(slope);
  if(!isfinite(r)||r<0.001f||r>10000.0f)return 0;
  *ratio=r;*relative_dir=slope>=0?1:-1;return 1;
}
#endif
