#pragma once
// VOFA上位机通讯
#define CH_COUNT (8)
typedef struct Frame {
  float fdata[CH_COUNT];
  unsigned char tail[4]; //{0x00, 0x00, 0x80, 0x7f};
} Frame_t;
static inline void FRAME_Init(Frame_t *f)
{
  f->tail[0] = 0x00;
  f->tail[1] = 0x00;
  f->tail[2] = 0x80;
  f->tail[3] = 0x7f;
}