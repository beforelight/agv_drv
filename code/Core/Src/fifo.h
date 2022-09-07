#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
typedef struct __FIFO {
  uint32_t size /*= 0*/;
  uint32_t size_mask /*= 0*/;
  volatile uint32_t in /*= 0*/;  // addr of buffer to write in
  volatile uint32_t out /*= 0*/; // addr of buffer to read out
  char *buffer /*= nullptr*/;
} FIFO;
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif // !MAX
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif // !MIN

//构造
static inline void FIFO_Init(FIFO *f, uint32_t _size)
{
  memset(f, 0, sizeof(FIFO));
  f->size = 1U;
  while (f->size < _size) {
    f->size <<= 1;
  }
  f->size_mask = f->size - 1;
  f->buffer = malloc(f->size);
}

//析构
static inline void FIFO_Denit(FIFO *f)
{
  free(f->buffer);
}

//写入buff，返回实际读写大小
static inline uint32_t FIFO_Put(FIFO *f, char *src, uint32_t len)
{
  len = MIN(len, f->size - f->in + f->out); //满了之后就不会再接收了
  uint32_t l = MIN(len, f->size - f->in);
  memcpy(f->buffer + f->in, src, l * sizeof(char));
  memcpy(f->buffer, src + l, (len - l) * sizeof(char));
  f->in = (f->in + len) & f->size_mask;
  return len;
}

//读出buff，返回实际读写大小
static inline uint32_t FIFO_Get(FIFO *f, char *dst, uint32_t len)
{
  len = MIN(len, f->in - f->out);
  uint32_t l = MIN(len, f->size - f->out);
  memcpy(dst, f->buffer + f->out, l * sizeof(char));
  memcpy(dst + l, f->buffer, (len - l) * sizeof(char));
  f->out = (f->out + len) & f->size_mask;
  return len;
}

//环形buff大小
static inline uint32_t FIFO_Size(FIFO *f) { return f->size; }
//已使用大小
static inline uint32_t FIFO_UsedSize(FIFO *f) { return ((f->size + f->in - f->out) & f->size_mask); }
//未使用大小
static inline uint32_t FIFO_UnusedSize(FIFO *f) { return ((f->size + f->out - f->in) & f->size_mask); }
//清空buff
static inline void FIFO_Clear(FIFO *f) { f->out = f->in; }
