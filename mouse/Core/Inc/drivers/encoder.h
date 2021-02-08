#ifndef _M5_DRIVER_ENCODER_H_
#define _M5_DRIVER_ENCODER_H_
#include "stm32f4xx_hal.h"
#include "pins/timer.h"

struct m5EncoderRecord {
  m5TimerConfiguration timer;
  uint8_t direction;
  uint8_t active;
};

typedef struct m5EncoderRecord m5EncoderRecord;
typedef struct m5EncoderRecord *m5Encoder;

extern void m5encoder_init(m5Encoder encoder);
extern void m5encoder_start(m5Encoder encoder);
extern int16_t m5encoder_count(m5Encoder encoder);
extern void m5encoder_stop(m5Encoder encoder);

#endif
