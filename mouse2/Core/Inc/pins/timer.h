#ifndef _M5_PINS_TIMER_H
#define _M5_PINS_TIMER_H
#include "stm32f4xx_hal.h"

struct m5TimerConfigurationRecord {
  TIM_HandleTypeDef *handler;
  uint32_t channel;
};
typedef struct m5TimerConfigurationRecord m5TimerConfigurationRecord;
typedef struct m5TimerConfigurationRecord *m5TimerConfiguration;

#endif
