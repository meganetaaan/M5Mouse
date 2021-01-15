#ifndef _M5_PINS_ANALOG_H
#define _M5_PINS_ANALOG_H
#include "stm32f4xx_hal.h"

struct m5AnalogConfigurationRecord {
  ADC_HandleTypeDef *handler;
  ADC_ChannelConfTypeDef *channel;
};
typedef struct m5AnalogConfigurationRecord m5AnalogConfigurationRecord;
typedef struct m5AnalogConfigurationRecord *m5AnalogConfiguration;

uint8_t m5analog_read(m5AnalogConfiguration analog);

#endif
