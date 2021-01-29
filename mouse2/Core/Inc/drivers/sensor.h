#ifndef _M5_DRIVER_SENSOR_H_
#define _M5_DRIVER_SENSOR_H_
#include <pins/analog.h>

#include "stm32f4xx_hal.h"

struct m5SensorRecord {
  m5AnalogConfiguration analog;
  GPIO_TypeDef *led_port;
  uint16_t led_pin;
  float gain;
  uint16_t value;
  uint8_t active;
};

typedef struct m5SensorRecord m5SensorRecord;
typedef struct m5SensorRecord *m5Sensor;

void m5sensor_init(m5Sensor sensor);
void m5sensor_start(m5Sensor sensor);
uint16_t m5sensor_read(m5Sensor sensor);
void m5sensor_stop(m5Sensor sensor);

#endif
