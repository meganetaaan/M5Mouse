#ifndef _M5_PINS_I2C_H_
#define _M5_PINS_I2C_H_
#include "stm32f4xx_hal.h"

struct m5I2CConfigurationRecord {
  I2C_HandleTypeDef *handler;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
};
typedef struct m5I2CConfigurationRecord m5I2CConfigurationRecord;
typedef struct m5I2CConfigurationRecord *m5I2CConfiguration;

uint8_t m5i2c_read_byte(m5I2CConfiguration i2c, uint8_t reg, size_t size,);
void m5i2c_write_byte(m5I2CConfiguration i2c, uint8_t reg, uint8_t val, size_t size);

#endif
