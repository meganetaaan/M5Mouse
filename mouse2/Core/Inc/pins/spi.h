#ifndef _M5_PINS_SPI_H
#define _M5_PINS_SPI_H
#include "stm32f4xx_hal.h"

struct m5SPIConfigurationRecord {
  SPI_HandleTypeDef *handler;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
};
typedef struct m5SPIConfigurationRecord m5SPIConfigurationRecord;
typedef struct m5SPIConfigurationRecord *m5SPIConfiguration;

uint8_t m5spi_read_byte(m5SPIConfiguration spi, uint8_t reg);
void m5spi_write_byte(m5SPIConfiguration spi, uint8_t reg, uint8_t val);

#endif
