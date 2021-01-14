#include "stm32f4xx_hal.h"
#include "pins/spi.h"

uint8_t m5spi_read_byte(m5SPIConfiguration spi, uint8_t reg) {
  uint8_t val = 0x00;
  reg = reg | 0x80;  // write bit 1--- ----
  HAL_GPIO_WritePin(spi->cs_port, spi->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(spi->handler, &reg, 1, 100);
  HAL_SPI_Receive(spi->handler, &val, 1, 100);
  HAL_GPIO_WritePin(spi->cs_port, spi->cs_pin, GPIO_PIN_SET);
  return val;
}

void m5spi_write_byte(m5SPIConfiguration spi, uint8_t reg, uint8_t val) {
  reg = reg & 0x7F;  // write bit 0--- ----

  HAL_GPIO_WritePin(spi->cs_port, spi->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(spi->handler, &reg, 1, 100);
  HAL_SPI_Transmit(spi->handler, &val, 1, 100);
  HAL_GPIO_WritePin(spi->cs_port, spi->cs_pin, GPIO_PIN_SET);
}
