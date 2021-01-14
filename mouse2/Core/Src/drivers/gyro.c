#include "drivers/gyro.h"
#include "pins/spi.h"
#include <stdio.h>

uint8_t m5gyro_read_byte(m5SPIConfiguration spi, uint8_t reg) {
  return m5spi_read_byte(spi, reg | 0x80 /* mask first bit 1--- ---- */);
}

void m5gyro_write_byte(m5SPIConfiguration spi, uint8_t reg, uint8_t val) {
  return m5spi_write_byte(spi, reg & 0x7F, val /* mask first bit 0--- ---- */);
}

void m5gyro_init(m5Gyro gyro) {
  gyro->raw = 1000;
  gyro->ang_vel = 0.01f;
  gyro->active = 0;

  // SPI_CSをプルアップ
  HAL_GPIO_WritePin(gyro->spi->cs_port, gyro->spi->cs_pin, GPIO_PIN_SET);

  // who am i
  uint8_t who_am_i;
  who_am_i = m5gyro_read_byte(gyro->spi, 0x00);
  printf("WHO_AM_I = 0x%02x\n", who_am_i);

  m5SPIConfiguration spi = gyro->spi;
  // ICM-20648 settings
  m5gyro_write_byte(spi, 0x06, 0x81);  // 1000 0001 wake from sleep
  HAL_Delay(50);
  m5gyro_write_byte(spi, 0x06, 0x01);  // 0000 0001 turn off low power mode
  HAL_Delay(50);

  m5gyro_write_byte(spi, 0x7f, 0x20);  // 0010 0000 User Bank2に変更
  HAL_Delay(50);
  m5gyro_write_byte(spi, 0x01,
                    0x07);  // 0000 0111 GYRO_FS_SEL=3(Range 2000dps)に変更
  HAL_Delay(50);            //           Digital Low Pass Filterを有効化
                  // このときgyro_sensitivityは 32768/2000=16.4 LSB/dps
  m5gyro_write_byte(spi, 0x7f, 0x00);  // 0000 0000 User Bank0に変更
  HAL_Delay(50);

  m5gyro_write_byte(spi, 0x06, 0x21);  // 0010 0001 turn on low power mode
  HAL_Delay(50);
}

void m5gyro_start(m5Gyro gyro) { gyro->active = 1; }

void m5gyro_stop(m5Gyro gyro) { gyro->active = 0; }

void m5gyro_update(m5Gyro gyro) {
  uint8_t zout_h, zout_l;

  zout_h = m5gyro_read_byte(gyro->spi, 0x37);
  zout_l = m5gyro_read_byte(gyro->spi, 0x38);

  gyro->raw = ((zout_h << 8) & 0xff00) | (zout_l & 0x00ff);
  gyro->ang_vel = (float)gyro->raw / 16.4;
}
