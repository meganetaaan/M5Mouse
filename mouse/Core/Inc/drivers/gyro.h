#ifndef _M5_DRIVER_GYRO_H_
#define _M5_DRIVER_GYRO_H_
#include "stm32f4xx_hal.h"
#include "pins/spi.h"

#define M5_GYRO_OFFSET_COUNT 100

struct m5GyroRecord {
  m5SPIConfiguration spi;
  int16_t raw;
  float ang_vel;
  int16_t raw_offset;
  int32_t offset_sum;
  size_t offset_count;
  uint8_t active;
};

typedef struct m5GyroRecord m5GyroRecord;
typedef struct m5GyroRecord *m5Gyro;

extern void m5gyro_init(m5Gyro gyro);
extern void m5gyro_start(m5Gyro gyro);
extern void m5gyro_update(m5Gyro gyro);
extern void m5gyro_calibrate(m5Gyro gyro);
extern void m5gyro_stop(m5Gyro gyro);

#endif
