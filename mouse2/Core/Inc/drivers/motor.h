#ifndef _M5_DRIVER_MOTOR_H
#define _M5_DRIVER_MOTOR_H
#include "pins/timer.h"
#include "stm32f4xx_hal.h"

struct m5MotorRecord {
  m5TimerConfiguration timer;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  uint8_t direction;
  uint8_t active;
  uint16_t duty;
};

typedef struct m5MotorRecord m5MotorRecord;
typedef struct m5MotorRecord *m5Motor;

void m5motor_init(m5Motor motor);
void m5motor_start(m5Motor motor);
void m5motor_set_pwm(m5Motor motor, uint8_t direction, uint16_t pwm);
void m5motor_stop(m5Motor motor);

#endif
