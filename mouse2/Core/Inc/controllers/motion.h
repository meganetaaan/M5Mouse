#ifndef M5_CONTROLLERS_MOTION_H_
#define M5_CONTROLLERS_MOTION_H_
#include <stm32f4xx_hal.h>

typedef struct {
  float start_velocity;
  float max_velocity;
  float end_velocity;
  float distance;
  float accel;
  size_t count;
  uint8_t is_end;
  uint8_t direction;
  size_t t1;
  size_t t2;
  size_t t3;
  float delta;
} m5Motion2Record, *m5Motion2;

m5Motion2 m5motion2_constructor(float start_velocity, float max_velocity, float end_velocity, float distance, uint8_t direction, float accel, float frequency);
void m5motion2_destructor(m5Motion2 motion);
void m5motion2_reset(m5Motion2 motion);
float m5motion2_get_next_velocity(m5Motion2 motion);

#endif