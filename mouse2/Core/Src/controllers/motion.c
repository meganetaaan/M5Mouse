#include "controllers/motion.h"
#include <stm32f4xx_hal.h>
#include <stdlib.h>
#define ARM_MATH_CM4
#include <arm_math.h>

m5Motion2 m5motion2_constructor(float start_velocity, float max_velocity, float end_velocity, float distance, uint8_t direction, float accel, float frequency) {
  if (distance < 0) {
    distance = -distance;
    direction = -direction;
  }
  m5Motion2 motion = malloc(sizeof(m5Motion2Record));
  float t1 = (max_velocity - start_velocity) * frequency / accel;
  float t3 = (max_velocity - end_velocity) * frequency / accel;
  float l1 = t1 * (start_velocity + max_velocity) / (2.0 * frequency);
  float l3 = t3 * (max_velocity + end_velocity) / (2.0 * frequency);
  float t2 = (distance - l1 - l3) * frequency / max_velocity;
  if (t1 > t2) {
    float Vmaxp = sqrt(((start_velocity * start_velocity + end_velocity * end_velocity) + 2 * accel * distance) / 2);
    t1 = (Vmaxp - start_velocity) * frequency / accel;
    t3 = (Vmaxp - end_velocity) * frequency / accel;
    t2 = 0;
  }
  motion->t1 = t1;
  motion->t2 = t1 + t2;
  motion->t3 = t1 + t2 + t3;
  motion->start_velocity = start_velocity;
  motion->max_velocity = max_velocity;
  motion->end_velocity = end_velocity;
  motion->distance = distance;
  motion->delta = 1.0 / frequency;
  motion->accel = accel;
  motion->direction = direction;
  m5motion2_reset(motion);
  return motion;
}

void m5motion2_destructor(m5Motion2 motion) {
  free(motion);
}

void m5motion2_reset(m5Motion2 motion) {
  motion->count = 0;
  motion->is_end = 0;
}

float m5motion2_get_next_velocity(m5Motion2 motion) {
  float velocity;
  if (motion->count < motion->t1) {
    velocity = motion->start_velocity + (motion->count * motion->accel * motion->delta);
  } else if (motion->count < motion->t2) {
    velocity = motion->max_velocity;
  } else if (motion->count < motion->t3) {
    size_t c = motion->t3 - motion->count;
    velocity = motion->end_velocity + (c * motion->accel * motion->delta);
  } else {
    velocity = motion->end_velocity;
    motion->is_end = 1;
  }
  motion->count += 1;
  return velocity * motion->direction;
}
