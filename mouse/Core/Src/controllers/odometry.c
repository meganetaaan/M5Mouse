#include <stdlib.h>
#include <stm32f405xx.h>
#include <arm_math.h>

#include "controllers/odometry.h"

m5Odometry m5odometry_constructor(float delta) {
  m5Odometry odo = malloc(sizeof(m5OdometryRecord));
  odo->position = (m5Position){0.0, 0.0, 0.0};
  odo->initialized = 0;
  odo->delta = delta;
  odo->prev_dx = 0;
  odo->prev_dy = 0;
  odo->prev_omega = 0;
  odo->x = 0;
  odo->y = 0;
  return odo;
}

void m5odometry_destructor(m5Odometry odo) {
  free(odo);
}

void m5odometry_update(m5Odometry odo, m5Velocity v) {
  m5Position pos = odo->position;
  float theta = pos.theta;
  float sign = 1.0;
  if (theta < 0) {
    theta = -theta;
    sign = -1;
  }
  float cos_theta = arm_cos_f32(theta);
  float sin_theta = arm_sin_f32(theta) * sign;
  float dx = sin_theta * v.v;
  float dy = cos_theta * v.v;
  if (!odo->initialized) {
    odo->prev_dx = dx;
    odo->prev_dy = dy;
    odo->prev_omega = v.omega;
    odo->initialized = 1;
  }

  odo->x += ((dx + odo->prev_dx) / 2) * odo->delta;
  odo->y += ((dy + odo->prev_dy) / 2) * odo->delta;
  odo->position.x = (float)odo->x;
  odo->position.y = (float)odo->y;
  odo->position.theta += (v.omega + odo->prev_omega) / 2 * odo->delta;

  odo->prev_dx = dx;
  odo->prev_dy = dy;
  odo->prev_omega = v.omega;
}

void m5odometry_reset(m5Odometry odo) {
  odo->x = 0;
  odo->y = 0;
  odo->position = (m5Position) {0.0, 0.0, 0.0};
  odo->prev_dx = 0.0;
  odo->prev_dy = 0.0;
  odo->prev_omega = 0.0;
  odo->initialized = 0;
}
