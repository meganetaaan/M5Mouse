#include "controllers/motion.h"

#include <stdlib.h>
#include <stm32f4xx_hal.h>
// #define ARM_MATH_CM4
#include <arm_math.h>

#include "common/geometry.h"

m5Trapezoid m5trapezoid_constructor(float Vs, float Vm, float Ve, float L,
                                    float a, float freq) {
  m5Trapezoid trapezoid = malloc(sizeof(m5TrapezoidRecord));
  float t1 = (Vm - Vs) * freq / a;
  float t3 = (Vm - Ve) * freq / a;
  float l1 = t1 * (Vs + Vm) / (2.0 * freq);
  float l3 = t3 * (Vm + Ve) / (2.0 * freq);
  float t2 = (L - l1 - l3) * freq / Vm;
  if (t1 > t2) {
    float Vmaxp = sqrt(((Vs * Vs + Ve * Ve) + 2 * a * L) / 2);
    t1 = (Vmaxp - Vs) * freq / a;
    t3 = (Vmaxp - Ve) * freq / a;
    t2 = 0;
  }
  trapezoid->start_velocity = Vs;
  trapezoid->max_velocity = Vm;
  trapezoid->end_velocity = Ve;
  trapezoid->distance = L;
  trapezoid->accel = a;
  trapezoid->delta = 1.0 / freq;
  trapezoid->t1 = t1;
  trapezoid->t2 = t1 + t2;
  trapezoid->t3 = t1 + t2 + t3;
  trapezoid->count = 0;
  trapezoid->is_end = 0;
  return trapezoid;
}

void m5trapezoid_reset(m5Trapezoid tr) {
  tr->is_end = 0;
  tr->count = 0;
}
float m5trapezoid_get_next(m5Trapezoid tr) {
  tr->count += 1;
  size_t count = tr->count;
  if (tr->count < tr->t1) {
    return tr->start_velocity + (count * tr->accel * tr->delta);
  } else if (count < tr->t2) {
    return tr->max_velocity;
  } else if (count < tr->t3) {
    size_t c = tr->t3 - count;
    return tr->end_velocity + (c * tr->accel * tr->delta);
  } else {
    tr->is_end = 1;
    return tr->end_velocity;
  }
}

m5Motion m5motion_constructor(m5MotionType type, m5Velocity start_velocity,
                              m5Velocity max_velocity, m5Velocity end_velocity,
                              m5Position destination, m5Accel accel,
                              float frequency) {
  uint8_t direction = 1;
  m5Motion motion = malloc(sizeof(m5MotionRecord));
  if (type == M5_STRAIGHT) {
    if (destination.y < 0) {
      destination.y = -destination.y;
      direction = -direction;
    }
    motion->trapezoid = m5trapezoid_constructor(
        start_velocity.v, max_velocity.v, end_velocity.v, destination.y,
        accel.a, frequency);
  } else if (type == M5_SPIN) {
    if (destination.theta < 0) {
      destination.theta = -destination.theta;
      direction = -direction;
    }
    motion->trapezoid = m5trapezoid_constructor(
        start_velocity.omega, max_velocity.omega, end_velocity.omega,
        destination.theta, accel.alpha, frequency);
  }
  motion->type = type;
  motion->direction = direction;
  motion->odometry = m5odometry_constructor(1.0 / frequency);
  motion->destination = destination;
  m5motion_reset(motion);
  return motion;
}

void m5motion_destructor(m5Motion motion) {
  if (motion->trapezoid != NULL) {
    free(motion->trapezoid);
    motion->trapezoid = NULL;
  }
  if (motion->odometry != NULL) {
    free(motion->odometry);
    motion->odometry = NULL;
  }
  free(motion);
}

void m5motion_reset(m5Motion motion) {
  m5trapezoid_reset(motion->trapezoid);
  motion->is_end = 0;
}

m5TrackTarget m5motion_get_next(m5Motion motion) {
  m5Trapezoid tr = motion->trapezoid;
  m5Velocity v;
  switch (motion->type) {
    case M5_STRAIGHT:
      v = (m5Velocity){m5trapezoid_get_next(tr) * motion->direction, 0.0};
      m5odometry_update(motion->odometry, v);
      if (tr->is_end) {
        motion->is_end = 1;
        return (m5TrackTarget){M5_NONE, motion->destination, v};
      }
      return (m5TrackTarget){M5_STRAIGHT, motion->odometry->position, v};
    case M5_SPIN:
      if (tr->is_end) {
        motion->is_end = 1;
        return (m5TrackTarget){M5_SPIN, motion->destination,
                               (m5Velocity){0, 0}};
      }
      v = (m5Velocity){0.0, m5trapezoid_get_next(tr) * motion->direction};
      m5odometry_update(motion->odometry, v);
      return (m5TrackTarget){M5_SPIN, motion->odometry->position, v};
  }
}
