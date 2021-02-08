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

m5Motion m5motionqueue_dequeue(m5MotionQueue queue) {
  return (m5Motion)m5queue_dequeue(queue);
}
void m5motionqueue_enqueue(m5MotionQueue queue, m5Motion motion) {
  m5queue_enqueue(queue, (void *)motion);
}

void m5motion_initialize_slalom(m5Motion motion, m5Velocity start_velocity,
                                m5Velocity max_velocity,
                                m5Velocity end_velocity, m5Position destination,
                                m5Accel accel, float frequency) {
  float theta = destination.theta;
  int8_t direction = 1;
  if (theta < 0) {
    theta = -theta;
    direction = -1;
  }
  motion->direction = direction;
  /*
  if (destination.theta < 0) {
    destination.theta = -destination.theta;
    destination.x = -destination.x;
    direction = -direction;
  }
  */
  m5Trapezoid tr = m5trapezoid_constructor(
      start_velocity.omega, max_velocity.omega, end_velocity.omega,
      theta, accel.alpha, frequency);
  m5Odometry odo = motion->odometry;
  m5odometry_reset(odo);
  size_t count = 0;
  float v = start_velocity.v;
  while(!tr->is_end) {
    float omega = m5trapezoid_get_next(tr);
    m5odometry_update(odo, (m5Velocity){v, omega});
    count++;
  }
  if (abs(odo->position.x) > abs(destination.x)) {
    // printf("error");
  }
  odo->position.x *= direction;

  float x_diff = destination.x - odo->position.x;
  float post_curve = x_diff / arm_sin_f32(destination.theta);
  float pre_curve = destination.y - odo->position.y - post_curve * arm_cos_f32(destination.theta);
  motion->n1 = pre_curve * frequency / v;
  motion->n2 = motion->n1 + count;
  motion->n3 = motion->n2 + post_curve * frequency / v;
  m5trapezoid_reset(tr);
  motion->trapezoid = tr;
  m5odometry_reset(motion->odometry);
  motion->count = 0;
  motion->is_end = 0;
}

m5Motion m5motion_constructor(m5MotionType type, m5Velocity start_velocity,
                              m5Velocity max_velocity, m5Velocity end_velocity,
                              m5Position destination, m5Accel accel,
                              float frequency) {
  uint8_t direction = 1;
  m5Motion motion = malloc(sizeof(m5MotionRecord));
  motion->odometry = m5odometry_constructor(1.0 / frequency);
  motion->start_velocity = start_velocity;
  motion->max_velocity = max_velocity;
  motion->end_velocity = end_velocity;
  if (type == M5_STRAIGHT) {
    float y = destination.y;
    if (destination.y < 0) {
      y = -y;
      direction = -direction;
    }
    motion->direction = direction;
    motion->trapezoid = m5trapezoid_constructor(
        start_velocity.v, max_velocity.v, end_velocity.v, y,
        accel.a, frequency);
  } else if (type == M5_SPIN) {
    float theta = destination.theta;
    if (destination.theta < 0) {
      theta = -theta;
      direction = -direction;
    }
    motion->direction = direction;
    motion->trapezoid = m5trapezoid_constructor(
        start_velocity.omega, max_velocity.omega, end_velocity.omega,
        theta, accel.alpha, frequency);
  } else if (type == M5_SLALOM) {
    m5motion_initialize_slalom(motion, start_velocity, max_velocity, end_velocity, destination, accel, frequency);
  }
  motion->type = type;
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
  motion->count = 0;
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
    case M5_SLALOM:
      if (motion->count < motion->n1) {
        v = motion->start_velocity;
      } else if (motion->count < motion->n2) {
        v = (m5Velocity){motion->start_velocity.v, m5trapezoid_get_next(tr) * motion->direction};
      } else if (motion->count <motion->n3) {
        v = motion->end_velocity;
      } else {
        motion->is_end = 1;
        v = motion->end_velocity;
      }
      motion->count += 1;
      m5odometry_update(motion->odometry, v);
      if (motion->is_end) {
        return (m5TrackTarget){M5_SLALOM, motion->destination, v};
      }
      return (m5TrackTarget){M5_SLALOM, motion->odometry->position, v};
  }
}
