#ifndef M5_CONTROLLERS_MOTION_H_
#define M5_CONTROLLERS_MOTION_H_
#include <stm32f4xx_hal.h>

#include "common/geometry.h"
#include "controllers/odometry.h"
#include "common/queue.h"

typedef enum {
  M5_STRAIGHT = 0x00U,
  M5_SLALOM = 0x01U,
  M5_SPIN = 0x02U,
  M5_NONE = 0xFFU
} m5MotionType;

typedef struct {
  float start_velocity;
  float max_velocity;
  float end_velocity;
  float distance;
  float accel;
  float delta;
  size_t t1;
  size_t t2;
  size_t t3;
  size_t count;
  uint8_t is_end;
} m5TrapezoidRecord, *m5Trapezoid;

typedef struct {
  m5MotionType type;
  m5Position position;
  m5Velocity velocity;
} m5TrackTarget;

typedef struct {
  m5MotionType type;
  m5Trapezoid trapezoid;
  m5Odometry odometry;
  int8_t direction;
  m5Position destination;
  uint8_t is_end;
} m5MotionRecord, *m5Motion;

typedef m5Queue m5MotionQueue;

m5Motion m5motionqueue_dequeue(m5MotionQueue queue);
void m5motionqueue_enqueue(m5MotionQueue queue, m5Motion motion);

m5Trapezoid m5trapezoid_constructor(float Vs, float Vm, float Ve, float L,
                                    float a, float freq);
void m5trapezoid_reset(m5Trapezoid tr);
float m5trapezoid_get_next(m5Trapezoid tr);
// TODO: 直進、超信地旋回、スラロームで別のコンストラクタでよい
m5Motion m5motion_constructor(m5MotionType type, m5Velocity start_velocity,
                              m5Velocity max_velocity, m5Velocity end_velocity,
                              m5Position destination, m5Accel accel,
                              float frequency);
void m5motion_destructor(m5Motion motion);
void m5motion_reset(m5Motion motion);
m5TrackTarget m5motion_get_next(m5Motion motion);

#endif