#ifndef M5_CONTROLLERS_ODOMETRY_H_
#define M5_CONTROLLERS_ODOMETRY_H_

#include <stdlib.h>

#include "common/geometry.h"

typedef struct {
  m5Position position;
  float prev_dx;
  float prev_dy;
  float prev_omega;
  float delta;
  uint8_t initialized;
} m5OdometryRecord, *m5Odometry;

m5Odometry m5odometry_constructor(float delta);
void m5odometry_destructor(m5Odometry odo);
void m5odometry_update(m5Odometry odo, m5Velocity v);
void m5odometry_reset(m5Odometry odo);

#endif
