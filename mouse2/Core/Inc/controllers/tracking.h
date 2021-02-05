#ifndef M5_CONTROLLERS_TRACKING_H_
#define M5_CONTROLLERS_TRACKING_H_
#include <arm_math.h>

#include "common/geometry.h"
#include "controllers/motion.h"

#define M5_TRACKING_GAIN_X (0.001f)
#define M5_TRACKING_GAIN_Y (0.001f)
#define M5_TRACKING_GAIN_THETA (0.001f)

m5Velocity m5tracking_get_velocity(m5Position pos, m5TrackTarget target);

#endif
