#ifndef M5_CONTROLLERS_TRACKING_H_
#define M5_CONTROLLERS_TRACKING_H_
#include <arm_math.h>

#include "common/geometry.h"
#include "controllers/motion.h"

// #define M5_TRACKING_GAIN_THETA (1.6 * 0.01f)
// #define M5_TRACKING_GAIN_X (M5_TRACKING_GAIN_THETA * M5_TRACKING_GAIN_THETA / 4)
// #define M5_TRACKING_GAIN_Y (0.01f)
#define M5_TRACKING_GAIN_THETA (0.95 * 0.1f)
#define M5_TRACKING_GAIN_X (M5_TRACKING_GAIN_THETA * M5_TRACKING_GAIN_THETA / 4)
#define M5_TRACKING_GAIN_Y (50.0f)

m5Velocity m5tracking_get_velocity(m5Position pos, m5TrackTarget target);

#endif
