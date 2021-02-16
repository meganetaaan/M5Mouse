#include "controllers/tracking.h"

m5Velocity m5tracking_get_velocity(m5Position pos, m5TrackTarget target) {
  if (target.type == M5_SPIN || target.type == M5_NONE) {
    // その場に留まって動いてほしいので、追従制御はしない。
    return target.velocity;
  }
  float x_error = target.position.x - pos.x;
  float y_error = target.position.y - pos.y;
  float theta_error = target.position.theta - pos.theta;
  float sign = 1;
  if (theta_error < 0) {
    theta_error = -theta_error;
    sign = -1;
  }
  float cos_theta = arm_cos_f32(theta_error);
  float sin_theta = arm_sin_f32(theta_error) * sign;
  float v = target.velocity.v * cos_theta + M5_TRACKING_GAIN_Y * y_error;
  float omega =
      target.velocity.omega +
      target.velocity.v * (M5_TRACKING_GAIN_X * x_error +
                           M5_TRACKING_GAIN_THETA * sin_theta);
  return (m5Velocity){v, omega};
}
