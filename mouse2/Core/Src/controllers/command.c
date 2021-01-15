#include <controllers/command.h>
#include <stm32f4xx_hal.h>

/**
 * 現在の座標と速度から次の加速度を出す
 **/
M5ACCEL_OP next_accel(M5ACCEL_OP current_op, m5Velocity current_velocity,
                      m5Velocity target_velocity, m5Coordinate current,
                      m5Coordinate target, uint16_t delta) {
  int cur_vel = current_velocity->velocity;
  int tar_vel = target_velocity->velocity;
  int dist = current_velocity->velocity * delta;
  int left = target->distance - current->distance - dist;
  if (left <= 0) {
    return M5_BRAKE;  // brake
  }
  // TODO: 残りの距離が足りなくなったら減速
  if (current_op == M5_DECEL ||
      left < (float)(cur_vel * cur_vel - tar_vel * tar_vel) / 2.0 * ACCEL) {
    return M5_DECEL;
  }
  if (current_velocity >= target_velocity) {
    return M5_CONST;
  }
  return M5_ACCEL;
}
