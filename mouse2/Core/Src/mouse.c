#include "mouse.h"
#define ARM_MATH_CM7
#include <arm_math.h>

#define M5_FREQUENCY (1000.0f)
#define M5_DELTA (1 / M5_FREQUENCY)

// TODO: mouseオブジェクトのインスタンス変数にする
#define M5_WHEEL_RADIUS 24
#define M5_TREAD_WIDTH 40
#define M5_VBAT 7.4
m5MotionValue clamp(m5MotionValue min, m5MotionValue max, m5MotionValue value) {
  return fmin(fmax(value, min), max);
}

void m5mouse_update(m5Mouse mouse) {
  m5mouse_update_target_motion(mouse);
  m5mouse_update_current_motion(mouse);

  // TODO: PID制御に食わせて速度、角速度の目標値を算出

  // TODO: モータの電圧値を計算

  // TODO: motorを更新
}
void m5mouse_update_target_motion(m5Mouse mouse) {
  // tar_velを更新（加速度*微小時間）
  mouse->target_motion->vel = clamp(
      -mouse->cap_motion->vel, mouse->cap_motion->vel,
      mouse->target_motion->vel + mouse->current_motion->accel * M5_DELTA);
  mouse->target_motion->ang_vel = clamp(
      -mouse->cap_motion->ang_vel, mouse->cap_motion->ang_vel,
      mouse->target_motion->ang_vel + mouse->current_motion->ang_accel * M5_DELTA);
}

void m5mouse_update_current_motion(m5Mouse mouse) {
  // TODO: センサから情報を取得する
  int count_l = m5encoder_count(mouse->encoderL);
  int count_r = m5encoder_count(mouse->encoderR);

  // パルス数から車輪の速度への変換
  float vel_r =
      count_r * 2.0 * PI * M5_WHEEL_RADIUS / M5_PULSE_PER_ROTATE * M5_FREQUENCY;
  float vel_l =
      count_l * 2.0 * PI * M5_WHEEL_RADIUS / M5_PULSE_PER_ROTATE * M5_FREQUENCY;

  // 車輪の速度から車体の速度、角速度への変換
  float vel = (vel_r + vel_l) / 2;
  // TODO: 本来はジャイロセンサから取得したい値
  float ang_vel = (vel_r - vel_l) / M5_TREAD_WIDTH;

  // ローパスフィルタで更新
  mouse->current_motion->vel = mouse->current_motion->vel * 0.9 + vel * 0.1;
  mouse->current_motion->ang_vel = mouse->current_motion->ang_vel * 0.9 + vel * 0.1;

  return;
};

void m5mouse_set_operation(m5Mouse mouse) {
  // TODO
  return;
}