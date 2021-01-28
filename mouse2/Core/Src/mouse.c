#include "mouse.h"

#include <controllers/pid.h>
// #include <controllers/command.h>
#define ARM_MATH_CM4
#include <arm_math.h>

#define M5_FREQUENCY (1000.0f)
#define M5_DELTA (1.0f / M5_FREQUENCY)

float clamp(float min, float max, float value) {
  return fmin(fmax(value, min), max);
}

// TODO: mouseオブジェクトのインスタンス変数にする
#define M5_WHEEL_RADIUS 12
#define M5_TREAD_WIDTH 40
#define M5_MAX_VOLTAGE 6
#define M5_VBAT (7.4f)

m5Motion m5motion_constructor(m5MotionValue vel, m5MotionValue ang_vel,
                              m5MotionValue accel, m5MotionValue ang_accel) {
  m5Motion motion = malloc(sizeof(m5MotionRecord));
  motion->accel = accel;
  motion->ang_accel = ang_accel;
  motion->vel = vel;
  motion->ang_vel = ang_vel;
  return motion;
}

m5Coordinate m5coordinate_constructor(m5MotionValue distance, m5MotionValue angle) {
  m5Coordinate coord = malloc(sizeof(m5CoordinateRecord));
  coord->distance = distance;
  coord->angle = angle;
  return coord;
}

void m5mouse_start(m5Mouse mouse) {
  m5encoder_start(mouse->encoder_l);
  m5encoder_start(mouse->encoder_r);
  m5gyro_start(mouse->gyro);
  m5motor_start(mouse->motor_l);
  m5motor_start(mouse->motor_r);
  m5sensor_start(mouse->sensor_l);
  m5sensor_start(mouse->sensor_fl);
  m5sensor_start(mouse->sensor_fr);
  m5sensor_start(mouse->sensor_r);
  mouse->active = 1;
}

void m5mouse_apply_velocity(m5Mouse mouse) {
  // PID制御に食わせて速度、角速度の目標値を算出
  // モータの電圧値を計算
  m5MotionValue ref_vel_l = mouse->target_motion->vel -
                            (mouse->target_motion->ang_vel / 2.0);
  m5MotionValue ref_vel_r = mouse->target_motion->vel +
                            (mouse->target_motion->ang_vel / 2.0);
  m5MotionValue vel_l = mouse->current_motion->vel -
                        (mouse->current_motion->ang_vel / 2.0);
  m5MotionValue vel_r = mouse->current_motion->vel +
                        (mouse->current_motion->ang_vel / 2.0);
  m5Value voltage_l = m5pid_update(mouse->controller_l, ref_vel_l, vel_l);
  m5Value voltage_r = m5pid_update(mouse->controller_r, ref_vel_r, vel_r);

  // モータを更新
  voltage_l = clamp(-M5_MAX_VOLTAGE, M5_MAX_VOLTAGE, voltage_l);
  voltage_r = clamp(-M5_MAX_VOLTAGE, M5_MAX_VOLTAGE, voltage_r);
  m5Value voltage_bat = M5_VBAT;
  m5motor_set_voltage(mouse->motor_l, voltage_l, voltage_bat);
  m5motor_set_voltage(mouse->motor_r, voltage_r, voltage_bat);
}

void m5mouse_update(m5Mouse mouse) {
  m5mouse_update_current_accel(mouse);
  m5mouse_update_current_velocity(mouse);
  m5mouse_update_current_orientation(mouse);
  m5mouse_update_target_velocity(mouse);
  m5mouse_apply_velocity(mouse);
}

void m5mouse_update_current_orientation(m5Mouse mouse) {
  mouse->current_coordinate->distance += mouse->current_motion->vel * M5_DELTA;
  mouse->current_coordinate->angle += mouse->current_motion->ang_vel * M5_DELTA;
}

M5ACCEL_OP m5command_next_operation(M5ACCEL_OP current_accel_op,
                                    m5Motion current_motion,
                                    m5Motion cap_motion,
                                    m5Coordinate current_coordinate,
                                    m5Coordinate target, float delta, M5RUN_OP current_run_op) {
  float dist = current_motion->vel * delta;
  float left = target->distance - current_coordinate->distance - dist;
  float angle_dist = current_motion->ang_vel * delta;
  float angle_left = target->angle - current_coordinate->angle - angle_dist;
  switch (current_run_op) {
    // TODO: 各MotionでI/F切ってswitch文をなくす
    case M5_SPIN:
        if (angle_left <= 0) {
          return M5_BRAKE;
        }
        if (current_motion->ang_vel >= cap_motion->ang_vel) {
          return M5_CONST;
        }
        return M5_ACCEL;
      break;
    case M5_STRAIGHT:
        if (left <= 0) {
          return M5_BRAKE;  // brake
        }
        // if (current_accel_op == M5_DECEL ||
        //     left < (float)(current_motion->vel * current_motion->vel -
        //                    cap_motion->vel * cap_motion->vel) /
        //                2.0 * current_motion->accel) {
        //   return M5_DECEL;
        // }
        if (current_motion->vel >= cap_motion->vel) {
          return M5_CONST;
        }
        return M5_ACCEL;
      break;
  }
  // TODO: マイナスの距離に対応する
}

void m5mouse_update_current_accel(m5Mouse mouse) {
  M5ACCEL_OP op = m5command_next_operation(
      mouse->current_accel_op, mouse->current_motion, mouse->cap_motion,
      mouse->current_coordinate, mouse->target_coordinate, M5_DELTA, mouse->current_run_op);
  switch (op) {
    case M5_ACCEL:
      mouse->current_motion->accel = mouse->current_run_op == M5_STRAIGHT ? mouse->target_motion->accel : 0;
      mouse->current_motion->ang_accel = mouse->current_run_op == M5_SPIN ? mouse->target_motion->ang_accel : 0;
      break;
    case M5_DECEL:
      mouse->current_motion->accel = mouse->current_run_op == M5_STRAIGHT ? -mouse->target_motion->accel : 0;
      mouse->current_motion->ang_accel = mouse->current_run_op == M5_SPIN ? -mouse->target_motion->ang_accel : 0;
      break;
    case M5_CONST:
      mouse->current_motion->accel = 0;
      mouse->current_motion->ang_accel = 0;
      break;
    case M5_BRAKE:
      mouse->current_motion->accel = 0;
      mouse->current_motion->ang_accel = 0;
      mouse->target_motion->vel = 0;
      mouse->target_motion->ang_vel = 0;
      break;
  }
  mouse->current_accel_op = op;
}

void m5mouse_update_target_velocity(m5Mouse mouse) {
  // tar_velを更新（加速度*微小時間）
  mouse->target_motion->vel = clamp(
      -mouse->cap_motion->vel, mouse->cap_motion->vel,
      mouse->target_motion->vel + mouse->current_motion->accel * M5_DELTA);
  mouse->target_motion->ang_vel =
      clamp(-mouse->cap_motion->ang_vel, mouse->cap_motion->ang_vel,
            mouse->target_motion->ang_vel +
                mouse->current_motion->ang_accel * M5_DELTA);
}

void m5mouse_update_current_velocity(m5Mouse mouse) {
  // TODO: センサから情報を取得する
  int count_l = m5encoder_count(mouse->encoder_l);
  int count_r = m5encoder_count(mouse->encoder_r);

  // パルス数から車輪の速度への変換
  float vel_r =
      (count_r * 2.0 * PI * M5_WHEEL_RADIUS * M5_FREQUENCY) / M5_PULSE_PER_ROTATE;
  float vel_l =
      (count_l * 2.0 * PI * M5_WHEEL_RADIUS * M5_FREQUENCY) / M5_PULSE_PER_ROTATE;

  // 車輪の速度から車体の速度、角速度への変換
  float vel = (vel_r + vel_l) / 2;
  // TODO: 本来はジャイロセンサから取得したい値
  float ang_vel = (vel_r - vel_l) / 2;

  // ローパスフィルタで更新
  mouse->current_motion->vel = mouse->current_motion->vel * 0.9 + vel * 0.1;
  mouse->current_motion->ang_vel =
      mouse->current_motion->ang_vel * 0.9 + ang_vel * 0.1;
  return;
};

void m5mouse_straight(m5Mouse mouse, float distance, float max_velocity, float end_velocity) {
  mouse->current_coordinate->distance = 0;
  mouse->current_coordinate->angle = 0;

  mouse->target_coordinate->distance = distance;
  mouse->target_coordinate->angle = 0;
  mouse->current_run_op = M5_STRAIGHT;

  mouse->current_accel_op = M5_ACCEL;
  while (mouse->current_accel_op != M5_BRAKE) {
    HAL_Delay(1);
  }
  return;
}

void m5mouse_spin(m5Mouse mouse, float angle) {
  mouse->current_coordinate->distance = 0;
  mouse->current_coordinate->angle = 0;

  mouse->target_coordinate->distance = 0;
  mouse->target_coordinate->angle = PI * M5_TREAD_WIDTH * (angle / 360);
  mouse->current_run_op = M5_SPIN;

  mouse->current_accel_op = M5_ACCEL;
  while (mouse->current_accel_op != M5_BRAKE) {
    HAL_Delay(1);
  }
  return;
}

void m5mouse_set_operation(m5Mouse mouse) {
  // TODO
  return;
}