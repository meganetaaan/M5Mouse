#include "mouse.h"

#include <controllers/pid.h>
#define ARM_MATH_CM4
#include <arm_math.h>
#include <stdlib.h>

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
  m5wallsensor_start(mouse->sensor);
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
  // TODO: 壁補正の補正項を計算する
  m5Value voltage_wall = 0;
  if (mouse->current_run_op == M5_STRAIGHT && mouse->wall.left && mouse->wall.right) {
    m5Value error = (mouse->wall.right_error - mouse->wall.left_error) / 100.0;
    voltage_wall = m5pid_update(mouse->controller_wall, 0, error);
  } else {
    m5pid_reset(mouse->controller_wall);
  }
  m5Value voltage_l = m5pid_update(mouse->controller_l, ref_vel_l, vel_l);
  m5Value voltage_r = m5pid_update(mouse->controller_r, ref_vel_r, vel_r);

  // モータを更新
  voltage_l = clamp(-M5_MAX_VOLTAGE, M5_MAX_VOLTAGE, voltage_l + voltage_wall);
  voltage_r = clamp(-M5_MAX_VOLTAGE, M5_MAX_VOLTAGE, voltage_r - voltage_wall);
  m5Value voltage_bat = M5_VBAT;
  m5motor_set_voltage(mouse->motor_l, voltage_l, voltage_bat);
  m5motor_set_voltage(mouse->motor_r, voltage_r, voltage_bat);
}

void m5mouse_update(m5Mouse mouse) {
  m5mouse_update_wallinfo(mouse);
  m5mouse_update_current_velocity(mouse);
  m5mouse_update_current_orientation(mouse);
  m5mouse_update_target_velocity(mouse);
  m5mouse_apply_velocity(mouse);
}

void m5mouse_update_wallinfo(m5Mouse mouse) {
  mouse->wall = m5wallsensor_update(mouse->sensor);
}

void m5mouse_update_current_orientation(m5Mouse mouse) {
  mouse->current_coordinate->distance += mouse->current_motion->vel * M5_DELTA;
  mouse->current_coordinate->angle += mouse->current_motion->ang_vel * M5_DELTA;
}

uint8_t m5mouse_is_moving(m5Mouse mouse) {
  return mouse->motion && !mouse->motion->is_end;
}

void m5mouse_update_target_velocity(m5Mouse mouse) {
  // tar_velを更新（加速度*微小時間）
  if (!m5mouse_is_moving(mouse)) {
    mouse->target_motion->vel = 0;
    mouse->target_motion->ang_vel = 0;
    mouse->current_motion->accel = 0;
    mouse->current_motion->ang_accel = 0;
    return;
  }
  if (mouse->current_run_op == M5_STRAIGHT) {
    mouse->target_motion->vel = clamp(
      -mouse->cap_motion->vel,
      mouse->cap_motion->vel,
      m5motion2_get_next_velocity(mouse->motion));
    mouse->target_motion->ang_vel = 0;
  } else if (mouse->current_run_op == M5_SPIN) {
    mouse->target_motion->vel = 0;
    mouse->target_motion->ang_vel = clamp(
      -mouse->cap_motion->ang_vel,
      mouse->cap_motion->ang_vel,
      m5motion2_get_next_velocity(mouse->motion));
  } else if (mouse->current_run_op == M5_SLALOM) {
    /* TODO */
  } else if (mouse->current_run_op == M5_NONE) {
    /* DO NOTHING */
  }
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
  m5gyro_update(mouse->gyro);
  float ang_vel = mouse->gyro->ang_vel;
  if (-1.0 < ang_vel && ang_vel < 1.0) {
    ang_vel = 0;
  }
  ang_vel = (ang_vel / 360.0) * -1 * PI * M5_TREAD_WIDTH;

  // ローパスフィルタで更新
  mouse->current_motion->vel = mouse->current_motion->vel * 0.9 + vel * 0.1;
  mouse->current_motion->ang_vel =
      mouse->current_motion->ang_vel * 0.9 + ang_vel * 0.1;
  return;
};

void m5mouse_straight(m5Mouse mouse, float distance, float max_velocity, float end_velocity) {
  uint8_t direction = 1;
  if (distance < 0) {
    distance = -distance;
    direction = -1;
  }
  mouse->current_coordinate->distance = 0;
  mouse->current_coordinate->angle = 0;
  mouse->motion = m5motion2_constructor(mouse->current_motion->vel, max_velocity, end_velocity, distance, direction, mouse->cap_motion->accel, M5_FREQUENCY);
  mouse->current_run_op = M5_STRAIGHT;
  while(m5mouse_is_moving(mouse) && mouse->current_coordinate->distance < distance) {
    HAL_Delay(1);
  }
  m5motion2_destructor(mouse->motion);
  mouse->motion = NULL;
  return;
}

void m5mouse_spin(m5Mouse mouse, float angle) {
  uint8_t direction = 1;
  if (angle < 0) {
    angle = -angle;
    direction = -1;
  }
  angle = PI * M5_TREAD_WIDTH * (angle / 360);
  mouse->current_coordinate->distance = 0;
  mouse->current_coordinate->angle = 0;
  mouse->current_run_op = M5_SPIN;
  mouse->motion = m5motion2_constructor(0, mouse->cap_motion->ang_vel, 10, angle, direction, mouse->cap_motion->ang_accel, M5_FREQUENCY);
  while(m5mouse_is_moving(mouse) && mouse->current_coordinate->angle < angle) {
    HAL_Delay(1);
  }
  m5motion2_destructor(mouse->motion);
  mouse->motion = NULL;
  return;
}

void m5mouse_set_operation(m5Mouse mouse) {
  // TODO
  return;
}