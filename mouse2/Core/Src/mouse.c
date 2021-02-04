#include "mouse.h"

#include <arm_math.h>
#include <controllers/pid.h>
#include <stdlib.h>

#include "global.h"

#define to_radians(degrees) ((degrees)*PI / 180.0)
#define to_degrees(radians) ((radians)*180.0 / PI)

float clamp(float min, float max, float value) {
  return fmin(fmax(value, min), max);
}

// TODO: mouseオブジェクトのインスタンス変数にする
#define M5_WHEEL_RADIUS 12
#define M5_TREAD_WIDTH 40
#define M5_MAX_VOLTAGE 6
#define M5_VBAT (7.4f)

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
  m5Velocity target = mouse->target_velocity;
  m5Velocity current = mouse->current_velocity;
  float target_omega = target.omega * M5_TREAD_WIDTH / 4.0;
  float current_omega = current.omega * M5_TREAD_WIDTH / 4.0;
  float ref_vel_l = target.v + target_omega;
  float ref_vel_r = target.v - target_omega;
  float vel_l = current.v + current_omega;
  float vel_r = current.v - current_omega;

  // 壁補正の補正項を計算する
  m5Value voltage_wall = 0;
  if (mouse->is_wall_adjust_enabled && mouse->motion != NULL &&
      mouse->motion->type == M5_STRAIGHT &&
      (mouse->wall.left || mouse->wall.right)) {
    m5Value error = 0;
    if (mouse->wall.right && mouse->wall.left) {
      error = (mouse->wall.right_error - mouse->wall.left_error) *
              M5_WALL_ADJUST_GAIN;
    } else {
      error = ((mouse->wall.right_error - mouse->wall.left_error) * 2) *
              M5_WALL_ADJUST_GAIN;
    }
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
  m5mouse_update_velocity(mouse);
  m5mouse_update_position(mouse);
  m5mouse_update_target_velocity(mouse);
  m5mouse_apply_velocity(mouse);
}

void m5mouse_update_wallinfo(m5Mouse mouse) {
  mouse->wall = m5wallsensor_update(mouse->sensor);
  if (mouse->wall.front) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  }
  if (mouse->wall.right) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  }
}

void m5mouse_update_position(m5Mouse mouse) {
  mouse->position.y = mouse->current_velocity.v * M5_DELTA;
  mouse->position.theta = mouse->current_velocity.omega * M5_DELTA;
}

uint8_t m5mouse_is_moving(m5Mouse mouse) {
  return mouse->motion && !mouse->motion->is_end;
}

void m5mouse_update_target_velocity(m5Mouse mouse) {
  // tar_velを更新（加速度*微小時間）
  if (!m5mouse_is_moving(mouse)) {
    if (mouse->motion != NULL) {
      m5motion_destructor(mouse->motion);
      mouse->motion = NULL;
    }
    if (m5queue_is_empty(mouse->motion_queue)) {
      mouse->target_velocity = (m5Velocity){0, 0};
      return;
    } else {
      mouse->motion = m5motionqueue_dequeue(mouse->motion_queue);
    }
  }
  m5TrackTarget t = m5motion_get_next(mouse->motion);
  // TODO:
  mouse->target_velocity = t.velocity;
}

void m5mouse_update_velocity(m5Mouse mouse) {
  // TODO: センサから情報を取得する
  int count_l = m5encoder_count(mouse->encoder_l);
  int count_r = m5encoder_count(mouse->encoder_r);

  // パルス数から車輪の速度への変換
  float vel_r = (count_r * 2.0 * PI * M5_WHEEL_RADIUS * M5_FREQUENCY) /
                M5_PULSE_PER_ROTATE;
  float vel_l = (count_l * 2.0 * PI * M5_WHEEL_RADIUS * M5_FREQUENCY) /
                M5_PULSE_PER_ROTATE;

  // 車輪の速度から車体の速度、角速度への変換
  float vel = (vel_r + vel_l) / 2;
  m5gyro_update(mouse->gyro);
  float ang_vel = mouse->gyro->ang_vel;
  if (-1.0 < ang_vel && ang_vel < 1.0) {
    ang_vel = 0;
  }
  ang_vel = to_radians(ang_vel);
  m5Velocity v_meajured = (m5Velocity){vel, ang_vel};

  // ローパスフィルタで更新
  mouse->current_velocity =
      m5v_add(m5v_mul(mouse->current_velocity, 0.9), m5v_mul(v_meajured, 0.1));
  return;
};

void m5mouse_straight(m5Mouse mouse, float distance, float max_velocity,
                      float end_velocity) {
  m5Position destination = (m5Position){0, distance, 0};
  m5Velocity max = (m5Velocity){max_velocity, 0};
  m5Velocity end = (m5Velocity){end_velocity, 0};
  m5motionqueue_enqueue(
      mouse->motion_queue,
      m5motion_constructor(M5_STRAIGHT, mouse->current_velocity, max, end,
                           destination, mouse->cap_accel, M5_FREQUENCY));
  return;
}

void m5mouse_spin(m5Mouse mouse, float degrees) {
  m5Position destination = (m5Position){0, 0, to_radians(degrees)};
  m5motionqueue_enqueue(
      mouse->motion_queue,
      m5motion_constructor(M5_SPIN, (m5Velocity){0, 0}, mouse->cap_velocity,
                           (m5Velocity){0, 0}, destination, mouse->cap_accel,
                           M5_FREQUENCY));
  return;
}
