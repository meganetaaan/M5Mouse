#include <stm32f4xx_hal.h>
#include "test.h"
#include "global.h"

void m5test_straight(m5Mouse mouse) {
  HAL_Delay(300);
  float v = mouse->cap_velocity.v;
  m5mouse_straight(mouse, 1000, 0, v, 0);
  m5mouse_spin(mouse, 180);
}

void m5test_crank(m5Mouse mouse) {
  HAL_Delay(300);
  float v = mouse->cap_velocity.v;
  float omega = mouse->cap_velocity.omega;
  float alpha = mouse->cap_accel.alpha;
  m5mouse_straight(mouse, 270, 0, v, v);
  m5mouse_slalom(mouse, -90, M5_CELL_WIDTH * 0.5, v, alpha, omega);
  m5mouse_straight(mouse, 180, v, v, v);
  m5mouse_slalom(mouse, 90, M5_CELL_WIDTH * 0.5, v, alpha, omega);
  m5mouse_straight(mouse, 270, v, v, 0);
  m5mouse_spin(mouse, 180);
}

void m5test_calibrate(m5Mouse mouse) {
  HAL_Delay(300);
  m5gyro_calibrate(mouse->gyro);
}

void m5test_calibrate_wall(m5Mouse mouse) {
  HAL_Delay(300);
  m5wallsensor_calibrate(mouse->sensor);
}

void m5test_slalom(m5Mouse mouse) {
  float v = mouse->cap_velocity.v;
  m5mouse_straight(mouse, 90, 0, v, v);
  for (int i = 0; i < 3; i++) {
    m5mouse_straight(mouse, 180, v, v, v);
    m5mouse_slalom(mouse, 90, M5_CELL_WIDTH * 0.5, v, mouse->cap_accel.alpha,
                   mouse->cap_velocity.omega);
  }
  m5mouse_straight(mouse, 270, v, v, 0);
  m5mouse_spin(mouse, 90);
}

void m5test_zig_zag(m5Mouse mouse) {
  HAL_Delay(300);
  float v = mouse->cap_velocity.v;
  m5mouse_straight(mouse, 90, 0, v, v);
  for (int i = 0; i < 3; i++) {
    m5mouse_slalom(mouse, -90, M5_CELL_WIDTH * 0.5, v, mouse->cap_accel.alpha,
                   mouse->cap_velocity.omega);
    m5mouse_slalom(mouse, 90, M5_CELL_WIDTH * 0.5, v, mouse->cap_accel.alpha,
                   mouse->cap_velocity.omega);
  }
  m5mouse_straight(mouse, 90, v, v, 0);
  m5mouse_spin(mouse, -180);
}
