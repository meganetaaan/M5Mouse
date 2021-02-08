#include "drivers/motor.h"

#include <stdio.h>

void m5motor_init(m5Motor motor) {
  // TODO: initialization
  motor->active = 0;
  motor->duty = 0;
}

void m5motor_start(m5Motor motor) {
  // TODO: Error handler
  HAL_TIM_PWM_Start(motor->timer->handler, motor->timer->channel);
  motor->active = 1;
}

void m5motor_stop(m5Motor motor) {
  // TODO: Error handler
  HAL_TIM_PWM_Stop(motor->timer->handler, motor->timer->channel);
  motor->active = 0;
}

void m5motor_set_voltage(m5Motor motor, float voltage, float battery_voltage) {
  uint8_t direction = 1;
  if (voltage < 0) {
    voltage = -voltage;
    direction = 0;
  }
  uint16_t pwm = voltage * 1000 / battery_voltage;
  m5motor_set_pwm(motor, direction, pwm);
}

void m5motor_set_pwm(m5Motor motor, uint8_t direction, uint16_t pwm) {
  GPIO_PinState state = (motor->direction ^ direction) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, state);

  __HAL_TIM_SET_COMPARE(motor->timer->handler, motor->timer->channel, pwm);
  // TIM_OC_InitTypeDef sConfigOC = {0};
  // sConfigOC.OCMode = TIM_OCMODE_PWM1;
  // sConfigOC.Pulse = pwm;
  // sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  // sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  // if (HAL_TIM_PWM_ConfigChannel(motor->timer->handler, &sConfigOC, motor->timer->channel) != HAL_OK) {
  //   return;
  // }
  motor->duty = pwm;

  /*
    // TODO: HALの内部関数を使わない方式に差し替え
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    { return;
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    { return;
    }
    }
  */
}
