#include <stm32f4xx_hal.h>
#include "controllers/pid.h"

m5PIDController m5pid_constructor(float p, float i, float d, float saturation) {
  m5PIDConfiguration config = malloc(sizeof(m5PIDConfigurationRecord));
  config->p_gain = p;
  config->i_gain = i;
  config->d_gain = d;
  config->i_saturation = saturation;
  m5PIDContext ctx = malloc(sizeof(m5PIDContextRecord));
  ctx->initialized = 0;
  ctx->last_value = 0;
  ctx->integrated_value = 0;
  m5PIDController controller = malloc(sizeof(m5PIDControllerRecord));
  controller->config = config;
  controller->context = ctx;
  return controller;
}

m5Value m5pid_update(m5PIDController pid, m5Value target_value, m5Value current_value) {
  if (!pid->context->initialized) {
    pid->context->last_value = current_value;
    pid->context->initialized = 1;
  }
  m5Value diff = target_value - current_value;
  m5Value integrated = pid->context->integrated_value += diff;
  if (integrated > pid->config->i_saturation) {
    integrated = pid->config->i_saturation;
  } else if (integrated < -pid->config->i_saturation) {
    integrated = -pid->config->i_saturation;
  }
  m5Value derivative = current_value - pid->context->last_value;

  // calculate value
  float p = diff * pid->config->p_gain;
  float i = integrated * pid->config->i_gain;
  float d = derivative * pid->config->d_gain;
  pid->value = p + i + d;

  // update_context
  pid->context->integrated_value = integrated;
  pid->context->last_value = current_value;

  return pid->value;
}
