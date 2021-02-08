#include "drivers/encoder.h"

int16_t m5encoder_get_value(TIM_HandleTypeDef *htim) {
  uint16_t enc_buff = htim->Instance->CNT;
  return enc_buff;
}

void m5encoder_reset_value(TIM_HandleTypeDef *htim) { htim->Instance->CNT = 0; }

int16_t m5encoder_count_encoder(TIM_HandleTypeDef *htim) {
  int16_t count = m5encoder_get_value(htim);
  m5encoder_reset_value(htim);
  return count;
}

void m5encoder_init(m5Encoder encoder) {
  // TODO: MX_TIM1_Initを参考にタイマーの初期化処理を行う
}

void m5encoder_start(m5Encoder encoder) {
  HAL_TIM_Encoder_Start(encoder->timer->handler, TIM_CHANNEL_ALL);
  encoder->active = 1;
}

void m5encoder_stop(m5Encoder encoder) {
  HAL_TIM_Encoder_Stop(encoder->timer->handler, TIM_CHANNEL_ALL);
  encoder->active = 0;
}

int16_t m5encoder_count(m5Encoder encoder) {
  int sign = encoder->direction ? 1 : -1;
  return m5encoder_count_encoder(encoder->timer->handler) * sign;
}
