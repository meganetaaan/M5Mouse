#include <drivers/sensor.h>
#include <delay_us.h>

void m5sensor_init(m5Sensor sensor) {
  // m5AnalogConfiguration ad;
  HAL_GPIO_WritePin(sensor->led_port, sensor->led_pin, GPIO_PIN_RESET);
  sensor->active = 0;
}

void m5sensor_start(m5Sensor sensor) {
  // HAL_ADC_Start(sensor->analog->handler);
  sensor->active = 1;
}

uint16_t m5sensor_read(m5Sensor sensor) {
  ADC_HandleTypeDef *handler = sensor->analog->handler;
  ADC_ChannelConfTypeDef sConfig = {0};
  // config
  sConfig.Channel = sensor->analog->channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(handler, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  HAL_GPIO_WritePin(sensor->led_port, sensor->led_pin, GPIO_PIN_SET);
  delay_us(1);
  HAL_ADC_Start(handler);
  HAL_ADC_PollForConversion(handler, 1000);
  uint16_t adcValueOn = HAL_ADC_GetValue(handler);

  HAL_GPIO_WritePin(sensor->led_port, sensor->led_pin, GPIO_PIN_RESET);
  delay_us(1);
  HAL_ADC_Start(handler);
  HAL_ADC_PollForConversion(handler, 1000);
  uint16_t adcValueOff = HAL_ADC_GetValue(handler);
  sensor->value = adcValueOn - adcValueOff;
  return sensor->value;
}

void m5sensor_stop(m5Sensor sensor) {
  HAL_ADC_Stop(sensor->analog->handler);
  sensor->active = 0;
}
