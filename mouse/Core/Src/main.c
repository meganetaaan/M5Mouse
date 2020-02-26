/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// extern void initialise_monitor_handles(void);
#define COMMAND_POWER_ON 0x00
#define COMMAND_POWER_OFF 0x01
#define COMMAND_READ_ENCODER_L 0x10
#define COMMAND_READ_ENCODER_R 0x11
#define COMMAND_READ_SENSOR_L 0x20
#define COMMAND_READ_SENSOR_FL 0x21
#define COMMAND_READ_SENSOR_FR 0x22
#define COMMAND_READ_SENSOR_R 0x23
#define COMMAND_WRITE_MOTOR_V_L 0x30
#define COMMAND_WRITE_MOTOR_V_R 0x31
#define COMMAND_WHO_AM_I 0x68

#define WHO_AM_I 0x01

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int16_t get_encoder_value(void)
{
  uint16_t enc_buff = TIM1->CNT;

  if (enc_buff > 32767)
  {
    return (int16_t)enc_buff * -1;
  }
  else
  {
    return (int16_t)enc_buff;
  }
}

void reset_encoder_count(void)
{
  TIM1->CNT = 0;
}

int16_t count_encoder(void)
{
  int16_t count = get_encoder_value();
  reset_encoder_count();
  return count;
}

void set_pwm(uint8_t channel, uint8_t direction, uint16_t pwm) {
  int dir = direction > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, dir);
  __HAL_TIM_SET_COMPARE(&htim2, channel, pwm);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // initialise_monitor_handles();

  char usr_buf[1000];
  sprintf(usr_buf, "Hello World\n\r");

  /* USER CODE END 1 */


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_ADC2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  uint16_t uhADCxConvertedValue = 0;
  uint8_t commandBuffer[4] = {0};
  uint16_t delay = 0;
  uint8_t whoami[1] = {WHO_AM_I};
  uint8_t status = 0;

  // Turn on LED
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  // Start ADC
  if (HAL_ADC_Start(&hadc2) != HAL_OK)
  {
    /* Start Conversation Error */
    Error_Handler();
  }

  // Start Encoder
  if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK) {
    Error_Handler();
  } // encoder start


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    if (HAL_I2C_Slave_Receive(&hi2c1, (uint8_t *)commandBuffer, 1, 1000) == HAL_OK) {
      switch(commandBuffer[0]) {
        case COMMAND_WHO_AM_I:
          status = HAL_I2C_Slave_Transmit(&hi2c1, (uint8_t *)whoami, 1, 1000);
          sprintf(usr_buf, "Send whoami: %d\n\r", status);
          break;
        default:
        ; // Do nothing
      }
    }
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    set_pwm(TIM_CHANNEL_1, 1, 100);
    set_pwm(TIM_CHANNEL_2, 1, 999);
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
      Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) {
      Error_Handler();
    }
    HAL_Delay(3000);

    set_pwm(TIM_CHANNEL_1, 1, 500);
    set_pwm(TIM_CHANNEL_2, 1, 500);
    HAL_Delay(3000);

    set_pwm(TIM_CHANNEL_1, 1, 999);
    set_pwm(TIM_CHANNEL_2, 1, 100);
    HAL_Delay(3000);

    if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1) != HAL_OK) {
      Error_Handler();
    }
    if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2) != HAL_OK) {
      Error_Handler();
    }
    // ADC
    /*
    HAL_StatusTypeDef status = HAL_ADC_PollForConversion(&hadc2, 1000);
    if (status != HAL_OK)
    {
      // sprintf(usr_buf, "Error: %d\n\r", status);
      CDC_Transmit_FS((uint8_t *)usr_buf, strlen(usr_buf));
      Error_Handler();
    }
    else
    {

      uhADCxConvertedValue = HAL_ADC_GetValue(&hadc2);
      sprintf(usr_buf, "Sensor: %d\n\r", uhADCxConvertedValue);
      delay = uhADCxConvertedValue >> 1;
      // CDC_Transmit_FS((uint8_t *)usr_buf, strlen(usr_buf));
    }
    */

    // Encoder
    // count += count_encoder();
    // sprintf(usr_buf, "Encoder: %d\n\r", count);
    // sprintf(usr_buf, "I2C Address: %d, Data: %d, %d, %d\n\r", commandBuffer[0], commandBuffer[1], commandBuffer[2], commandBuffer[3]);

    CDC_Transmit_FS((uint8_t *)usr_buf, strlen(usr_buf));
    HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
