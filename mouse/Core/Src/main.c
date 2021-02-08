/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <controllers/pid.h>
#include <delay_us.h>
#include <drivers/encoder.h>
#include <drivers/gyro.h>
#include <drivers/motor.h>
#include <drivers/sensor.h>
#include <global.h>
#include <mouse.h>
#include <stdio.h>
#include <stdlib.h>
// #define ARM_MATH_CM4
#include <arm_math.h>

#include "controllers/motion.h"
#include "maze/agent.h"
#include "maze/maze.h"
#include "maze/maze_data.h"

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

extern void initialise_monitor_handles(void);
extern uint32_t m5timerCount;
extern m5Mouse mouse;
extern uint8_t m5transferRequested;
extern uint8_t m5transferDirection;
extern uint8_t m5i2cbuffer[256];

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */


/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */
  // #ifdef DEBUG
  // initialise_monitor_handles();
  // #endif


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  // 開始
  m5main_init_mouse();
  m5mouse_start(mouse);

  // 割り込みハンドラの開始
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim6);
  m5Maze maze = m5maze_constructor();
  m5MazeAgent agent = m5mazeagent_constructor(maze, mouse);
  HAL_Delay(500);
  // m5mouse_slalom(mouse, 90, mouse->cap_velocity.v, mouse->cap_accel.alpha, mouse->cap_velocity.omega);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)

  {
    // printf("transfer complete. count: %d\n", count);
    HAL_StatusTypeDef status =
        HAL_I2C_Slave_Receive(&hi2c1, m5i2cbuffer, 1, 200);
    if (status == HAL_OK) {
      switch (m5i2cbuffer[0]) {
        case M5_REGISTER_WHO_AM_I:
          m5i2cbuffer[0] = M5_VALUE_WHO_AM_I;
          HAL_I2C_Slave_Transmit(&hi2c1, m5i2cbuffer, 1, 100);
          break;
        case M5_REGISTER_TEST:
          m5i2cbuffer[0] = m5i2c_count++;
          HAL_I2C_Slave_Transmit(&hi2c1, m5i2cbuffer, 1, 100);
          // m5wallsensor_calibrate(mouse->sensor);
          HAL_Delay(300);
          m5gyro_calibrate(mouse->gyro);
          HAL_Delay(200);
          // m5agent_search_run(agent, (m5Index){7, 8});
          float v = mouse->cap_velocity.v;
          /*
          m5mouse_straight(mouse, 60, v, 0);
          m5mouse_spin(mouse, 30);
          m5mouse_spin(mouse, -30);
          */
          // m5mouse_slalom(mouse, -90, M5_CELL_WIDTH * 0.5, v, mouse->cap_accel.alpha, mouse->cap_velocity.omega);
          // ouse_slalom(mouse, -90, M5_CELL_WIDTH * 0.5, v, mouse->cap_accel.alpha, mouse->cap_velocity.omega);
          m5mouse_straight(mouse, 90, 0, v, v);
          // m5mouse_straight(mouse, 90, v, v, 0);
          // m5mouse_slalom(mouse, -90, M5_CELL_WIDTH * 0.5, v, mouse->cap_accel.alpha, mouse->cap_velocity.omega);
          // m5mouse_slalom(mouse, 90, v, mouse->cap_accel.alpha, mouse->cap_velocity.omega);
          for (int i = 0; i < 3; i++) {
            // m5mouse_straight(mouse, 180, v, v, v);
            // m5mouse_straight(mouse, 90, v, v, 0);
            // m5mouse_spin(mouse, 90);
            // m5mouse_straight(mouse, 90, 0, v, v);
            m5mouse_slalom(mouse, -90, M5_CELL_WIDTH * 0.5, v, mouse->cap_accel.alpha, mouse->cap_velocity.omega);
            m5mouse_slalom(mouse, 90, M5_CELL_WIDTH * 0.5, v, mouse->cap_accel.alpha, mouse->cap_velocity.omega);
          }
          m5mouse_straight(mouse, 90, v, v, 0);
          m5mouse_spin(mouse, -180);
          HAL_Delay(300);
          break;
        case M5_REGISTER_CALIBRATE:
          // TODO:
          m5wallsensor_calibrate(mouse->sensor);
          HAL_Delay(300);
          break;
        default:
          m5i2cbuffer[0] = 0xFF;
      }
      while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
        ;
    } else {
      // printf("error %d\n", status);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {
  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 200;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {
  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 42 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {
  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {
  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void) {
  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 84 - 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 250 - 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {
  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_12,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA,
                    GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void m5main_init_mouse(void) {
  // DRV8835のモード指定（PHASE/ENBLモード）
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

  // モータ初期化
  m5TimerConfiguration timerMotorL = malloc(sizeof(m5TimerConfigurationRecord));
  timerMotorL->handler = &htim3;
  timerMotorL->channel = TIM_CHANNEL_1;
  m5Motor motorL = malloc(sizeof(m5MotorRecord));
  motorL->dir_port = GPIOA;
  motorL->dir_pin = GPIO_PIN_10;
  motorL->direction = 0;
  motorL->timer = timerMotorL;
  m5motor_init(motorL);
  mouse->motor_l = motorL;

  m5TimerConfiguration timerMotorR = malloc(sizeof(m5TimerConfigurationRecord));
  timerMotorR->handler = &htim3;
  timerMotorR->channel = TIM_CHANNEL_2;
  m5Motor motorR = malloc(sizeof(m5MotorRecord));
  motorR->dir_port = GPIOA;
  motorR->dir_pin = GPIO_PIN_11;
  motorR->direction = 1;
  motorR->timer = timerMotorR;
  m5motor_init(motorR);
  mouse->motor_r = motorR;

  // IMUの初期化
  HAL_Delay(50);
  m5SPIConfiguration spi = malloc(sizeof(m5SPIConfigurationRecord));
  spi->cs_pin = GPIO_PIN_12;
  spi->cs_port = GPIOC;
  spi->handler = &hspi1;
  m5Gyro gyro = malloc(sizeof(m5GyroRecord));
  gyro->spi = spi;
  m5gyro_init(gyro);
  mouse->gyro = gyro;

  // エンコーダの初期化
  m5Encoder encoderL = malloc(sizeof(m5EncoderRecord));
  m5Encoder encoderR = malloc(sizeof(m5EncoderRecord));
  m5TimerConfiguration timerL = malloc(sizeof(m5TimerConfigurationRecord));
  m5TimerConfiguration timerR = malloc(sizeof(m5TimerConfigurationRecord));
  timerL->handler = &htim1;
  encoderL->timer = timerL;
  encoderL->direction = 1;
  m5encoder_init(encoderL);
  mouse->encoder_l = encoderL;
  timerR->handler = &htim4;
  encoderR->timer = timerR;
  encoderR->direction = 0;
  m5encoder_init(encoderR);
  mouse->encoder_r = encoderR;

  m5AnalogConfiguration adRec_l = malloc(sizeof(m5AnalogConfigurationRecord));
  adRec_l->handler = &hadc1;
  adRec_l->channel = (ADC_ChannelConfTypeDef *)ADC_CHANNEL_1;
  m5Sensor sensor_l = malloc(sizeof(m5SensorRecord));
  sensor_l->analog = adRec_l;
  sensor_l->led_port = GPIOC;
  sensor_l->led_pin = GPIO_PIN_5;
  m5sensor_init(sensor_l);

  m5AnalogConfiguration adRec_fl = malloc(sizeof(m5AnalogConfigurationRecord));
  adRec_fl->handler = &hadc1;
  adRec_fl->channel = (ADC_ChannelConfTypeDef *)ADC_CHANNEL_0;
  m5Sensor sensor_fl = malloc(sizeof(m5SensorRecord));
  sensor_fl->analog = adRec_fl;
  sensor_fl->led_port = GPIOC;
  sensor_fl->led_pin = GPIO_PIN_4;
  m5sensor_init(sensor_fl);

  m5AnalogConfiguration adRec_fr = malloc(sizeof(m5AnalogConfigurationRecord));
  adRec_fr->handler = &hadc1;
  adRec_fr->channel = (ADC_ChannelConfTypeDef *)ADC_CHANNEL_3;
  m5Sensor sensor_fr = malloc(sizeof(m5SensorRecord));
  sensor_fr->analog = adRec_fr;
  sensor_fr->led_port = GPIOB;
  sensor_fr->led_pin = GPIO_PIN_1;
  m5sensor_init(sensor_fr);

  m5AnalogConfiguration adRec_r = malloc(sizeof(m5AnalogConfigurationRecord));
  adRec_r->handler = &hadc1;
  adRec_r->channel = (ADC_ChannelConfTypeDef *)ADC_CHANNEL_2;
  m5Sensor sensor_r = malloc(sizeof(m5SensorRecord));
  sensor_r->analog = adRec_r;
  sensor_r->led_port = GPIOB;
  sensor_r->led_pin = GPIO_PIN_0;
  m5sensor_init(sensor_r);

  m5WallSensor ws =
      m5wallsensor_constructor(sensor_l, sensor_fl, sensor_fr, sensor_r);
  mouse->sensor = ws;

  m5PIDController controller_l =
      m5pid_constructor(M5DEFAULT_PGAIN, M5DEFAULT_IGAIN, M5DEFAULT_DGAIN, M5DEFAULT_SATURATION);
  m5PIDController controller_r =
      m5pid_constructor(M5DEFAULT_PGAIN, M5DEFAULT_IGAIN, M5DEFAULT_DGAIN, M5DEFAULT_SATURATION);
  m5PIDController controller_wall = m5pid_constructor(
      M5DEFAULT_WALL_PGAIN, M5DEFAULT_WALL_IGAIN, M5DEFAULT_WALL_DGAIN, 10);
  mouse->controller_l = controller_l;
  mouse->controller_r = controller_r;
  mouse->controller_wall = controller_wall;
  mouse->motion_queue = m5queue_constructor();

  mouse->current_velocity = (m5Velocity){0, 0};
  mouse->target_velocity = (m5Velocity){0, 0};
  mouse->cap_velocity = (m5Velocity){M5_VELOCITY_SEARCH, M5_ANG_VELOCITY};
  mouse->cap_accel = (m5Accel){M5_ACCEL_SEARCH, M5_ANG_ACCEL};
  mouse->position = (m5Position){0, 0};
  mouse->is_wall_adjust_enabled = 0;
  mouse->odometry = m5odometry_constructor(M5_DELTA);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
    /* USER CODE END WHILE */

    int i = 0;
    GPIOA->BSRR = GPIO_PIN_15;
    for (i = 0; i < 40000000; i++) {
    }
    GPIOA->BSRR = (uint32_t)GPIO_PIN_15 << 16U;
    for (i = 0; i < 40000000; i++) {
    }
    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
