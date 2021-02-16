/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "global.h"
#include "mouse.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_HAL_I2C_REGISTER_CALLBACKS 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */
extern uint32_t m5TimerCount;
extern m5Mouse mouse;
extern uint8_t m5transferRequested;
extern uint8_t m5transferDirection;
extern uint16_t m5sensor_count;
// extern uint8_t i2cbuffer[];
// extern uint16_t m5i2c_count;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    GPIOA->BSRR = GPIO_PIN_15;
    for(int i = 0; i < 4000000; i++){
    }
    GPIOA->BSRR = (uint32_t)GPIO_PIN_15 << 16U;
    for(int i = 0; i < 4000000; i++){
    }
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  if (mouse->active) {
    m5mouse_update(mouse);
  }
  m5timerCount++;
  if (m5timerCount % 500 == 0) {
    // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
    // printf("\n\n");
    // printf("current_accel_op: %d\n", mouse->current_accel_op);
    // printf("current_corrdinate->distance: %f, angle: %f, target_coordinate->distance: %f, angle: %f\n", mouse->current_coordinate->distance, mouse->current_coordinate->angle, mouse->target_coordinate->distance, mouse->target_coordinate->angle);
    // printf("current_motion->vel: %f, ang_vel: %f, accel: %f, ang_accel: %f\n", mouse->current_motion->vel, mouse->current_motion->ang_vel, mouse->current_motion->accel, mouse->current_motion->ang_accel);
    // printf("target_motion->vel: %f, ang_vel: %f, accel: %f, ang_accel: %f\n", mouse->target_motion->vel, mouse->target_motion->ang_vel, mouse->target_motion->accel, mouse->target_motion->ang_accel);
  }

  /*
  if (m5timerCount % 100 == 0) {
    printf("\n\n");
    printf("current_accel_op: %d\n", mouse->current_accel_op);
    printf("current_motion->vel: %f, ang_vel: %f, accel: %f, ang_accel: %f\n", mouse->current_motion->vel, mouse->current_motion->ang_vel, mouse->current_motion->accel, mouse->current_motion->ang_accel);
    printf("current_motion->vel: %f, ang_vel: %f, accel: %f, ang_accel: %f\n", mouse->current_motion->vel, mouse->current_motion->ang_vel, mouse->current_motion->accel, mouse->current_motion->ang_accel);
  }
  */
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
  if (mouse->active) {
    m5sensor_count++;
    switch(m5sensor_count % 4) {
      case 0:
        m5sensor_read(mouse->sensor->sensor_l);
        break;
      case 1:
        m5sensor_read(mouse->sensor->sensor_fr);
        break;
      case 2:
        m5sensor_read(mouse->sensor->sensor_fl);
        break;
      case 3:
        m5sensor_read(mouse->sensor->sensor_r);
        break;
    }
  }
  // if (m5sensor_count % 4000 == 0) {
  //   printf(
  //       "m5sensor_count: %u, gyro: %f, sensor...l: %u, fl: %u, fr: %u, r: %u\r\n",
  //       m5sensor_count, mouse->gyro->ang_vel, mouse->sensor->sensor_l->value,
  //       mouse->sensor->sensor_fl->value, mouse->sensor->sensor_fr->value,
  //       mouse->sensor->sensor_r->value);
  // }

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
                          uint16_t AddrMatchCode) {
  UNUSED(AddrMatchCode);
  // if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
  //   // HAL_I2C_Slave_Sequential_Receive_IT(&hi2c1, i2cbuffer, 1, I2C_FIRST_FRAME);
  //   HAL_I2C_Slave_Receive(hi2c, m5i2cbuffer, 1, 1000);
  //   switch (m5i2cbuffer[0]) {
  //     case M5_REGISTER_WHO_AM_I:
  //       m5i2cbuffer[0] = M5_VALUE_WHO_AM_I;
  //       break;
  //     case M5_REGISTER_TEST:
  //       m5i2cbuffer[0] = m5i2c_count++;
  //       break;
  //     default:
  //       m5i2cbuffer[0] = 0xFF;
  //   }
  // } else if (TransferDirection == I2C_DIRECTION_RECEIVE) {
  //   HAL_I2C_Slave_Transmit(hi2c, m5i2cbuffer, 1, 1000);
  //   // HAL_I2C_Slave_Sequential_Transmit_IT(&hi2c1, i2cbuffer, 1, I2C_LAST_FRAME);
  // }
  // hi2c->State = HAL_I2C_STATE_READY;
  // HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
  hi2c->State = HAL_I2C_STATE_READY;
  HAL_I2C_EnableListen_IT(hi2c);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
