/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wheel.h"
#include "mpu6050.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int count = 0;
int speed_diff = 0;
float real_angle_setpoint = 0;
float real_speed_setpoint = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
float smooth_setpoint(float setpoint, float current_setpoint, float smooth_factor);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern WHEEL left_wheel;
extern WHEEL right_wheel;
extern MPU6050_t MPU6050;
extern PID left_wheel_stand_pid;
extern PID right_wheel_stand_pid;
extern PID left_wheel_speed_pid;
extern PID right_wheel_speed_pid;
extern PID left_wheel_turn_pid;
extern PID right_wheel_turn_pid;
extern int left_wheel_pidout;
extern int right_wheel_pidout;
extern float angle_offset;
extern float speed_setpoint;
extern float angle_setpoint;
extern float turn_setpoint;
extern uint16_t pwm_max_arr;
extern uint16_t feedforward;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
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
  * @brief This function handles Prefetch fault, memory access fault.
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
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  if(count == 16) {
    MPU6050_Read_All(&hi2c2, &MPU6050);
    update_speed(&left_wheel);
    update_speed(&right_wheel);

    // stand PID
    real_angle_setpoint = smooth_setpoint(angle_setpoint, real_angle_setpoint, 0.02f);
    left_wheel_pidout = PID_Calc(&left_wheel_stand_pid, MPU6050.KalmanAngleX, real_angle_setpoint + angle_offset);
    right_wheel_pidout = PID_Calc(&right_wheel_stand_pid, MPU6050.KalmanAngleX, real_angle_setpoint + angle_offset);

    // speed PID
    real_speed_setpoint = smooth_setpoint(speed_setpoint, real_speed_setpoint, 0.02f);
    left_wheel_pidout += PID_Calc(&left_wheel_speed_pid, left_wheel.speed, real_speed_setpoint);
    right_wheel_pidout += PID_Calc(&right_wheel_speed_pid, right_wheel.speed, real_speed_setpoint);

    // turn PID
    speed_diff = left_wheel.speed - right_wheel.speed;
    left_wheel_pidout += PID_Calc(&left_wheel_turn_pid, speed_diff, turn_setpoint);
    right_wheel_pidout -= PID_Calc(&right_wheel_turn_pid, speed_diff, turn_setpoint);

    // feedforward
    if (left_wheel_pidout > 0) {
      left_wheel_pidout += feedforward;
    } else if (left_wheel_pidout < 0) {
      left_wheel_pidout -= feedforward;
    }
    if (right_wheel_pidout > 0) {
      right_wheel_pidout += feedforward;
    } else if (right_wheel_pidout < 0) {
      right_wheel_pidout -= feedforward;
    }
    // set speed
    set_speed(&left_wheel, left_wheel_pidout);
    set_speed(&right_wheel, right_wheel_pidout);
    count = 0;
  } else {
    count++;
  }
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
float smooth_setpoint(float setpoint, float current_setpoint, float smooth_factor) {
  return (setpoint * smooth_factor + current_setpoint * (1 - smooth_factor));
}
/* USER CODE END 1 */
