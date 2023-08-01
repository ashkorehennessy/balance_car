/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "mpu6050.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "wheel.h"
#include "pid.h"
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
MPU6050_t MPU6050;
WHEEL left_wheel;
WHEEL right_wheel;
PID left_wheel_stand_pid;
PID right_wheel_stand_pid;
PID left_wheel_speed_pid;
PID right_wheel_speed_pid;
PID left_wheel_turn_pid;
PID right_wheel_turn_pid;
int left_wheel_pidout;
int right_wheel_pidout;
float angle_offset;
float temp_speed_setpoint;
float speed_setpoint;
float angle_setpoint;
float turn_setpoint;
uint16_t pwm_max_arr;
uint16_t feedforward;
char buf[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void show_debug_info();
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Init MPU6050 */
  while(MPU6050_Init(&hi2c2) == 1);
  /* Init MPU6050 */

  /* Init SSD1306 */
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("SSD1306 OK", Font_11x18, White);
  ssd1306_UpdateScreen();
  /* Init SSD1306 */

  /* Init whells */
  left_wheel.encoder_tim = &htim2;
  left_wheel.pwm_tim = &htim1;
  left_wheel.pwm_channel = TIM_CHANNEL_1;
  left_wheel.motorpower1_gpio_port = GPIOB;
  left_wheel.motorpower1_gpio_pin = GPIO_PIN_14;
  left_wheel.motorpower2_gpio_port = GPIOB;
  left_wheel.motorpower2_gpio_pin = GPIO_PIN_15;
  left_wheel.speed = 0;

  right_wheel.encoder_tim = &htim3;
  right_wheel.pwm_tim = &htim1;
  right_wheel.pwm_channel = TIM_CHANNEL_4;
  right_wheel.motorpower1_gpio_port = GPIOB;
  right_wheel.motorpower1_gpio_pin = GPIO_PIN_13;
  right_wheel.motorpower2_gpio_port = GPIOB;
  right_wheel.motorpower2_gpio_pin = GPIO_PIN_12;
  right_wheel.speed = 0;
  /* Init whells */

  pwm_max_arr = __HAL_TIM_GET_AUTORELOAD(&htim1);  // Get PWM max value
  feedforward = pwm_max_arr * 0.05;  // ???
  angle_offset = 1.00f;  // angle offset, based on the mpu6050 placement
  speed_setpoint = 0;  // speed setpoint
  angle_setpoint = 0;  // angle setpoint

  /* Init wheel PID */
  left_wheel_stand_pid = PID_Init(60, 0, 380, pwm_max_arr, -pwm_max_arr);
  right_wheel_stand_pid = PID_Init(60, 0, 380, pwm_max_arr, -pwm_max_arr);
  left_wheel_speed_pid = PID_Init(-25, -0.1f, 0, pwm_max_arr*0.5, -pwm_max_arr*0.5);
  right_wheel_speed_pid = PID_Init(-25, -0.1f, 0, pwm_max_arr*0.5, -pwm_max_arr*0.5);
  left_wheel_turn_pid = PID_Init(0, 0, 0, pwm_max_arr*0.25, -pwm_max_arr*0.25);
  right_wheel_turn_pid = PID_Init(0, 0, 0, pwm_max_arr*0.25, -pwm_max_arr*0.25);
  /* Init wheel PID */

  /* Timers */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  /* Timers */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
     * Receive data from bluetooth serial and process
     * data format: s-12.00  (will set speed setpoint to -12.00)
     *             t12.00  (will set turn setpoint to 12.00)
     */
    HAL_UART_Receive(&huart2, (uint8_t*)buf, 20, 100);
    if(buf[0] == 's'){
      char *speed_buf = buf + 1;
      temp_speed_setpoint = atoff(speed_buf);
    }
    if(buf[0] == 't'){
      char *turn_buf = buf + 1;
      turn_setpoint = atoff(turn_buf);
    }
    angle_setpoint = temp_speed_setpoint * 0.1f;
    if(temp_speed_setpoint * turn_setpoint < 0){
      speed_setpoint = temp_speed_setpoint + turn_setpoint * 0.2f;
    } else if (temp_speed_setpoint * turn_setpoint > 0){
      speed_setpoint = temp_speed_setpoint - turn_setpoint * 0.2f;
    } else {
      speed_setpoint = temp_speed_setpoint;
    }

    // disable turn PID when stand still
    if(turn_setpoint == 0 && speed_setpoint == 0){
      left_wheel_turn_pid.Kp = 0;
      right_wheel_turn_pid.Kp = 0;
    } else {
      left_wheel_turn_pid.Kp = 6;
      right_wheel_turn_pid.Kp = 6;
    }

    show_debug_info();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void show_debug_info(){
  char ssd1306_buf[20];
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  sprintf(ssd1306_buf, "Speed:%.1f", speed_setpoint);
  ssd1306_WriteString(ssd1306_buf, Font_6x8, White);
  sprintf(ssd1306_buf, "Turn:%.1f", turn_setpoint);
  ssd1306_SetCursor(64, 0);
  ssd1306_WriteString(ssd1306_buf, Font_6x8, White);
  sprintf(ssd1306_buf, "l%.0f", left_wheel_stand_pid.last_out);
  ssd1306_SetCursor(0, 10);
  ssd1306_WriteString(ssd1306_buf, Font_6x8, White);
  sprintf(ssd1306_buf, "%.0f", left_wheel_speed_pid.last_out);
  ssd1306_SetCursor(44, 10);
  ssd1306_WriteString(ssd1306_buf, Font_6x8, White);
  sprintf(ssd1306_buf, "%.0f", left_wheel_turn_pid.last_out);
  ssd1306_SetCursor(86, 10);
  ssd1306_WriteString(ssd1306_buf, Font_6x8, White);
  sprintf(ssd1306_buf, "r%.0f", right_wheel_stand_pid.last_out);
  ssd1306_SetCursor(0, 20);
  ssd1306_WriteString(ssd1306_buf, Font_6x8, White);
  sprintf(ssd1306_buf, "%.0f", right_wheel_speed_pid.last_out);
  ssd1306_SetCursor(44, 20);
  ssd1306_WriteString(ssd1306_buf, Font_6x8, White);
  sprintf(ssd1306_buf, "%.0f", right_wheel_turn_pid.last_out);
  ssd1306_SetCursor(86, 20);
  ssd1306_WriteString(ssd1306_buf, Font_6x8, White);
  sprintf(ssd1306_buf, "ls%d", left_wheel.speed);
  ssd1306_SetCursor(0, 30);
  ssd1306_WriteString(ssd1306_buf, Font_6x8, White);
  sprintf(ssd1306_buf, "rs%d", right_wheel.speed);
  ssd1306_SetCursor(44, 30);
  ssd1306_WriteString(ssd1306_buf, Font_6x8, White);
  sprintf(ssd1306_buf, "df%d", left_wheel.speed - right_wheel.speed);
  ssd1306_SetCursor(86, 30);
  ssd1306_WriteString(ssd1306_buf, Font_6x8, White);
  sprintf(ssd1306_buf, "ax%.2f", MPU6050.KalmanAngleX);
  ssd1306_SetCursor(0, 40);
  ssd1306_WriteString(ssd1306_buf, Font_6x8, White);
  sprintf(ssd1306_buf, "ay%.2f", MPU6050.KalmanAngleY);
  ssd1306_SetCursor(44, 40);
  ssd1306_WriteString(ssd1306_buf, Font_6x8, White);
  sprintf(ssd1306_buf, "gz%.2f", MPU6050.Gz - 0.91);
  ssd1306_SetCursor(86, 40);
  ssd1306_WriteString(ssd1306_buf, Font_6x8, White);
  ssd1306_UpdateScreen();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
