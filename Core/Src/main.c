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
uint8_t MPU6050_OK = 0;
int temp;
int pidout;
WHEEL left_wheel;
WHEEL right_wheel;
PID left_wheel_stand_pid;
PID right_wheel_stand_pid;
PID left_wheel_speed_pid;
PID right_wheel_speed_pid;
int left_wheel_pidout;
int right_wheel_pidout;
uint16_t pwm_max_arr;
uint16_t feedforward;
char buf[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* Init MPU6050 */
  while(MPU6050_Init(&hi2c2) == 1);
  /* Init MPU6050 */

  /* Init SSD1306 */
//  ssd1306_Init();
//  ssd1306_Fill(Black);
//  ssd1306_SetCursor(0, 0);
//  ssd1306_WriteString("MPU6050 OK", Font_11x18, White);
//  ssd1306_UpdateScreen();
  /* Init SSD1306 */

  /* Init whells */
  left_wheel.encoder_tim = &htim2;
  left_wheel.pwm_tim = &htim1;
  left_wheel.pwm_channel = TIM_CHANNEL_1;
  left_wheel.motorpower1_gpiox = GPIOB;
  left_wheel.motorpower1_gpio_pin = GPIO_PIN_14;
  left_wheel.motorpower2_gpiox = GPIOB;
  left_wheel.motorpower2_gpio_pin = GPIO_PIN_15;
  left_wheel.speed = 0;

  right_wheel.encoder_tim = &htim3;
  right_wheel.pwm_tim = &htim1;
  right_wheel.pwm_channel = TIM_CHANNEL_4;
  right_wheel.motorpower1_gpiox = GPIOB;
  right_wheel.motorpower1_gpio_pin = GPIO_PIN_13;
  right_wheel.motorpower2_gpiox = GPIOB;
  right_wheel.motorpower2_gpio_pin = GPIO_PIN_12;
  right_wheel.speed = 0;
  /* Init whells */

  pwm_max_arr = __HAL_TIM_GET_AUTORELOAD(&htim1);  // Get PWM max value
  feedforward = pwm_max_arr * 0.095;  // This wheel needs more than 9.5% PWM to start

  /* Init wheel PID */
  left_wheel_stand_pid = PID_Init(45, 0, 280, pwm_max_arr, -pwm_max_arr);
  right_wheel_stand_pid = PID_Init(45, 0, 280, pwm_max_arr, -pwm_max_arr);
  left_wheel_speed_pid = PID_Init(-23, -0.115, 0, pwm_max_arr, -pwm_max_arr);
  right_wheel_speed_pid = PID_Init(-23, -0.115, 0, pwm_max_arr, -pwm_max_arr);
  /* Init wheel PID */

  /* Timers */
  HAL_TIM_Base_Start_IT(&htim4);
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
//    sprintf(buf,"speed:%d,%d,%d\r\n",left_wheel.speed,right_wheel.speed,temp);
//    HAL_UART_Transmit(&huart1,(uint8_t*)buf,strlen(buf),1000);
    sprintf(buf,"%.2f,%.2f,%d,%d,%d,%d\r\n",MPU6050.KalmanAngleX,MPU6050.KalmanAngleY,left_wheel.speed,left_wheel_pidout,right_wheel.speed,right_wheel_pidout);
    HAL_UART_Transmit(&huart1,(uint8_t*)buf,strlen(buf),1000);
    HAL_Delay(8);
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
