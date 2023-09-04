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
#include <cstdio>
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wheel.h"
#include "PID.h"
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
Wheel left_wheel(&htim2, &htim1, TIM_CHANNEL_1, GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15);
Wheel right_wheel(&htim3, &htim1, TIM_CHANNEL_4, GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_12);

PID left_wheel_stand_pid(43, 0, 200, 65535, -65535);
PID right_wheel_stand_pid(43, 0, 200, 65535, -65535);
PID left_wheel_speed_pid(-39, 0, 0, 65535, -65535);
PID right_wheel_speed_pid(-39, 0, 0, 65535, -65535);
PID left_wheel_turn_pid(0, 0, 0, 1000, -1000);
PID right_wheel_turn_pid(0, 0, 0, 1000, -1000);
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

  pwm_max_arr = __HAL_TIM_GET_AUTORELOAD(&htim1);  // Get PWM max value
  feedforward = pwm_max_arr * 0.00;  // ???
  angle_offset = 1.5f;  // angle offset, based on the mpu6050 placement
  speed_setpoint = 0;  // speed setpoint
  angle_setpoint = 0;  // angle setpoint

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
      speed_setpoint = temp_speed_setpoint + turn_setpoint * 0.03f;
    } else if (temp_speed_setpoint * turn_setpoint > 0){
      speed_setpoint = temp_speed_setpoint - turn_setpoint * 0.03f;
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

    // if angle not a number, reset system
    if(is_nan(MPU6050.KalmanAngleX)){
      NVIC_SystemReset();
    }

    // re-stand when fall
    if(MPU6050.KalmanAngleX > 45){
      HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
      left_wheel.set_speed(600);
      right_wheel.set_speed(600);
      HAL_Delay(700);
      HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
      HAL_Delay(1000);
    } else if (MPU6050.KalmanAngleX < -45){
      HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
      left_wheel.set_speed(-600);
      right_wheel.set_speed(-600);
      HAL_Delay(700);
      HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
      HAL_Delay(1000);
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

uint8_t is_nan(double x){
  return x != x;
}

float atoff(char *str){
  float result = 0;
  float decimal = 0;
  int i = 0;
  int sign = 1;
  if(str[0] == '-'){
    sign = -1;
    i++;
  }
  while(str[i] != '\0'){
    if(str[i] == '.'){
      decimal = 1;
      i++;
      continue;
    }
    if(decimal == 0){
      result = result * 10 + str[i] - '0';
    } else {
      result = result + (str[i] - '0') * pow(10, -decimal);
      decimal++;
    }
    i++;
  }
  return result * sign;
}

double pow(double x, int y){
  double result = 1;
  if(y == 0){
    return 1;
  }
  if(y > 0){
    for(int i = 0; i < y; i++){
      result *= x;
    }
  } else {
    for(int i = 0; i < -y; i++){
      result /= x;
    }
  }
  return result;
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
