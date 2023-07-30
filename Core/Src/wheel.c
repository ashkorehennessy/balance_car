//
// Created by ashkore on 23-7-25.
//

#include "wheel.h"

void update_speed(WHEEL *wheel) {
  int speed = __HAL_TIM_GET_COUNTER(wheel->encoder_tim);
  if(speed > 32767) {
    speed -= 65536;  // If speed is negative, make it positive
  }
  wheel->speed = speed;
  __HAL_TIM_SET_COUNTER(wheel->encoder_tim, 0);
}

void set_speed(WHEEL *wheel, int speed) {
  if (speed > 0) {
    HAL_GPIO_WritePin(wheel->motorpower1_gpiox, wheel->motorpower1_gpio_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(wheel->motorpower2_gpiox, wheel->motorpower2_gpio_pin, GPIO_PIN_RESET);
  } else if (speed < 0) {
    HAL_GPIO_WritePin(wheel->motorpower1_gpiox, wheel->motorpower1_gpio_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(wheel->motorpower2_gpiox, wheel->motorpower2_gpio_pin, GPIO_PIN_SET);
    speed = -speed;  // If speed is negative, make it positive
  } else {
    HAL_GPIO_WritePin(wheel->motorpower1_gpiox, wheel->motorpower1_gpio_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(wheel->motorpower2_gpiox, wheel->motorpower2_gpio_pin, GPIO_PIN_RESET);
  }
  __HAL_TIM_SET_COMPARE(wheel->pwm_tim, wheel->pwm_channel, speed);
}