//
// Created by ashkore on 23-7-25.
//

#ifndef BALANCE_CAR_WHEEL_H
#define BALANCE_CAR_WHEEL_H

#endif //BALANCE_CAR_WHEEL_H
#include "stm32f1xx_hal.h"

// Wheel struct
typedef struct {
    TIM_HandleTypeDef *encoder_tim;  // wheel encoder TIM
    TIM_HandleTypeDef *pwm_tim;  // wheel PWM TIM
    int pwm_channel;  // wheel PWM TIM channel, connect to L298N motor driver's ENA or ENB
    GPIO_TypeDef *motorpower1_gpio_port;  // L298N motor driver's GPIO INx
    int motorpower1_gpio_pin;  // L298N motor driver's GPIO INx
    GPIO_TypeDef *motorpower2_gpio_port;  // L298N motor driver's GPIO INx
    int motorpower2_gpio_pin;  // L298N motor driver's GPIO INx
    int speed;  // wheel speed
} WHEEL;

void update_speed(WHEEL *wheel);

void set_speed(WHEEL *wheel, int speed);