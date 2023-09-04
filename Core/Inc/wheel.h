//
// Created by ashkore on 23-9-4.
//



#ifndef BALANCE_CAR_WHEEL_H
#define BALANCE_CAR_WHEEL_H

#include "stm32f1xx_hal.h"


class Wheel {
private:
    TIM_HandleTypeDef *encoder_tim;  // wheel encoder TIM
    TIM_HandleTypeDef *pwm_tim;  // wheel PWM TIM
    int pwm_channel;  // wheel PWM TIM channel, connect to L298N motor driver's ENA or ENB
    GPIO_TypeDef *motorpower1_gpio_port;  // L298N motor driver's GPIO INx
    int motorpower1_gpio_pin;  // L298N motor driver's GPIO INx
    GPIO_TypeDef *motorpower2_gpio_port;  // L298N motor driver's GPIO INx
    int motorpower2_gpio_pin;  // L298N motor driver's GPIO INx
public:
    Wheel(TIM_HandleTypeDef *encoder_tim, TIM_HandleTypeDef *pwm_tim, int pwm_channel,
          GPIO_TypeDef *motorpower1_gpio_port, int motorpower1_gpio_pin,
          GPIO_TypeDef *motorpower2_gpio_port, int motorpower2_gpio_pin);

    void update_speed();

    void set_speed(int speed);

    int speed;
};


#endif //BALANCE_CAR_WHEEL_H
