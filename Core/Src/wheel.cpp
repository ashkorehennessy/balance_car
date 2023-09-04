//
// Created by ashkore on 23-9-4.
//

#include "wheel.h"

Wheel::Wheel(TIM_HandleTypeDef *encoder_tim, TIM_HandleTypeDef *pwm_tim, int pwm_channel,
             GPIO_TypeDef *motorpower1_gpio_port, int motorpower1_gpio_pin,
             GPIO_TypeDef *motorpower2_gpio_port, int motorpower2_gpio_pin) {
    this->encoder_tim = encoder_tim;
    this->pwm_tim = pwm_tim;
    this->pwm_channel = pwm_channel;
    this->motorpower1_gpio_port = motorpower1_gpio_port;
    this->motorpower1_gpio_pin = motorpower1_gpio_pin;
    this->motorpower2_gpio_port = motorpower2_gpio_port;
    this->motorpower2_gpio_pin = motorpower2_gpio_pin;
    this->speed = 0;
}

void Wheel::update_speed() {
    int tim_counter = __HAL_TIM_GET_COUNTER(this->encoder_tim);
    if (tim_counter > 32767) {
      tim_counter -= 65536;  // If tim_counter is negative, make it positive
    }
    this->speed = tim_counter;
    __HAL_TIM_SET_COUNTER(this->encoder_tim, 0);
}

void Wheel::set_speed(int _speed) {
    if (_speed > 0) {
        HAL_GPIO_WritePin(this->motorpower1_gpio_port, this->motorpower1_gpio_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(this->motorpower2_gpio_port, this->motorpower2_gpio_pin, GPIO_PIN_RESET);
    } else if (_speed < 0) {
        HAL_GPIO_WritePin(this->motorpower1_gpio_port, this->motorpower1_gpio_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(this->motorpower2_gpio_port, this->motorpower2_gpio_pin, GPIO_PIN_SET);
        _speed = -_speed;  // If speed is negative, make it positive
    }
    __HAL_TIM_SET_COMPARE(this->pwm_tim, this->pwm_channel, _speed);
}