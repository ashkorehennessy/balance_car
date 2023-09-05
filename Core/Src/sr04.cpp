//
// Created by ashkore on 23-9-2.
//

#include "sr04.h"

SR04::SR04(GPIO_TypeDef *trig_port, uint16_t trig_pin, TIM_HandleTypeDef *echo_htim, uint16_t echo_channel) {
    this->trig_port = trig_port;
    this->trig_pin = trig_pin;
    this->echo_htim = echo_htim;
    this->echo_channel = echo_channel;
    this->distance = 0;
    this->capture_flag = 0;
}

void SR04::init() {
    // Enable trigger pin
    HAL_GPIO_WritePin(this->trig_port, this->trig_pin, GPIO_PIN_RESET);
    // Set input capture edge to rising
    __HAL_TIM_SET_CAPTUREPOLARITY(this->echo_htim, this->echo_channel, TIM_INPUTCHANNELPOLARITY_RISING);
    // Set capture flag to 0
    this->capture_flag = 0;
    // Enable echo pin
    HAL_TIM_IC_Start_IT(this->echo_htim, this->echo_channel);
}

void SR04::trigger() {
    // Send pulse to trigger pin
    HAL_GPIO_WritePin(this->trig_port, this->trig_pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(this->echo_htim, this->echo_htim->Init.Period * 0.99);
    HAL_Delay(1);
    HAL_GPIO_WritePin(this->trig_port, this->trig_pin, GPIO_PIN_RESET);
}

void SR04::read_distance() {
    // This function should be called in the timer input capture callback
    static uint32_t start_counter;
    static uint32_t end_counter;
    switch (this->capture_flag) {
        case 0:
            start_counter = __HAL_TIM_GET_COUNTER(this->echo_htim);
            this->capture_flag = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(this->echo_htim, this->echo_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
            break;
        case 1:
            end_counter = __HAL_TIM_GET_COUNTER(this->echo_htim);
            if(end_counter < start_counter) {
                end_counter += 5000;
            }
            this->capture_flag = 0;
            // Calculate distance in mm
            this->distance = (end_counter - start_counter) * 340 / (SystemCoreClock / 1000000) / 2 /
                              (1000 / this->echo_htim->Init.Prescaler);
            __HAL_TIM_SET_CAPTUREPOLARITY(this->echo_htim, this->echo_channel, TIM_INPUTCHANNELPOLARITY_RISING);
            break;
    }
}