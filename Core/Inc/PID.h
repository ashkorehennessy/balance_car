//
// Created by ashkore on 23-9-4.
//

#ifndef BALANCE_CAR_PID_H
#define BALANCE_CAR_PID_H

#include "stm32f1xx_hal.h"


class PID {
private:
    float last_error;
    float integral;
    float outmax;
    float outmin;
    uint8_t use_lowpass_filter;
    float lowpass_filter_factor;

public:
    float Kp;
    float Ki;
    float Kd;
    float last_out;

    PID(float Kp, float Ki, float Kd, float outmax, float outmin, uint8_t use_lowpass_filter = 0, float lowpass_filter_factor = 0.7);

    float calc(float input_value, float setpoint);

};


#endif //BALANCE_CAR_PID_H
