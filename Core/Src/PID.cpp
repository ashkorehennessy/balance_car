//
// Created by ashkore on 23-9-4.
//

#include "PID.h"

PID::PID(float Kp, float Ki, float Kd, float outmax, float outmin, uint8_t use_lowpass_filter, float lowpass_filter_factor) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->last_error = 0;
    this->last_out = 0;
    this->integral = 0;
    this->outmax = outmax;
    this->outmin = outmin;
    this->use_lowpass_filter = use_lowpass_filter;
    this->lowpass_filter_factor = lowpass_filter_factor;
}

float PID::calc(float input_value, float setpoint) {
    float error = setpoint - input_value;
    float derivative = error - this->last_error;
    this->integral += error;
    this->last_error = error;
    float output = this->Kp * error + this->Ki * this->integral + this->Kd * derivative;

    // Integral limit
    if (this->integral > this->outmax / 100) {
        this->integral = this->outmax / 100;
    } else if (this->integral < this->outmin / 100) {
        this->integral = this->outmin / 100;
    }

    // Output limit
    if (output > this->outmax) {
        output = this->outmax;
    } else if (output < this->outmin) {
        output = this->outmin;
    }

    // Low pass filter
    if (this->use_lowpass_filter) {
        output = this->last_out * this->lowpass_filter_factor + output * (1 - this->lowpass_filter_factor);
    }

    this->last_out = output;

    return output;
}