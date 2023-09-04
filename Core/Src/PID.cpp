//
// Created by ashkore on 23-9-4.
//

#include "PID.h"

PID::PID(float Kp, float Ki, float Kd, float outmax, float outmin) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->last_error = 0;
    this->last_out = 0;
    this->integral = 0;
    this->outmax = outmax;
    this->outmin = outmin;
}

float PID::calc(float input_value, float setpoint) {
    float error = setpoint - input_value;
    float derivative = error - this->last_error;
    this->integral += error;
    this->last_error = error;
    float output = this->Kp * error + this->Ki * this->integral + this->Kd * derivative;

    // Integral limit
    if (this->integral > this->outmax) {
        this->integral = this->outmax;
    } else if (this->integral < this->outmin) {
        this->integral = this->outmin;
    }

    // Output limit
    if (output > this->outmax) {
        output = this->outmax;
    } else if (output < this->outmin) {
        output = this->outmin;
    }

    this->last_out = output;

    return output;
}