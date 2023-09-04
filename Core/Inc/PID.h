//
// Created by ashkore on 23-9-4.
//

#ifndef BALANCE_CAR_PID_H
#define BALANCE_CAR_PID_H


class PID {
private:
    float last_error;
    float integral;
    float outmax;
    float outmin;

public:
    float Kp;
    float Ki;
    float Kd;
    float last_out;

    PID(float Kp, float Ki, float Kd, float outmax, float outmin);

    float calc(float input_value, float setpoint);

};


#endif //BALANCE_CAR_PID_H
