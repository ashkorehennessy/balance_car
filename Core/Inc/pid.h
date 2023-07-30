//
// Created by ashkore on 2023/7/21.
//

#ifndef BALANCE_CAR_PID_H
#define BALANCE_CAR_PID_H

#endif //BALANCE_CAR_PID_H

typedef struct PID{
  float Kp;
  float Ki;
  float Kd;
  float last_error;
  float last_out;
  float integral;
  float outmax;
  float outmin;
}PID;

PID PID_Init(float Kp, float Ki, float Kd, float outmax, float outmin);

float PID_Calc(PID *pid, float input_value, float setpoint);