//
// Created by ashkore on 2023/7/21.
//

#include "pid.h"

PID PID_Init(float Kp, float Ki, float Kd, float outmax, float outmin){
  PID ret;
  ret.Kp = Kp;
  ret.Ki = Ki;
  ret.Kd = Kd;
  ret.last_error = 0;
  ret.last_out = 0;
  ret.integral = 0;
  ret.outmax = outmax;
  ret.outmin = outmin;
  return ret;
}

float PID_Calc(PID *pid, float input_value, float setpoint){
  float error = setpoint - input_value;
  float derivative = error - pid->last_error;
  pid->integral += error;
  pid->last_error = error;
  float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

  // Integral limit
  if(pid->integral > pid->outmax){
      pid->integral = pid->outmax;
  } else if(pid->integral < pid->outmin){
      pid->integral = pid->outmin;
  }

  // Output limit
  if(output > pid->outmax){
      output = pid->outmax;
  } else if(output < pid->outmin){
      output = pid->outmin;
  }

  return output;
}