/*
 * PID_Base.h
 *
 *  Created on: 2023年9月13日
 *      Author: ashkore
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include <stdbool.h>

typedef struct{
    float Kp;
    float Kp2;
    float Ki;
    float Kd;
    float GzKd;
    float last_error;
    float last_out;
    float integral;
    float outmax;
    float outmin;
    bool use_lowpass_filter;
    float lowpass_filter_factor;
} PID_Base;

typedef struct{
    float Kp;
    float Ki;
    float Kd;
    float error;
    float last_error;
    float last_last_error;
    float last_out;
    float out;
    float outmax;
    float outmin;
    bool use_lowpass_filter;  // 驱动板或电机发热太大可以开，牺牲一点响应速度
    float lowpass_filter_factor;
} PID_Incremental;

PID_Base PID_Base_Init(float Kp, float Kp2, float Ki, float Kd, float GzKd, float outmax, float outmin, bool use_lowpass_filter, float lowpass_filter_factor);

float PID_Base_Calc(PID_Base *pid, float input_value, float gyroz_value, float setpoint);

PID_Incremental PID_Incremental_Init(float Kp, float Ki, float Kd, float outmax, float outmin, bool use_lowpass_filter, float lowpass_filter_factor);

float PID_Incremental_Calc(PID_Incremental *pid, float input_value, float setpoint);

#endif /* CODE_PID_H_ */
