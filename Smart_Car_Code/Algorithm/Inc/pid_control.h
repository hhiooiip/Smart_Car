#ifndef __PID_CONTROL_H__
#define __PID_CONTROL_H__

#include "main.h"

// PID 结构体定义
typedef struct {
    float target;    // 目标速度 (脉冲数/10ms)
    float current;   // 当前速度 (脉冲数/10ms)
    float Kp, Ki, Kd;
    float err;       // 本次误差
    float last_err;  // 上次误差
    float prev_err;  // 上上次误差
    float out;       // PWM 输出值 (-1000 ~ 1000)
} PID_TypeDef;

// 外部声明4个电机的PID结构体
extern PID_TypeDef pid_A, pid_B, pid_C, pid_D;

void PID_Init(void);
void PID_SetTarget(float speed_left, float speed_right);
void PID_SetTarget4(float speed_A, float speed_B, float speed_C, float speed_D);
void Motor_Calc_PID_10ms(void);

#endif