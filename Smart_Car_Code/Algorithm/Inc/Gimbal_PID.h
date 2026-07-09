#ifndef __GIMBAL_PID_H
#define __GIMBAL_PID_H

#include "Gimbal_Common.h"

typedef struct
{
    float Down_Gimbal_Kp;
    float Down_Gimbal_Ki;
    float Down_Gimbal_Kd;
    float Down_Gimbal_sum;

    float Up_Gimbal_Kp;
    float Up_Gimbal_Ki;
    float Up_Gimbal_Kd;
    float Up_Gimbal_sum;

    int Down_Gimbal_Angle_Out;
    int Up_Gimbal_Angle_Out;

    int Down_Gimbal_Angle_Target;
    int Up_Gimbal_Angle_Target;
} Gimbal_PID_t;

extern Gimbal_PID_t Gimbal_PID;

int Gimbal_Down_PID(int measure, int target);
int Gimbal_Up_PID(int measure, int target);
void Gimbal_PID_SetTarget(int down_target, int up_target);
void Gimbal_PID_Init(void);

#endif
