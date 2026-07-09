#include "Gimbal_PID.h"

Gimbal_PID_t Gimbal_PID;

void Gimbal_PID_Init(void)
{
    Gimbal_PID.Down_Gimbal_Kp = 2.0f;
    Gimbal_PID.Down_Gimbal_Ki = 0.0f;
    Gimbal_PID.Down_Gimbal_Kd = 0.0f;
    Gimbal_PID.Down_Gimbal_sum = 0;
    Gimbal_PID.Down_Gimbal_Angle_Out = 0;

    Gimbal_PID.Up_Gimbal_Kp = 2.0f;
    Gimbal_PID.Up_Gimbal_Ki = 0.0f;
    Gimbal_PID.Up_Gimbal_Kd = 0.0f;
    Gimbal_PID.Up_Gimbal_sum = 0;
    Gimbal_PID.Up_Gimbal_Angle_Out = 0;
}

int Gimbal_Down_PID(int measure, int target)
{
    static float last_err = 0;
    float err = (float)(measure - target);

    Gimbal_PID.Down_Gimbal_sum += err;
    if (Gimbal_PID.Down_Gimbal_sum > 3000)  Gimbal_PID.Down_Gimbal_sum = 3000;
    if (Gimbal_PID.Down_Gimbal_sum < -3000) Gimbal_PID.Down_Gimbal_sum = -3000;

    float d_err = err - last_err;
    last_err = err;

    float out = Gimbal_PID.Down_Gimbal_Kp * err +
                Gimbal_PID.Down_Gimbal_Ki * Gimbal_PID.Down_Gimbal_sum +
                Gimbal_PID.Down_Gimbal_Kd * d_err;

    if (out > 2000)  out = 2000;
    if (out < -2000) out = -2000;
    return (int)out;
}

int Gimbal_Up_PID(int measure, int target)
{
    static float last_err = 0;
    float err = (float)(measure - target);

    Gimbal_PID.Up_Gimbal_sum += err;
    if (Gimbal_PID.Up_Gimbal_sum > 800)  Gimbal_PID.Up_Gimbal_sum = 800;
    if (Gimbal_PID.Up_Gimbal_sum < -800) Gimbal_PID.Up_Gimbal_sum = -800;

    float d_err = err - last_err;
    last_err = err;

    float out = Gimbal_PID.Up_Gimbal_Kp * err +
                Gimbal_PID.Up_Gimbal_Ki * Gimbal_PID.Up_Gimbal_sum +
                Gimbal_PID.Up_Gimbal_Kd * d_err;

    if (out > 400)  out = 400;
    if (out < -400) out = -400;
    return (int)out;
}

void Gimbal_PID_SetTarget(int down_target, int up_target)
{
    Gimbal_PID.Down_Gimbal_Angle_Target = down_target;
    Gimbal_PID.Up_Gimbal_Angle_Target = up_target;
}
