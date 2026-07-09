#include "Gimbal_API.h"

void Gimbal_Init(void)
{
    Gimbal_CAN_Init();
    Gimbal_PID_Init();
    Gimbal_PID_SetTarget(400, 240);
}

void Gimbal_SetTarget(int center_x, int center_y)
{
    Gimbal_PID_SetTarget(center_x, center_y);
}

void Gimbal_Track(int current_x, int current_y)
{
    int x_out = Gimbal_Down_PID(current_x, Gimbal_PID.Down_Gimbal_Angle_Target);
    int y_out = Gimbal_Up_PID(current_y, Gimbal_PID.Up_Gimbal_Angle_Target);

    Gimbal_PID.Down_Gimbal_Angle_Out = x_out;
    Gimbal_PID.Up_Gimbal_Angle_Out = y_out;

    if (x_out > 0)
    {
        Emm_V5_Pos_Control(Gimbal_Down, CCW, 5000, 0, (uint32_t)x_out, 0, 0);
        Gimbal_CAN_WaitBlocking();
    }
    else if (x_out < 0)
    {
        Emm_V5_Pos_Control(Gimbal_Down, CW, 5000, 0, (uint32_t)(-x_out), 0, 0);
        Gimbal_CAN_WaitBlocking();
    }

    if (y_out > 0)
    {
        Emm_V5_Pos_Control(Gimbal_Up, CCW, 5000, 0, (uint32_t)y_out, 0, 0);
        Gimbal_CAN_WaitBlocking();
    }
    else if (y_out < 0)
    {
        Emm_V5_Pos_Control(Gimbal_Up, CW, 5000, 0, (uint32_t)(-y_out), 0, 0);
        Gimbal_CAN_WaitBlocking();
    }
}

void Gimbal_Stop(void)
{
    Emm_V5_Stop_Now(Gimbal_Down, 0);
    Emm_V5_Stop_Now(Gimbal_Up, 0);
}

void Gimbal_MovePos(uint8_t axis, uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses)
{
    Emm_V5_Pos_Control(axis, dir, speed, acc, pulses, 0, 0);
}

void Gimbal_MoveVel(uint8_t axis, uint8_t dir, uint16_t speed, uint8_t acc)
{
    Emm_V5_Vel_Control(axis, dir, speed, acc, 0);
}
