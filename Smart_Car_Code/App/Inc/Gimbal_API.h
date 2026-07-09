#ifndef __GIMBAL_API_H
#define __GIMBAL_API_H

#include "Gimbal_Common.h"
#include "Gimbal_CAN.h"
#include "Gimbal_Emm_V5.h"
#include "Gimbal_PID.h"

/**
 * 云台模块顶层 API
 * 在循迹小车中 #include "Gimbal_API.h" 即可使用全部功能
 */

/* 初始化：CAN滤波器 + PID参数，在 main() 中 MX_CAN1_Init() 之后调用 */
void Gimbal_Init(void);

/* 跟踪控制：传入当前检测到的像素坐标，内部计算PID并驱动电机 */
void Gimbal_Track(int current_x, int current_y);

/* 设置目标像素坐标（画面中心），默认 (400, 240) 对应 800x480 */
void Gimbal_SetTarget(int center_x, int center_y);

/* 急停 */
void Gimbal_Stop(void);

/* 直接位置控制 (绕过PID) */
void Gimbal_MovePos(uint8_t axis, uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses);

/* 直接速度控制 */
void Gimbal_MoveVel(uint8_t axis, uint8_t dir, uint16_t speed, uint8_t acc);

#endif
