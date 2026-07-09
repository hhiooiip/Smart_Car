#ifndef __GIMBAL_CAN_H
#define __GIMBAL_CAN_H

#include "Gimbal_Common.h"

/* CAN 接收数据结构 */
typedef struct {
    __IO bool rxFrameFlag;
    __IO uint8_t packNum;
    __IO uint8_t pack[256];
} Gimbal_CAN_t;

extern __IO Gimbal_CAN_t gimbal_can;
extern float gimbal_angle_Real;

void Gimbal_CAN_Init(void);
void Gimbal_CAN_RX_IRQHandler(void);
bool Gimbal_CAN_WaitResponse(void);
void Gimbal_CAN_WaitBlocking(void);

#endif
