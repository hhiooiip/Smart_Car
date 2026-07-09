#ifndef __GIMBAL_COMMON_H
#define __GIMBAL_COMMON_H

#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "math.h"
#include "stdbool.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

#define ABS(x) (((x) >= 0) ? (x) : -(x))

#define Gimbal_Down  1
#define Gimbal_Up    2
#define CW           0
#define CCW          1

extern CAN_HandleTypeDef hcan1;

#endif
