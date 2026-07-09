#ifndef __MPU6050_H__
#define __MPU6050_H__

// ===================================================
// 【赛场紧急切换开关】
// 1 = 使用全新的串口 IMU (带卡尔曼滤波)
// 0 = 使用老版本的 I2C MPU6050 (代码积分计算)
// ===================================================
#define USE_SERIAL_IMU 1

#include "main.h"

// MPU6050 I2C 地址 (AD0接地时为 0x68，HAL库需要左移1位变为 0xD0)
#define MPU_ADDR 0xD0 

void MPU6050_Init(void);
void MPU6050_Calibrate(void);
void MPU6050_Update_Yaw(float dt);
float Get_MPU6050_Yaw(void);

#endif