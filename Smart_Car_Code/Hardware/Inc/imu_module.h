#ifndef __IMU_MODULE_H
#define __IMU_MODULE_H

#include "stdint.h"

// 声明全局变量，让系统里的其他文件都能读取到备用姿态数据
extern float imu_yaw;      // 备用航向角 (°)
extern float imu_gyro_z;   // 备用Z轴角速度 (°/s)

// 声明解析函数
void IMU_Data_Parse(uint8_t *data, uint16_t len);

#endif