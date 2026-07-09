#ifndef __TRACK_H__
#define __TRACK_H__

#include "main.h"

// 基础速度和转向速度配置 (根据你的电机和小车重量调整)
#define BASE_SPEED    400   // 直行基础速度
#define TURN_SPEED_1  150   // 轻微偏转减速幅度
#define TURN_SPEED_2  300   // 严重偏转减速/反转幅度

// 比赛模式枚举定义
typedef enum {
    MODE_TRACKING = 0,    // 纯循迹模式 (开启避障，全程不停)
    MODE_RECOGNITION = 1  // 打靶识别模式 (关闭避障，特定黑线停车)
} AppMode_t;

// 暴露全局变量
extern volatile AppMode_t Task_Mode;
extern volatile uint8_t Black_Line_Count;

void Track_Line(void);
void Execute_Obstacle_Bypass(void);

#endif