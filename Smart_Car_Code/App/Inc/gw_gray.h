#ifndef __GW_GRAY_H__
#define __GW_GRAY_H__

#include "main.h"  // 包含HAL库基础头文件
#include <stdint.h>

/* ================== 硬件引脚配置区 ================== */
/* 已经更新为最新的 PB10 和 PB11 */

#define GW_CLK_PORT  GPIOB
#define GW_CLK_PIN   GPIO_PIN_10

#define GW_DAT_PORT  GPIOB
#define GW_DAT_PIN   GPIO_PIN_11

/* ================== 外部可用函数声明 ================== */

// 粗略的微秒级延时函数
void delay_us(uint32_t us);

// 读取 8 路灰度传感器数据
// 返回值：8位无符号整数，每一位对应一路传感器的状态
uint8_t GW_Gray_Serial_Read(void);

#endif /* __GW_GRAY_H__ */