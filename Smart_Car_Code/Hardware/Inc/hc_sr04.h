#ifndef __HC_SR04_H__
#define __HC_SR04_H__

#include "main.h"

// 引脚定义：根据实际接线修改
#define TRIG_PORT GPIOD
#define TRIG_PIN  GPIO_PIN_0

#define ECHO_PORT GPIOE
#define ECHO_PIN  GPIO_PIN_15

// 函数声明
void HC_SR04_Start_Measure(void);
uint8_t HC_SR04_Read_Distance(float *dist);
void HC_SR04_EXTI_Callback(uint16_t GPIO_Pin);

#endif