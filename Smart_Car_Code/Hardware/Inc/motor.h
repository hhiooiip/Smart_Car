#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

/* ================= 硬件引脚宏定义 ================= */
// A: 右前电机 (Motor A)
#define DIR_A_IN1_PORT  GPIOE
#define DIR_A_IN1_PIN   GPIO_PIN_7
#define DIR_A_IN2_PORT  GPIOE
#define DIR_A_IN2_PIN   GPIO_PIN_8

// B: 右后电机 (Motor B)
#define DIR_B_IN1_PORT  GPIOE
#define DIR_B_IN1_PIN   GPIO_PIN_10
#define DIR_B_IN2_PORT  GPIOE
#define DIR_B_IN2_PIN   GPIO_PIN_12

// C: 左后电机 (Motor C)
#define DIR_C_IN1_PORT  GPIOB
#define DIR_C_IN1_PIN   GPIO_PIN_14
#define DIR_C_IN2_PORT  GPIOB
#define DIR_C_IN2_PIN   GPIO_PIN_15

// D: 左前电机 (Motor D)
#define DIR_D_IN1_PORT  GPIOB
#define DIR_D_IN1_PIN   GPIO_PIN_12
#define DIR_D_IN2_PORT  GPIOB
#define DIR_D_IN2_PIN   GPIO_PIN_13

/* ================= 函数声明 ================= */
void Motor_Init(void);
void Set_Motor_PWM(int pwm_A, int pwm_B, int pwm_C, int pwm_D);

#endif /* __MOTOR_H__ */