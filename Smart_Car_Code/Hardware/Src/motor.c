#include "motor.h"
#include "tim.h" 

/**
  * @brief  电机及编码器底层初始化
  */
void Motor_Init(void)
{
    // 1. 开启 TIM4 的四个 PWM 输出通道
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // A: 右前 (PD12)
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // B: 右后 (PD13)
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // C: 左后 (PD14)
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // D: 左前 (PD15)
    
    // 2. 开启四个定时器的编码器模式 (对应 TIM5, TIM3, TIM8, TIM1)
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // A相编码器
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // B相编码器
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL); // C相编码器
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // D相编码器
}

/**
  * @brief  独立设置 4 个电机的 PWM 值 (带方向控制)
  * @param  pwm_A: 右前电机 PWM (-1000 到 1000)
  * @param  pwm_B: 右后电机 PWM (-1000 到 1000)
  * @param  pwm_C: 左后电机 PWM (-1000 到 1000)
  * @param  pwm_D: 左前电机 PWM (-1000 到 1000)
  * @note   正数代表前进，负数代表后退
  */
void Set_Motor_PWM(int pwm_A, int pwm_B, int pwm_C, int pwm_D)
{
    // ================= 1. 输入限幅保护 =================
    // 防止 PID 积分饱和或计算爆炸导致给定时器的值越界 (假设 ARR 为 1000-1)
    if(pwm_A > 1000) pwm_A = 1000; else if(pwm_A < -1000) pwm_A = -1000;
    if(pwm_B > 1000) pwm_B = 1000; else if(pwm_B < -1000) pwm_B = -1000;
    if(pwm_C > 1000) pwm_C = 1000; else if(pwm_C < -1000) pwm_C = -1000;
    if(pwm_D > 1000) pwm_D = 1000; else if(pwm_D < -1000) pwm_D = -1000;

    // ================= 2. Motor A (右前) =================
    if (pwm_A >= 0) {
        HAL_GPIO_WritePin(DIR_A_IN1_PORT, DIR_A_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR_A_IN2_PORT, DIR_A_IN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_A);
    } else {
        HAL_GPIO_WritePin(DIR_A_IN1_PORT, DIR_A_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR_A_IN2_PORT, DIR_A_IN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, -pwm_A); // 取绝对值
    }

    // ================= 3. Motor B (右后) =================
    if (pwm_B >= 0) {
        HAL_GPIO_WritePin(DIR_B_IN1_PORT, DIR_B_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR_B_IN2_PORT, DIR_B_IN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm_B);
    } else {
        HAL_GPIO_WritePin(DIR_B_IN1_PORT, DIR_B_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR_B_IN2_PORT, DIR_B_IN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, -pwm_B);
    }

    // ================= 4. Motor C (左后) =================
    if (pwm_C >= 0) {
        HAL_GPIO_WritePin(DIR_C_IN1_PORT, DIR_C_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR_C_IN2_PORT, DIR_C_IN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_C);
    } else {
        HAL_GPIO_WritePin(DIR_C_IN1_PORT, DIR_C_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR_C_IN2_PORT, DIR_C_IN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, -pwm_C);
    }

    // ================= 5. Motor D (左前) =================
    if (pwm_D >= 0) {
        HAL_GPIO_WritePin(DIR_D_IN1_PORT, DIR_D_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR_D_IN2_PORT, DIR_D_IN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_D);
    } else {
        HAL_GPIO_WritePin(DIR_D_IN1_PORT, DIR_D_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR_D_IN2_PORT, DIR_D_IN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, -pwm_D);
    }
}