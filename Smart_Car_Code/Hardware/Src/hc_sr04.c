#include "hc_sr04.h"
#include "tim.h" 

volatile uint8_t  echo_state = 0; 
volatile uint32_t echo_start_time = 0; 
volatile float    measured_distance = 999.0f; 
volatile uint8_t  distance_updated = 0; 
static uint16_t   last_trigger_time = 0;

/**
 * @brief 绝对安全的软件延时 (强行循环，无视任何定时器异常，绝不卡死！)
 */
static void Delay_20us(void) {
    // 粗略延时，确保大于 10us 即可激活模块
    for (volatile int i = 0; i < 1500; i++) {
        __NOP();
    }
}

/**
 * @brief 触发超声波测距
 */
void HC_SR04_Start_Measure(void) {
    uint16_t current_time = __HAL_TIM_GET_COUNTER(&htim7);
    
    // 1. 防死机超时机制 (50ms)
    if (echo_state != 0) {
        if ((uint16_t)(current_time - last_trigger_time) > 50000) {
            echo_state = 0; // 超时复位
        } else {
            return; 
        }
    }

    // 2. 纯软件脉冲触发
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    Delay_20us(); 
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    last_trigger_time = __HAL_TIM_GET_COUNTER(&htim7);
    echo_state = 1; 
}

/**
 * @brief 读取距离
 */
uint8_t HC_SR04_Read_Distance(float *dist) {
    if (distance_updated == 1) {
        *dist = measured_distance;
        distance_updated = 0; 
        return 1; 
    }
    return 0; 
}

/**
 * @brief 外部中断计算逻辑
 */
void HC_SR04_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ECHO_PIN) {
        
        // 上升沿
        if (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET) {
            if (echo_state == 1) {
                echo_start_time = __HAL_TIM_GET_COUNTER(&htim7);
                echo_state = 2; 
            }
        } 
        // 下降沿
        else {
            if (echo_state == 2) {
                uint32_t echo_end_time = __HAL_TIM_GET_COUNTER(&htim7);
                uint32_t time_diff;

                // 计算高电平持续的微秒数
                if (echo_end_time >= echo_start_time) {
                    time_diff = echo_end_time - echo_start_time;
                } else {
                    time_diff = (65535 - echo_start_time) + echo_end_time + 1;
                }

                // ==================================================
                // 【终极结算】过滤掉贴脸杂波，并乘以声速 0.017！
                // ==================================================
                if (time_diff > 100 && time_diff < 30000) {
                    measured_distance = (float)time_diff * 0.017f; 
                    distance_updated = 1; 
                }
                
                echo_state = 0; 
            }
        }
    }
}