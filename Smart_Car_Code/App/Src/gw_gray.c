#include "gw_gray.h"

/**
  * @brief  粗略的微秒级延时函数 (基于 CPU 空循环)
  * @param  us: 需要延时的微秒数
  * @note   适用于 STM32F407，基于系统当前主频自动计算
  */
void delay_us(uint32_t us)
{
    // 根据当前主频计算大概的循环次数 (F407 通常配置为 168MHz)
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
    while (delay--) {
        __NOP(); 
    }
}

/**
  * @brief  读取感为八路灰度传感器 (数字量串行时序通信)
  * @retval uint8_t 包含 8 个通道数据的字节
  */
uint8_t GW_Gray_Serial_Read(void)
{
    uint8_t ret = 0;
    uint8_t i;

    // 初始状态：拉低时钟线
    HAL_GPIO_WritePin(GW_CLK_PORT, GW_CLK_PIN, GPIO_PIN_RESET);
    
    for (i = 0; i < 8; ++i) {
        /* 1. 时钟线上升沿，通知模块准备数据 */
        HAL_GPIO_WritePin(GW_CLK_PORT, GW_CLK_PIN, GPIO_PIN_SET);
        
        /* 2. 延时 5us 等待数据稳定 */
        delay_us(5); 
        
        /* 3. 时钟线下降沿 */
        HAL_GPIO_WritePin(GW_CLK_PORT, GW_CLK_PIN, GPIO_PIN_RESET);
        
        /* 4. 读取 DAT 数据线电平，并按位组装 */
        // 注意：如果你发现车子左右转方向是反的，
        // 可以将 (1 << i) 改为 (1 << (7 - i)) 来翻转读取顺序
        if (HAL_GPIO_ReadPin(GW_DAT_PORT, GW_DAT_PIN) == GPIO_PIN_SET) {
            ret |= (1 << i);
        }
    }
    
    return ret;
}