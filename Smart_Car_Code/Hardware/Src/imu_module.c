#include "imu_module.h"

// 定义实际存储数据的全局变量
float imu_yaw = 0.0f;
float imu_gyro_z = 0.0f;

/**
 * @brief  解析新模组发来的 5 字节协议数据
 * @param  data: 串口接收缓冲区的指针
 * @param  len:  本次接收到的总字节数
 */
void IMU_Data_Parse(uint8_t *data, uint16_t len)
{
    // 遍历数据，留出4个字节的余量（完整一帧是5个字节：头 + 类型 + 低位 + 高位 + 校验）
    for (uint16_t i = 0; i < len - 4; i++) 
    {
        // 1. 寻找固定的帧头 0x5A
        if (data[i] == 0x5A)
        {
            uint8_t type  = data[i+1];
            uint8_t dataL = data[i+2];
            uint8_t dataH = data[i+3];
            uint8_t sum   = data[i+4];

            // 2. 计算校验和 (只取低8位)
            uint8_t calc_sum = (0x5A + type + dataL + dataH) & 0xFF;
            
            // 3. 校验比对，如果通过说明这是一帧正确的数据
            if (calc_sum == sum) 
            {
                // 4. 数据拼接与符号处理 (强制转换为有符号 short)
                short raw_data = (short)(((short)dataH << 8) | dataL);

                // 5. 换算成实际物理量
                if (type == 0xAA) // 0xAA 代表角速度
                {
                    imu_gyro_z = (float)raw_data / 32768.0f * 2000.0f;
                }
                else if (type == 0xBB) // 0xBB 代表航向角度
                {
                    imu_yaw = (float)raw_data / 32768.0f * 180.0f;
                }
                
                // 成功解析一帧后，游标往后跳 4 步，直接去搜寻下一帧，节省 CPU 算力
                i += 4; 
            }
        }
    }
}