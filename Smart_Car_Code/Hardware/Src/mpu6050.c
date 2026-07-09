#include "mpu6050.h"
#include "i2c.h" // 包含 CubeMX 生成的 hi2c1
#include "imu_module.h"

static float gyro_z_bias = 0.0f;  // 静态零点漂移值
static float current_yaw = 0.0f;  // 全局偏航角

/**
 * @brief 简单的寄存器写函数
 */
static void MPU_Write_Reg(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, 1, &data, 1, 100);
}

/**
 * @brief 初始化 MPU6050
 */
void MPU6050_Init(void) {
    MPU_Write_Reg(0x6B, 0x00); // 唤醒 MPU6050 (解除休眠)
    HAL_Delay(10);
    MPU_Write_Reg(0x1B, 0x18); // 陀螺仪量程: ±2000 dps (高转速不溢出)
    MPU_Write_Reg(0x1C, 0x00); // 加速度计量程: ±2g
    MPU_Write_Reg(0x1A, 0x04); // 配置低通滤波，过滤高频震动噪声
}

/** * @brief 开机静态校准 (消除 Z 轴零偏)
 * @note  调用此函数时，小车必须绝对静止在地面上！！！
 */
void MPU6050_Calibrate(void) {
    int32_t sum_z = 0;
    uint8_t buf[2];
    int16_t raw_z = 0;

    // 读取 500 次静止数据求平均
    for (int i = 0; i < 500; i++) {
        HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x47, 1, buf, 2, 100); // 0x47是GYRO_ZOUT_H
        raw_z = (int16_t)((buf[0] << 8) | buf[1]);
        sum_z += raw_z;
        HAL_Delay(2); // 每次延时 2ms
    }
    
    // 计算原始数据的平均零偏
    gyro_z_bias = (float)sum_z / 500.0f;
}

/**
 * @brief 更新 Yaw 角 (必须在恒定周期的定时器中断中调用！)
 * @param dt: 调用周期 (例如 10ms = 0.01f)
 */
void MPU6050_Update_Yaw(float dt) {
#if !USE_SERIAL_IMU  // 只有在使用老模块时，才执行耗时的 I2C 读取和积分
    uint8_t buf[2];
    int16_t raw_z = 0;
    float gyro_rate;

    // 1. 读取 Z 轴陀螺仪原始数据
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x47, 1, buf, 2, 10) == HAL_OK) {
        raw_z = (int16_t)((buf[0] << 8) | buf[1]);
        
        // 2. 减去开机校准的零偏值
        float z_corrected = (float)raw_z - gyro_z_bias;

        // 3. 转换为物理角速度 (±2000dps 对应的除数是 16.4)
        gyro_rate = z_corrected / 16.4f;

        // 4. 设立一个“死区”（过滤极微小的底噪震动）
        if (gyro_rate > -0.5f && gyro_rate < 0.5f) {
            gyro_rate = 0.0f;
        }

        // 5. 积分计算绝对偏航角
        current_yaw += gyro_rate * dt;
    }
#endif
}

/**
 * @brief 对外提供获取角度的接口 (核心拦截点)
 */
float Get_MPU6050_Yaw(void) {
#if USE_SERIAL_IMU
    // 如果开关是 1，直接返回串口中断里自动解析好的新数据！
    // 外层 track.c 里的避障和循迹代码根本不知道底层已经换了传感器
    return imu_yaw; 
#else
    // 如果开关是 0，返回老代码算出来的角度
    return current_yaw;
#endif
}