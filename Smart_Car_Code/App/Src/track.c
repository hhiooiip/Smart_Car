#include "track.h"
#include "gw_gray.h"
#include "pid_control.h"
#include "motor.h" 
#include "tim.h"   
#include "mpu6050.h" 
#include <math.h> // 避障转角需要用到 fabs() 绝对值函数
#include <stdio.h> // <--- 新增这一行，专门用来消除 printf 的隐式声明警告

// =================【状态机与全局变量】=================
volatile AppMode_t Task_Mode = MODE_TRACKING; // 比赛模式：默认循迹题
volatile uint8_t Black_Line_Count = 0;        // 记录遇到全黑线的次数

// =========================================================
// 【第一部分：核心参数配置区 (全车神经中枢)】
// =========================================================

// 1. 灰度循迹参数
#define BASE_TARGET_SPEED 32.0f  // 基础巡线速度
#define KP_TRACK          10.0f   // 巡线 PID 的 P 参数 (控制转弯灵敏度)
#define FRONT_TURN_RATIO  1.2f   // 前轮转弯放大系数 (前轮拉力更强)
#define REAR_TURN_RATIO   0.8f   // 后轮转弯缩减系数 (后轮平稳跟随)

// 2. 断线盲走参数
#define KP_YAW            2.5f   // 陀螺仪锁向直行的 P 参数
#define YAW_DIR_MULTIPLIER 1.0f  // MPU6050方向开关：如果转向相反改为 -1.0f

// 3. 避障动作组参数
#define KP_TURN          4.0f    // 原地转向的灵敏度
#define BYPASS_SPEED     25.0f   // 绕行直行时的速度
#define PULSES_PER_CM    67.0f   // 【关键实测值】小车走1厘米，TIM2编码器增加的脉冲数

// 引入 motor.c 中的全局脉冲累加器 (解决避障直行脉冲被 PID 定时器清零的 Bug)
extern volatile int32_t Global_Pulse_A;

// 探头权重数组（正负号决定方向，外侧探头权重更大，转弯拉回力更强）
const float Weights[8] = {4.0f, 3.0f, 2.0f, 1.0f, -1.0f, -2.0f, -3.0f, -4.0f};

// =========================================================
// 【第二部分：动作组补充 (跨线与打靶)】
// =========================================================

// 提前声明绝对直行函数
void Encoder_Move(float cm);

/**
 * @brief 锁向跨越黑线 (防止在黑线上反复横跳导致误触发多重计数)
 */
void Skip_Black_Line(void) {
    // 1. 瞬间锁定当前车头朝向
    float lock_yaw = Get_MPU6050_Yaw() * YAW_DIR_MULTIPLIER;
    
    // 2. 清零脉冲累加器
    Global_Pulse_A = 0; 
    
    // 3. 计算跨越 8cm 需要的脉冲数 (如果十字路口很窄，改成 5.0f 也可以)
    int32_t target_pulses = (int32_t)(5.0f * PULSES_PER_CM);
    
    while (1) {
        int32_t current_pulses = Global_Pulse_A;
        if (current_pulses < 0) current_pulses = -current_pulses;
        
        // 【关键改动】：达到目标距离后，直接退出循环！
        // 绝不踩刹车！绝不写 HAL_Delay！把当前的速度和惯性完美甩给巡线函数！
        if (current_pulses >= target_pulses) {
            break; 
        }
        
        // 4. 跨线期间，使用巡线的默认速度 (保持动力一致)，并带上陀螺仪防跑偏
        float current_yaw = Get_MPU6050_Yaw() * YAW_DIR_MULTIPLIER;
        float yaw_error = current_yaw - lock_yaw; 
        
        // 注意：这里用的是 BASE_TARGET_SPEED，而不是避障的 BYPASS_SPEED，保证速度零波动
        float target_L = BASE_TARGET_SPEED + (yaw_error * KP_YAW);
        float target_R = BASE_TARGET_SPEED - (yaw_error * KP_YAW);
        PID_SetTarget(target_L, target_R);
        
        HAL_Delay(10);
    }
}

/**
 * @brief 停车执行打靶/识别任务 (仅在打靶模式下触发)
 */
void Stop_Car_Wait_Task(void) {
    // 1. 强力重刹，确保画面稳定无残影
    PID_SetTarget(0, 0);
    HAL_Delay(300); 
    
    // 2. ！！！未来这里加入你的 K230 识别和二维云台代码！！！
    printf(">>> 到达打靶区！正在执行视觉任务... <<<\r\n");
    HAL_Delay(2000); // 暂时用 2 秒延时模拟打靶耗时
    
    // 3. 任务完成后，跨过当前这条黑线，继续前往下一站
    Skip_Black_Line();
}


// =========================================================
// 【第三部分：基础循迹与黑线状态机】
// =========================================================

// 盲走状态记忆变量
static uint8_t is_blind_driving = 0; 
static float target_yaw = 0.0f;      
// 寻迹记忆变量：永远记住最后一次看到黑线时的位置（用于平滑过直角弯）
static float last_error = 0.0f;

/**
 * @brief 计算灰度传感器偏差值 (加权平均法)
 */
float Calculate_Line_Error(uint8_t sensor_data) {
    float sum_weights = 0.0f; 
    int active_sensors = 0;   
    for (int i = 0; i < 8; i++) {
        if ((sensor_data >> i) & 0x01) {
            sum_weights += Weights[i];
            active_sensors++;
        }
    }
    return sum_weights / (float)active_sensors;
}

/**
 * @brief 主循迹函数 (包含 7次黑线仲裁、过直角弯、断线盲走)
 */
void Track_Line(void)
{
    // 读取灰度传感器数据 (取反，使得黑线对应位为1)
    uint8_t raw_val = GW_Gray_Serial_Read();
    uint8_t sensor_val = (~raw_val) & 0xFF; 
    
    float current_yaw = Get_MPU6050_Yaw() * YAW_DIR_MULTIPLIER; 
    
    // 统计当前踩到黑线的探头数量
    uint8_t black_count = 0;
    for (int i = 0; i < 8; i++) {
        if ((sensor_val >> i) & 0x01) black_count++;
    }
    
    // ===============================================
    // 状态 A：全黑线判定 (状态机核心，处理 7 次黑线)
    // ===============================================
    if (black_count >= 6) {
        Black_Line_Count++; // 全局黑线计数 +1
        printf("--- 检测到第 %d 次全黑线 ---\r\n", Black_Line_Count);
        
        // 节点 7：终点停车线，无论是哪种题，都强制结束比赛
        if (Black_Line_Count >= 8) {
            printf(">>> 到达终点！比赛结束！ <<<\r\n");
            is_blind_driving = 0; 
            HAL_TIM_Base_Stop_IT(&htim6);          // 剥夺 10ms PID 控制权
            Set_Motor_PWM(-800, -800, -800, -800); // 绝杀重刹
            HAL_Delay(50); 
            Set_Motor_PWM(0, 0, 0, 0); 
            while(1); // 死循环停机
        }
        // 节点 2 和 3：打靶识别区停车线
        else if (Black_Line_Count == 2 || Black_Line_Count == 3) {
            if (Task_Mode == MODE_RECOGNITION) {
                Stop_Car_Wait_Task(); // 打靶题：停车执行云台任务
            } else {
                Skip_Black_Line();    // 循迹题：直接跨过，不停留
            }
        }
        // 其他节点 (第1次发车线, 第4、5、6次十字路口)：全部直接跳过
        else {
            Skip_Black_Line();
        }
        
        return; // 跨越黑线后直接退出本次循环，重新读取最新的传感器状态！
    }

    // ===============================================
    // 状态 B：全白丢失区 (核心状态机分流)
    // ===============================================
    if (sensor_val == 0x00) {
        // 分支 1：过直角弯时轻微冲出赛道 -> 纯循迹接管！
        // 沿用最后一次看到线时的巨大误差，让 PID 控制小车画一个平滑的圆弧接回赛道
        if (fabs(last_error) > 1.5f) {
            float corr = last_error * KP_TRACK;
            float sp_RF = BASE_TARGET_SPEED + (corr * FRONT_TURN_RATIO);
            float sp_RR = BASE_TARGET_SPEED + (corr * REAR_TURN_RATIO);
            float sp_LR = BASE_TARGET_SPEED - (corr * REAR_TURN_RATIO);
            float sp_LF = BASE_TARGET_SPEED - (corr * FRONT_TURN_RATIO);
            PID_SetTarget4(sp_RF, sp_RR, sp_LR, sp_LF);
            return;
        }
        // 分支 2：直行时遇到虚线断口 -> MPU6050 锁向盲走接管！
        else {
            if (is_blind_driving == 0) {
                target_yaw = current_yaw; // 瞬间锁定车头朝向作为目标航向
                is_blind_driving = 1;
            }
            float yaw_error = current_yaw - target_yaw;
            float target_L = BASE_TARGET_SPEED + (yaw_error * KP_YAW);
            float target_R = BASE_TARGET_SPEED - (yaw_error * KP_YAW);
            PID_SetTarget(target_L, target_R);
            return; 
        }
    }

    // ===============================================
    // 状态 C：正常循迹区 (看到黑线)
    // ===============================================
    is_blind_driving = 0; // 解除盲走锁定

    float error = Calculate_Line_Error(sensor_val);
    last_error = error; // 【关键】永远记住最后一次看到黑线时的位置！

    float correction = error * KP_TRACK;
    float speed_RF = BASE_TARGET_SPEED + (correction * FRONT_TURN_RATIO); // 右前(A)
    float speed_RR = BASE_TARGET_SPEED + (correction * REAR_TURN_RATIO);  // 右后(B)
    float speed_LR = BASE_TARGET_SPEED - (correction * REAR_TURN_RATIO);  // 左后(C)
    float speed_LF = BASE_TARGET_SPEED - (correction * FRONT_TURN_RATIO); // 左前(D)

    PID_SetTarget4(speed_RF, speed_RR, speed_LR, speed_LF);
}


// =========================================================
// 【第四部分：超声波避障动作组 (MPU6050 + 编码器联动)】
// =========================================================

/**
 * @brief 基于 MPU6050 的精准原地转角
 * @param target_angle_change 要转的角度 (左转为正，右转为负)
 */
void MPU_Turn(float target_angle_change) {
    float start_yaw = Get_MPU6050_Yaw() * YAW_DIR_MULTIPLIER;
    float target_angle = start_yaw + target_angle_change;
    
    while (1) {
        float current_yaw = Get_MPU6050_Yaw() * YAW_DIR_MULTIPLIER;
        float error = target_angle - current_yaw;
        
        // 误差小于 2 度，刹车退出
        if (fabs(error) < 2.0f) {
            PID_SetTarget(0, 0);
            HAL_Delay(100); 
            break; 
        }
        
        // 转向速度 PID 限制
        float turn_speed = error * KP_TURN;
        if (turn_speed > 35.0f) turn_speed = 35.0f;
        if (turn_speed < -35.0f) turn_speed = -35.0f;
        
        PID_SetTarget(-turn_speed, turn_speed); 
        HAL_Delay(10); 
    }
}

/**
 * @brief 基于编码器和陀螺仪的绝对精准直行
 * @param cm 要行驶的距离 (厘米)
 */
void Encoder_Move(float cm) {
    // 1. 记录起点的车头朝向 (全程死死锁住这个角度防跑偏)
    float lock_yaw = Get_MPU6050_Yaw() * YAW_DIR_MULTIPLIER;
    
    // 2. 每次直行前，把累加器清零
    Global_Pulse_A = 0; 
    int32_t target_pulses = (int32_t)(cm * PULSES_PER_CM);
    
    while (1) {
        // 读取累加器的值（再也不怕被 PID 中断清零了！）
        int32_t current_pulses = Global_Pulse_A;
        if (current_pulses < 0) current_pulses = -current_pulses;
        
        // 达到目标脉冲数，刹车退出
        if (current_pulses >= target_pulses) {
            PID_SetTarget(0, 0);
            HAL_Delay(100);
            break;
        }
        
        // 陀螺仪角度修正计算
        float current_yaw = Get_MPU6050_Yaw() * YAW_DIR_MULTIPLIER;
        float yaw_error = current_yaw - lock_yaw; 
        
        float target_L = BYPASS_SPEED + (yaw_error * KP_YAW);
        float target_R = BYPASS_SPEED - (yaw_error * KP_YAW);
        PID_SetTarget(target_L, target_R);
        
        HAL_Delay(10);
    }
}

/**
 * @brief 霸道执行避障全流程 (从右侧绕行 16cm 方块)
 */
void Execute_Obstacle_Bypass(void) {
    // 0. 发现障碍物，先重刹停稳
    PID_SetTarget(0, 0);
    HAL_Delay(300);
    
    // 1. 原地右转 90 度
    MPU_Turn(-90.0f);
    
    // 2. 侧向直行 30 cm (躲开方块宽度)
    Encoder_Move(30.0f);
    
    // 3. 原地左转 90 度 (车头回正，朝向赛道前方)
    MPU_Turn(90.0f);
    
    // 4. 越障直行 65 cm (越过前方区域)
    Encoder_Move(65.0f);
    
    // 5. 原地左转 90 度 (车头对准赛道内部)
    MPU_Turn(90.0f);
    
    // 6. 开启锁向直行，直到灰度传感器重新看到黑线
    float lock_yaw = Get_MPU6050_Yaw() * YAW_DIR_MULTIPLIER;
    while (1) {
        uint8_t raw_val = GW_Gray_Serial_Read();
        uint8_t sensor_val = (~raw_val) & 0xFF;
        
        // 如果看到任何黑线，立刻刹车退出找线循环
        if (sensor_val != 0x00) {
            PID_SetTarget(0, 0);
            HAL_Delay(100);
            break;
        }
        
        float current_yaw = Get_MPU6050_Yaw() * YAW_DIR_MULTIPLIER;
        float yaw_error = current_yaw - lock_yaw; 
        float target_L = BYPASS_SPEED + (yaw_error * KP_YAW);
        float target_R = BYPASS_SPEED - (yaw_error * KP_YAW);
        PID_SetTarget(target_L, target_R);
        
        HAL_Delay(10);
    }
    
    // ==========================================================
    // 【中心点补偿直行】
    // 假设灰度探头距离车轮中心轴大概 10 厘米，往前补偿直行 10 厘米。
    // 这样转弯的时候，车身旋转中心刚好能压在黑线上，完美入轨！
    // ==========================================================
    Encoder_Move(10.0f); 
    
    // 7. 最后一次原地右转 90 度，车身对齐赛道！
    MPU_Turn(-90.0f);
    
    // 函数结束，控制权自动交还给主循环的 Track_Line()
}