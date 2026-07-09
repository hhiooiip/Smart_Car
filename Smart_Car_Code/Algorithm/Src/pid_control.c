#include "pid_control.h"
#include "tim.h"
#include "motor.h" // 复用你之前的 Set_Motor_Speed 等底层PWM驱动函数

PID_TypeDef pid_A, pid_B, pid_C, pid_D;
volatile int32_t Global_Pulse_A = 0;

// 初始化 PID 参数 (这里的 Kp, Ki, Kd 需要你根据实际小车微调)
void PID_Init(void) {
    // 假设初调参数，增量式一般 Kp 比较大，Ki 适中，Kd 较小甚至为0
    float kp = 15.0f, ki = 2.0f, kd = 0.5f;
    
    pid_A.Kp = kp; pid_A.Ki = ki; pid_A.Kd = kd;
    pid_B.Kp = kp; pid_B.Ki = ki; pid_B.Kd = kd;
    pid_C.Kp = kp; pid_C.Ki = ki; pid_C.Kd = kd;
    pid_D.Kp = kp; pid_D.Ki = ki; pid_D.Kd = kd;
}

// 增量式 PID 计算核心
void PID_Calc(PID_TypeDef *pid) {
    pid->err = pid->target - pid->current;
    
    // 增量公式：Δout = Kp*(e - e_last) + Ki*e + Kd*(e - 2*e_last + e_prev)
    float p_inc = pid->Kp * (pid->err - pid->last_err);
    float i_inc = pid->Ki * pid->err;
    float d_inc = pid->Kd * (pid->err - 2.0f * pid->last_err + pid->prev_err);
    
    pid->out += (p_inc + i_inc + d_inc);
    
    // 限制 PWM 最大幅度
    if(pid->out > 1000.0f) pid->out = 1000.0f;
    if(pid->out < -1000.0f) pid->out = -1000.0f;
    
    // 更新历史误差
    pid->prev_err = pid->last_err;
    pid->last_err = pid->err;
}

// 4轮独立目标速度设定
void PID_SetTarget4(float speed_A, float speed_B, float speed_C, float speed_D) {
    pid_A.target = speed_A; // 右前
    pid_B.target = speed_B; // 右后
    pid_C.target = speed_C; // 左后
    pid_D.target = speed_D; // 左前
}

// 兼容旧接口：左右两侧统一速度
void PID_SetTarget(float speed_left, float speed_right) {
    PID_SetTarget4(speed_right, speed_right, speed_left, speed_left);
}

// 10ms 定时器中断中调用的主函数
void Motor_Calc_PID_10ms(void) {
    // 1. 读取编码器并清零 (强转为 short 自动处理负数)
    int32_t enc_A = (short)__HAL_TIM_GET_COUNTER(&htim2); __HAL_TIM_SET_COUNTER(&htim2, 0);

   short enc_B = (short)__HAL_TIM_GET_COUNTER(&htim3); __HAL_TIM_SET_COUNTER(&htim3, 0);

    short enc_C = (short)__HAL_TIM_GET_COUNTER(&htim8); __HAL_TIM_SET_COUNTER(&htim8, 0);
    short enc_D = (short)__HAL_TIM_GET_COUNTER(&htim1); __HAL_TIM_SET_COUNTER(&htim1, 0);
	// 【新增这一句！】把被清零的脉冲累加起来！
    Global_Pulse_A += enc_A;
    
    // 注意：如果发现某轮子转动方向跟测速正负反了，直接在这里给 enc 取反，例如 enc_A = -enc_A;
    pid_A.current = enc_A;
    pid_B.current = enc_B;
    pid_C.current = enc_C;
    pid_D.current = enc_D;
    
    // 2. 执行 PID 计算
    PID_Calc(&pid_A);
    PID_Calc(&pid_B);
    PID_Calc(&pid_C);
    PID_Calc(&pid_D);
    
    // 3. 将计算出的 PWM 送给电机底层函数 (需要拆分你原来的 Set_Motor_Speed 为 4 个独立控制，或者如下修改)
    // 这里为了简便，假设你底层提供了一个 Set_Motor_PWM(int A, int B, int C, int D) 函数
    Set_Motor_PWM((int)pid_A.out, (int)pid_B.out, (int)pid_C.out, (int)pid_D.out);
}