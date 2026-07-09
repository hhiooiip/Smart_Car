/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "gw_gray.h"
#include "motor.h"
#include "track.h"
#include "pid_control.h"
#include "mpu6050.h"
#include "imu_module.h"
#include "hc_sr04.h"
#include "Gimbal_API.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern volatile uint8_t echo_state;
uint8_t imu_rx_buffer[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 开启 printf 串口打印
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim7); // <=== 启动TIM7(注意：不带_IT，它只做计数，不产生中断)
Motor_Init(); // 里面包含开启 4路PWM 和 4路Encoder定时器
PID_Init();
Gimbal_Init();

#if USE_SERIAL_IMU
    // 启动新模块的串口接收中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3, imu_rx_buffer, 50);
    printf("--- 系统启动：当前使用 新版串口IMU (USART3) ---\r\n");
#else
    // 只有开关为 0 时，才去初始化 I2C 和老陀螺仪
    printf("请保持小车静止，正在校准陀螺仪...\r\n");
    MPU6050_Init();
    MPU6050_Calibrate(); // 耗时约 1 秒，期间绝不能碰小车！
    printf("校准完成！\r\n");
#endif
  // 必须在校准完成后，再启动定时器中断！
  HAL_TIM_Base_Start_IT(&htim6);
  // ====================================================
  // 利用超声波当虚拟按键 (开机 2 秒窗口期)
  // ====================================================
  printf("【请选择模式】2秒内，用手挡住超声波(<10cm)为打靶题，不挡为循迹题...\r\n");
  
  // 循环检测 20 次，每次 100ms，总共 2 秒
  for(int i = 0; i < 20; i++) {
      HC_SR04_Start_Measure(); // 触发一次超声波
      HAL_Delay(20);           // 等待声波飞个来回
      
      float boot_dist = 0;
      if (HC_SR04_Read_Distance(&boot_dist) == 1) {
          // 如果开机时发现眼前 10cm 内有东西挡着 (也就是你的手)
          if (boot_dist > 0.1f && boot_dist < 10.0f) {
              Task_Mode = MODE_RECOGNITION; // 切换为打靶题！
              break; // 检测到就立刻退出循环
          }
      }
      HAL_Delay(80); // 凑够 100ms
  }

  // 打印最终确定的模式，并发出蜂鸣器提示音(如果有的话)
  if (Task_Mode == MODE_RECOGNITION) {
      printf(">>> 锁定模式：打靶识别题 (避障已禁用) <<<\r\n");
  } else {
      printf(">>> 锁定模式：纯循迹题 (避障已开启) <<<\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  // 避障防抖计数器
  static uint8_t obstacle_count = 0;
  while (1)
  {
	 
	 // 1. 只有在【纯循迹题】模式下，才进行超声波避障检测
      if (Task_Mode == MODE_TRACKING) 
      {
          HC_SR04_Start_Measure();
          float dist = 0;
          if (HC_SR04_Read_Distance(&dist) == 1) {
              // printf("超声波距离: %.2f cm\r\n", dist);
              
              if (dist > 0.0f && dist < 15.0f) {
                  obstacle_count++;
                  if (obstacle_count >= 3) { 
                      printf("触发避障！执行动作组！\r\n");
                      Execute_Obstacle_Bypass(); 
                      obstacle_count = 0; 
                  }
              } else {
                  obstacle_count = 0; 
              }
          }
      }

      // 2. 没被避障接管时，正常运行状态机巡线
      if (obstacle_count == 0) {
          Track_Line(); 
      }

      HAL_Delay(10);
	  // 1. 强制给所有电机一个正向的固定 PWM (开环)
      // 如果车轮是往后转的，说明你的电机线接反了，或者 IN1/IN2 逻辑反了
      //Set_Motor_PWM(300, 300, 300, 300);

      // 2. 读取四个编码器的原始累加值 (强转为 short 自动处理正负)
      //short enc_A = (short)__HAL_TIM_GET_COUNTER(&htim2); 
      //short enc_B = (short)__HAL_TIM_GET_COUNTER(&htim3); 
      //short enc_C = (short)__HAL_TIM_GET_COUNTER(&htim8); 
      //short enc_D = (short)__HAL_TIM_GET_COUNTER(&htim1);

      // 3. 打印出来看结果
      //printf("PWM=正向 | 编码器脉冲 -> A(右前):%6d  B(右后):%6d  C(左后):%6d  D(左前):%6d\r\n", 
           // enc_A, enc_B, enc_C, enc_D);

      // 清零计数器，模仿 TIM6 的 10ms 周期读取
    //  __HAL_TIM_SET_COUNTER(&htim2, 0);
      //__HAL_TIM_SET_COUNTER(&htim3, 0);
     // __HAL_TIM_SET_COUNTER(&htim8, 0);
     // __HAL_TIM_SET_COUNTER(&htim1, 0);

     // HAL_Delay(50); // 延时 50ms，防止串口打印太快卡死
	 // 2. 获取当前的偏航角
      //float my_yaw = Get_MPU6050_Yaw();
      
      // 3. 通过串口打印出来 (%.2f 表示保留两位小数)
     // printf("当前偏航角 Yaw: %.2f 度\r\n", my_yaw);
      
      // 4. 加一点延时，防止串口打印太快导致电脑卡死
     // HAL_Delay(50);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 所有的引脚外部中断，都会汇聚到这个 HAL 库内置的回调函数里
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 把中断信号扔给超声波的处理函数
    HC_SR04_EXTI_Callback(GPIO_Pin);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 如果是 TIM6 触发的中断 (10ms触发一次)
    if (htim->Instance == TIM6) {
		// 10ms = 0.01 秒，更新陀螺仪角度
        MPU6050_Update_Yaw(0.01f);
        Motor_Calc_PID_10ms();
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
