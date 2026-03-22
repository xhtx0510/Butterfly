/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  
/**
************************************************************************************************
* @brief    独立驱动、姿态调节与参数控制
* @param    None
* @return   None
* @author   创源启明		2025.12.28
************************************************************************************************
**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "elrs.h"
#include "AS5600_PWM.h"
#include <stdint.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */






// 左右电机的基准点（中位角度//调小翅膀向上）
int16_t motor_L_midpoint   = 2150;
int16_t motor_R_midpoint   = 2150;

// 相位累加器（16位：0..65535 表示一整周期）
static uint16_t g_phase16 = 0;

// 上一次循环的时间戳（ms）
static uint32_t g_last_ms = 0;
int motor_L_set;          //左电机目标角度
int motor_R_set;          //右电机目标角度
int Throttle,Pitch,Roll,Yaw;   //油门2，俯仰1，横滚0，偏航3

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  const long run = in_max - in_min;
  if (run == 0) {
    return -1;  // AVR returns -1, SAM returns 0
  }
  const long rise = out_max - out_min;
  const long delta = x - in_min;
  return (delta * rise) / run + out_min;
}


void motor_disable()//翅膀失能
{
	TIM2->CCR1=0;
	TIM2->CCR2=0;
	TIM3->CCR1=0;
	TIM3->CCR2=0;

}
void motor_stop()//翅膀暂停
{
	TIM2->CCR1=19999;
	TIM2->CCR2=19999;
	TIM3->CCR1=19999;
	TIM3->CCR2=19999;
}


void motor_test()//调试前翅膀水平打开，运行后翅膀向下摆动，即为电机方向正确
{
	Set_Pwm(3000,3000,3000,3000);
	HAL_Delay(100); 
	Set_Pwm(0, 0, 0, 0);
	while(1);
}

// 分支消除的 int16 绝对值（两补码）
static inline uint16_t abs16_fast(int16_t x) {
    int16_t m = x >> 15;            // x<0 → -1；x>=0 → 0
    return (uint16_t)((x ^ m) - m); // 等价于 abs(x)
}
// ====== 定点余弦表（Q15），15 点：20°..160° ======
static const int16_t COS_Q15_15[9] = {
    30784, 25133,  16384,
    11207,     0, -11207,
   -16384,-25133,-30784
};

// Q15 乘法： (a * b) >> 15
static inline int16_t q15_mul(int16_t a, int16_t b) {
    return (int16_t)(((int32_t)a * (int32_t)b) >> 15);
}

// ====== 非阻塞状态机的静态状态 ======
static uint8_t  sm_idx = 13;          // 0..14
static int8_t   sm_dir = 1;          // +1 正向 / -1 反向
static uint32_t sm_next_tick = 0;    // 下次步进的时刻（ms）
static uint8_t  thr = 0;
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//int ii=0;
int ii1=0;
int ii2=100;
int motor_i = 0;
int Take_off_Flag = 0;
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	
	
	HAL_Delay(1000);		//等待上电完成
	MX_USART1_UART_Init();	//开启接收机串口
	ELRS_Init();			//接收机初始化
	Chassis_PID_Init(); 	//电机PID初始化
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	g_last_ms = HAL_GetTick();  // 初始化时间戳

  while (1)
  {
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	StarAndGetResult();
	uint32_t now   = HAL_GetTick();
	(void)now;

    if (elrs_data.Switch == 1)
    {

        if (elrs_data.Mode == 2)
        {
			motor_1_pid.Kp = 30;
			motor_3_pid.Kp = 30;
			const int16_t motor_L_ready		=	motor_L_midpoint	+	elrs_data.midpoint	+	elrs_data.midpoint_1;
			const int16_t motor_R_ready		=	motor_R_midpoint	+	elrs_data.midpoint	+	elrs_data.midpoint_1;

            // 幅值（保持与原设计一致：Pitch ± Yaw，整型）
            const int16_t ampR = (int16_t)(800 + elrs_data.Yaw);			
            const int16_t ampL = (int16_t)(800 - elrs_data.Yaw);
			thr = 12;
			// 一个周期 30 步（15 去 + 15 回）
			// 频率 = thr/5  Hz；周期(ms) = 1000/f = 5000/thr
			// 每步时长 step_ms = 周期/30 = 5000 / (30*thr)
			const uint32_t STEPS = 30U;
			uint32_t step_ms = (5000U + (STEPS*thr/2U)) / (STEPS * thr);  // 四舍五入
			if (step_ms == 0) step_ms = 1;

			uint32_t tick = HAL_GetTick();
			
                if ((int32_t)(tick - sm_next_tick) >= 0)
                {
                    // 到时间：推进一步
                    int16_t c = COS_Q15_15[sm_idx];

                    // set = base - amp * cosQ15（定点）
                    Wings_Data.Wings_motor[0].Target_Angle	=	(int16_t)(motor_L_ready		-	q15_mul(ampL, c));
                    Wings_Data.Wings_motor[2].Target_Angle	=	(int16_t)(motor_R_ready		-	q15_mul(ampR, c));



                    // 安排下一次步进（按“绝对时标”推进，抖动小）
                    sm_next_tick += step_ms;

                    // 更新索引与方向（首尾“来回”扫）
                    if (sm_dir > 0) {
                        if (++sm_idx >= 8) { sm_idx = 8; sm_dir = -1; }
                    } else {
                        if (sm_idx-- == 0)   { sm_idx = 0;  sm_dir = +1; }
                    }
                }
				StarAndGetResult();
				Motor_PID_Control();
            
        }
        else if (elrs_data.Mode == 1)
		{
			motor_1_pid.Kp = 10;
			motor_3_pid.Kp = 10;
			// 基准角
			const int16_t ago_targetL		=	motor_L_midpoint	+	elrs_data.midpoint	+	elrs_data.midpoint_1	-	elrs_data.Yaw;
			const int16_t ago_targetR		=	motor_R_midpoint	+	elrs_data.midpoint	+	elrs_data.midpoint_1	+	elrs_data.Yaw;
			
			// 从当前值缓慢移动到目标值
			static uint32_t last_smooth_time = 0;
			const uint32_t SMOOTH_INTERVAL = 5; // 平滑间隔 5ms
			const int16_t MAX_STEP = 70;        // 最大步进值
			
			if ((int32_t)(now - last_smooth_time) >= SMOOTH_INTERVAL)
			{
				// 电机0（左前）平滑移动
				int16_t current0 = Wings_Data.Wings_motor[0].Corrective_Angle;
				if (current0 < ago_targetL) {
					current0 += MIN(MAX_STEP, ago_targetL - current0);
				} else if (current0 > ago_targetL) {
					current0 -= MIN(MAX_STEP, current0 - ago_targetL);
				}
				Wings_Data.Wings_motor[0].Target_Angle = current0;
				
				// 电机2（右前）平滑移动
				int16_t current2 = Wings_Data.Wings_motor[2].Corrective_Angle;
				if (current2 < ago_targetR) {
					current2 += MIN(MAX_STEP, ago_targetR - current2);
				} else if (current2 > ago_targetR) {
					current2 -= MIN(MAX_STEP, current2 - ago_targetR);
				}
				Wings_Data.Wings_motor[2].Target_Angle = current2;
				
				last_smooth_time = now;
			}
		
			
			StarAndGetResult();
			Motor_PID_Control();

			sm_idx = 0;						// 0..14
			sm_dir = 1;         			// +1 正向 / -1 反向
			sm_next_tick = HAL_GetTick();	// 下次步进的时刻（ms）
	
		}
        else
		{
			motor_1_pid.Kp = 12;
			motor_3_pid.Kp = 12;
			// 目标角度
			const int16_t ago_targetL		=	1024;
			const int16_t ago_targetR		=	1024;

			
			// 从当前值缓慢移动到目标值
			static uint32_t last_smooth_time_mode0 = 0;
			const uint32_t SMOOTH_INTERVAL = 10; // 平滑间隔10ms
			const int16_t MAX_STEP = 70;         // 最大步进值
			
			if ((int32_t)(now - last_smooth_time_mode0) >= SMOOTH_INTERVAL)
			{
				// 电机0（左前）平滑移动
				int16_t current0 = Wings_Data.Wings_motor[0].Corrective_Angle;
				if (current0 < ago_targetL) {
					current0 += MIN(MAX_STEP, ago_targetL - current0);
				} else if (current0 > ago_targetL) {
					current0 -= MIN(MAX_STEP, current0 - ago_targetL);
				}
				Wings_Data.Wings_motor[0].Target_Angle = current0;

	
				// 电机2（右前）平滑移动
				int16_t current2 = Wings_Data.Wings_motor[2].Corrective_Angle;
				if (current2 < ago_targetR) {
					current2 += MIN(MAX_STEP, ago_targetR - current2);
				} else if (current2 > ago_targetR) {
					current2 -= MIN(MAX_STEP, current2 - ago_targetR);
				}
				Wings_Data.Wings_motor[2].Target_Angle = current2;

				
				last_smooth_time_mode0 = now;
			}
			if(abs16_fast(ago_targetL-Wings_Data.Wings_motor[0].Corrective_Angle)>200&abs16_fast(ago_targetL-Wings_Data.Wings_motor[0].Corrective_Angle)>200)
			{
			motor_1_pid.Kp=12*(float)abs16_fast(ago_targetL-Wings_Data.Wings_motor[0].Corrective_Angle)/(float)abs16_fast(ago_targetR-Wings_Data.Wings_motor[2].Corrective_Angle);
			motor_3_pid.Kp=12*(float)abs16_fast(ago_targetR-Wings_Data.Wings_motor[2].Corrective_Angle)/(float)abs16_fast(ago_targetL-Wings_Data.Wings_motor[0].Corrective_Angle);	
			}
			StarAndGetResult();
			Motor_PID_Control();

			sm_idx = 0;						// 0..14
			sm_dir = 1;          			// +1 正向 / -1 反向
			sm_next_tick = HAL_GetTick();	// 下次步进的时刻（ms）
	
		}

    }
    else
    {
        motor_disable();
				sm_idx = 0;						// 0..14
				sm_dir = 1;          			// +1 正向 / -1 反向
				sm_next_tick = HAL_GetTick();	// 下次步进的时刻（ms）
	
    }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)  //串口接收中断回调函数
{
	if(huart == &huart1)   //判断串口1中断	
	{
		ELRS_UARTE_RxCallback(Size);

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

#ifdef  USE_FULL_ASSERT
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
