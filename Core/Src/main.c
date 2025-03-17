/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5600.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#include "micro_5ms.h"
#include "PID.h"
#include "SVPWM.h"
#include "acceleration.h"
#include "Routine.h"
#include <stdio.h>
//位置-速度-加速度

#define M0_Acceleration_P 0.5
#define M0_Acceleration_I 0.8 //此I环不同解决两电机不同阻尼问题
#define M0_Acceleration_D 0.1 //此D环不同解决两电机不同阻尼问题
#define M1_Acceleration_P 0.5
#define M1_Acceleration_I 0.2
#define M1_Acceleration_D 0.1
#define M0_Velocity_P 20
#define M0_Velocity_I 1
#define M0_Velocity_D 0.2
#define M1_Velocity_P 20
#define M1_Velocity_I 1
#define M1_Velocity_D 0.2
#define M0_Angle_P 100
#define M0_Angle_I 0
#define M0_Angle_D 10
#define M1_Angle_P 100
#define M1_Angle_I 0
#define M1_Angle_D 10

////速度
//#define M0_Velocity_P 1
//#define M0_Velocity_I 5
//#define M0_Velocity_D 0
//#define M1_Velocity_P 1  
//#define M1_Velocity_I 5
//#define M1_Velocity_D 0
//#define M0_Angle_P 20
//#define M0_Angle_I 0
//#define M0_Angle_D 0.1
//#define M1_Angle_P 20
//#define M1_Angle_I 0
//#define M1_Angle_D 0.1

#define MAX_Speed_M0 0.515
//速度最大值
#define MAX_Speed_M1 0.5

#define MAX_Acceleratio_M0 5//加速度最大值
#define MAX_Acceleratio_M1 5

#define MAX_AcAcceleratio_M0 6 
#define MAX_AcAcceleratio_M1 6
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile uint64_t REAL_TIME = 0;

float offset_ia_1 = 0;
float offset_ib_1 = 0;

float Current_a_1;
float Current_b_1;
float Current_c_1;


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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	if (htim==(&htim5))
	{
		REAL_TIME++;
	}
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  //初始化pwm输出
    
  HAL_TIM_PWM_Start((TIM_HandleTypeDef *)&htim2, (uint32_t) TIM_CHANNEL_1);
  HAL_TIM_PWM_Start((TIM_HandleTypeDef *)&htim2, (uint32_t) TIM_CHANNEL_2);
  HAL_TIM_PWM_Start((TIM_HandleTypeDef *)&htim2, (uint32_t) TIM_CHANNEL_3);
  HAL_TIM_PWM_Start((TIM_HandleTypeDef *)&htim3, (uint32_t) TIM_CHANNEL_1);
  HAL_TIM_PWM_Start((TIM_HandleTypeDef *)&htim3, (uint32_t) TIM_CHANNEL_2);
  HAL_TIM_PWM_Start((TIM_HandleTypeDef *)&htim3, (uint32_t) TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim5); 
  
  //初始化adc
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  HAL_ADC_PollForConversion(&hadc2, 1000);
  
  //初始化pid参数
  Acceleration_PIDInit_M0(M0_Acceleration_P, M0_Acceleration_I, M0_Acceleration_D);
  Acceleration_PIDInit_M1(M1_Acceleration_P, M1_Acceleration_I, M1_Acceleration_D);
  Velocity_PIDInit_M0(M0_Velocity_P ,M0_Velocity_I, M0_Velocity_D);
  Velocity_PIDInit_M1(M1_Velocity_P ,M1_Velocity_I, M1_Velocity_D);
  Angle_PIDInit_M0(M0_Angle_P ,M0_Angle_I, M0_Angle_D);
  Angle_PIDInit_M1(M1_Angle_P ,M1_Angle_I, M1_Angle_D);
  
  //初始化低通滤波器
  Vel_LowPass_FilterInit_M0();
  Vel_LowPass_FilterInit_M1();
  Acl_LowPass_FilterInit_M0();
  Acl_LowPass_FilterInit_M1();
  
  //初始化编码器
  bsp_as5600_1_Init();
  bsp_as5600_2_Init();
  
  //初始化看门狗
  MX_IWDG_Init();

//控制引脚使能
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  
  //设定速度最大值（小车）
  set_MAX_Velocity_M0(MAX_Speed_M0 / 0.04);
  set_MAX_Velocity_M1(MAX_Speed_M1 / 0.04);
  
  //设定加速度最大值
  set_MAX_Acceleration_M0(MAX_Acceleratio_M0);
  set_MAX_Acceleration_M1(MAX_Acceleratio_M1);
  
  //设定加加速度最大值
  set_MAX_AcAcceleration_M0(MAX_AcAcceleratio_M0);
  set_MAX_AcAcceleration_M1(MAX_AcAcceleratio_M1);  
  

//  char Send_Data[50];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  Calculate_CarVelocity(0.5, 0);
//    M0_set_Force_Angle(20); //闭环角度
//	  M0_setTorque(4, micro_5us()*7*5e-6f*5); //开环
//	  M0_SetVelocity(5); //闭环速度
//	  M1_SetVelocity(-5); //闭环速度
	  //调试串口
//	float time_send = s0_GetAcAcceleration();
//	sprintf(Send_Data,"%f",time_send);
//	  
//	  if((micro_5us() % 100000) <= 20)
//	  {
//		HAL_UART_Transmit(&huart1,(uint8_t *)Send_Data,5,1000);
//	  }
//	  
	  if (REAL_TIME > 200000)
	  {
		Calculate_CarDisPlacement(10, 0);
	  }
	  
	  
	  //喂狗
	  HAL_IWDG_Refresh(&hiwdg);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
