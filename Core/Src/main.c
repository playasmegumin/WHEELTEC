/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
u8 Way_Angle=2;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波
u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity=2; //蓝牙遥控相关的变量
u8 Flag_Stop=1,Flag_Show=0;                 //电机停止标志位和显示标志位  默认停止 显示打开
int Motor_Left,Motor_Right;                 //电机PWM变量 应是Motor的 向Moto致敬
int Temperature;                            //温度变量
int Voltage;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
u32 Distance;                               //超声波测距
u8 PID_Send;
volatile u8 delay_50,delay_flag; 						//延时和调参相关变量
u8 Flag_follow=0,Flag_avoid=0;							//超声波跟随、超声波壁障标志位
u8 Flag_swing=0;										//GUAHOOK 摇摆模式
float Velocity_Left,Velocity_Right;	//车轮速度(mm/s)
float Acceleration_Z;                       //Z轴加速度计
float Balance_Kp=22500,Balance_Kd=108,Velocity_Kp=16000,Velocity_Ki=80,Turn_Kp=4200,Turn_Kd=0;//PID参数（放大100倍）
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); //先关闭配置的外部中断，防止初始化过程中进入中断
  delay_init(); 						//释放SysTick，使用DWT作为HAL库时钟

  //HAL库不会自动配置部分
  HAL_ADCEx_Calibration_Start(&hadc1);//启动ADC1校准，不校准将导致ADC测量不准确

  /* PWM输出控制电机 */
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//开启定时器1通道1的PWM输出，左轮电机使用
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//开启定时器1通道4的PWM输出，右轮电机使用

  /* 使能编码器 */
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);//开启定时器2的编码器模式
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);//开启定时器4的编码器模式

  /* 使能定时器3输入捕获，用于超声波 */
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);//打开定时器3通道3的输入捕获中断
  HAL_TIM_Base_Start_IT(&htim3);            //打开定时器3通道3的更新中断

  OLED_Init();  						//OLED初始化
  MPU6050_initialize();           		//MPU6050初始化
  DMP_Init();                     		//初始化DMP
  RetargetInit(&huart3);				//映射printf函数到串口3中给APP使用

  /* 开启接收串口3接收中断 */
  HAL_UART_Receive_IT(&huart3,Usart3_Receive_buf,sizeof(Usart3_Receive_buf));

  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); //开启引脚外部中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(Flag_Show==0)          		//使用MiniBalance APP和OLED显示屏
	{
		 APP_Show();								//发送数据给APP
		 oled_show();          			//显示屏打开
	}
	else                      		//使用MiniBalance上位机 上位机使用的时候需要严格的时序，故此时关闭app监控部分和OLED显示屏
	{
		 DataScope();          			//开启MiniBalance上位机
	}
	delay_flag=1;
	delay_50=0;
	while(delay_flag);	     			//示波器需要50ms	高精度延时，delay函数不满足要求，故使用MPU6050中断提供50ms延时
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
