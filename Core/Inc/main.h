/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//简化变量的定义
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

//C语言库相关头文件
#include "stdio.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

//MPU6050DMP库相关
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "MPU6050.h"
#include "filter.h"

//板级支持包相关
#include "bsp_adc.h"
#include "delay.h"
#include "encoder.h"
#include "key.h"
#include "LED.h"
#include "oled.h"
#include "usart3.h"
#include "ReadDistance.h"
#include "iic.h"
#include "retarget.h"

//控制函数相关
#include "control.h"
#include "DataScope_DP.h"
#include "show.h"
#include "KF.h"
//外部声明方便使用
extern u8 Way_Angle;                                       				 //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波
extern int Motor_Left,Motor_Right;                                 //电机PWM变量 应是motor的 向moto致敬
extern u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity; //蓝牙遥控相关的变量
extern u8 Flag_Stop,Flag_Show;                               			 //停止标志位和 显示标志位 默认停止 显示打开
extern int Voltage;               																 //电池电压采样相关的变量
extern float Angle_Balance,Gyro_Balance,Gyro_Turn;     						 //平衡倾角 平衡陀螺仪 转向陀螺仪
extern int Temperature;
extern u32 Distance;                                          		//超声波测距
extern u8 Flag_follow,Flag_avoid,Flag_swing;
extern volatile u8 delay_50,delay_flag;
extern u8 PID_Send;
extern float Acceleration_Z;                       //Z轴加速度计
extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Kd;
extern float Velocity_Left,Velocity_Right;	//车轮速度(mm/s)
//外部声明当前速度
extern u8 sin100_counter;
extern float sin100[100];
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Encoder_A_Pin GPIO_PIN_0
#define Encoder_A_GPIO_Port GPIOA
#define Encoder_B_Pin GPIO_PIN_1
#define Encoder_B_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOA
#define User_key_Pin GPIO_PIN_5
#define User_key_GPIO_Port GPIOA
#define UltrasonicCapture_Pin GPIO_PIN_0
#define UltrasonicCapture_GPIO_Port GPIOB
#define Trigger_Pin GPIO_PIN_1
#define Trigger_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_12
#define BIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_13
#define BIN1_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_14
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_15
#define AIN2_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_8
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_11
#define PWMB_GPIO_Port GPIOA
#define MPU6050_INT_Pin GPIO_PIN_12
#define MPU6050_INT_GPIO_Port GPIOA
#define MPU6050_INT_EXTI_IRQn EXTI15_10_IRQn
#define OLED_DC_Pin GPIO_PIN_15
#define OLED_DC_GPIO_Port GPIOA
#define OLED_RES_Pin GPIO_PIN_3
#define OLED_RES_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_4
#define OLED_SDA_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_5
#define OLED_SCL_GPIO_Port GPIOB
#define IIC_SCL_Pin GPIO_PIN_8
#define IIC_SCL_GPIO_Port GPIOB
#define IIC_SDA_Pin GPIO_PIN_9
#define IIC_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
