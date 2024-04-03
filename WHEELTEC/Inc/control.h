/*
 * control.h
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_
#include "main.h"

//���������غ궨��
#define AIN1_SET    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET)
#define AIN1_RESET  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET)
#define AIN2_SET    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET)
#define AIN2_RESET  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET)
#define BIN1_SET    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET)
#define BIN1_RESET  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET)
#define BIN2_SET    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET)
#define BIN2_RESET  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET)
#define PWMA TIM1->CCR1
#define PWMB TIM1->CCR4

#define PI 3.14159265							//PIԲ����
#define Control_Frequency  200.0	//��������ȡƵ��
#define Diameter_67  67.0 				//����ֱ��67mm
#define EncoderMultiples   4.0 		//��������Ƶ��
#define Encoder_precision  13.0 	//���������� 13��
#define Reduction_Ratio  30.0			//���ٱ�30
#define Perimeter  210.4867 			//�ܳ�����λmm

#define Middle_angle 0
#define DIFFERENCE 100
int Balance(float angle,float gyro);
int Velocity(int encoder_left,int encoder_right);
int Turn(float gyro);
void Set_Pwm(int motor_left,int motor_right);
void Key(void);
void Limit_Pwm(void);
int PWM_Limit(int IN,int max,int min);
u8 Turn_Off(float angle, int voltage);
void Get_Angle(u8 way);
int myabs(int a);
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
void Choose(int encoder_left,int encoder_right);

#endif /* INC_CONTROL_H_ */
