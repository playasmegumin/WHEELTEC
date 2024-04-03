/*
 * ReadDistance.c
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */
#include "ReadDistance.h"
#include "tim.h"
//定义一个16位的TIM3CH3_CAPTURE_STA变量：用于存放捕获状态
//定义一个16位的TIM3CH3_CAPTURE_VAL变量：用于存放捕获的数值
u16 TIM3CH3_CAPTURE_STA,TIM3CH3_CAPTURE_VAL;

/**************************************************************************
Function: Ultrasonic receiving echo function
Input   : none
Output  : none
函数功能：超声波接收回波函数
入口参数: 无
返回  值：无
**************************************************************************/
void Read_Distane(void)
{
	Trigger_SET;
	 delay_us(15);
	 Trigger_RESET;
	 if(TIM3CH3_CAPTURE_STA&0X80)//成功捕获到了一次高电平
	 {
		 Distance=TIM3CH3_CAPTURE_STA&0X3F;
		 Distance*=65536;					        //溢出时间总和
		 Distance+=TIM3CH3_CAPTURE_VAL;		//得到总的高电平时间
		 Distance=Distance*170/1000;      //时间*声速/2（来回） 一个计数0.001ms
		 TIM3CH3_CAPTURE_STA=0;			//开启下一次捕获
	 }
}


/**************************************************************************
Function: Pulse width reading interruption of ultrasonic echo
Input   : none
Output  : none
函数功能：超声波回波脉宽读取中断①：捕获中断
入口参数: 无
返回  值：无
**************************************************************************/
//进入定时器3中断后，在定时器3中断里判断出是捕获中断，然后进入此回调函数
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//捕获中断发生时执行
{
	if(htim==&htim3)
	{
		if(htim ->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			 if((TIM3CH3_CAPTURE_STA&0X80)==0)//还未成功捕获
			 {
					if(TIM3CH3_CAPTURE_STA&0X40)  //捕获到一个下降沿
					{
					 TIM3CH3_CAPTURE_STA|=0X80;  //标记成功捕获到一次高电平脉宽
					 TIM3CH3_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_3);//获取当前的捕获值.
					 __HAL_TIM_DISABLE(&htim3);											//禁用定时器3
					 TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3);  					 //清除原来的设置
					 TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3,TIM_ICPOLARITY_RISING);//配置TIM3通道3上升沿捕获
					 __HAL_TIM_ENABLE(&htim3);//使能定时器3
					}
					else          //还未开始,第一次捕获上升沿
					{
					 TIM3CH3_CAPTURE_STA=0;   //清空
					 TIM3CH3_CAPTURE_VAL=0;
					 TIM3CH3_CAPTURE_STA|=0X40;  //标记捕获到了上升沿

					 __HAL_TIM_DISABLE(&htim3);        //关闭定时器3
					 __HAL_TIM_SET_COUNTER(&htim3,0);  //计数器CNT置0
					 TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3);   				//清除原来的设置
					 TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);//定时器3通道3设置为下降沿捕获
					 __HAL_TIM_ENABLE(&htim3);//使能定时器3
					}
			 }
		}
	}
}

/**************************************************************************
Function: Pulse width reading interruption of ultrasonic echo
Input   : none
Output  : none
函数功能：超声波回波脉宽读取中断②：更新中断
入口参数: 无
返回  值：无
**************************************************************************/
//定时器更新中断（计数溢出）中断处理回调函数， 该函数在HAL_TIM_IRQHandler中会被调用
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//更新中断（溢出）发生时执行
{
	if(htim==&htim3)
	{
		 if((TIM3CH3_CAPTURE_STA&0X80)==0)//还未成功捕获
		 {
			if(TIM3CH3_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
			 if((TIM3CH3_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
			 {
				TIM3CH3_CAPTURE_STA|=0X80;  //标记成功捕获了一次
				TIM3CH3_CAPTURE_VAL=0XFFFF;
			 }
			 else
				TIM3CH3_CAPTURE_STA++;
			}
		 }
	}

}
