/*
 * adc.c
 *
 *  Created on: Jun 11, 2022
 *      Author: WHEELTEC
 */
#include "bsp_adc.h"
#include "adc.h"

uint16_t Get_Adc(uint8_t ch,uint32_t SampleTime)
{
   uint32_t Adc_Channel;
   ADC_ChannelConfTypeDef sConfig = {0};

   //传入指定的通道，设置顺序为1，设置转换周期
   Adc_Channel = Get_Adc_Channel(ch);
   sConfig.Channel = Adc_Channel;
   sConfig.SamplingTime = SampleTime;
   sConfig.Rank = ADC_REGULAR_RANK_1;
   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
   {
     Error_Handler();
   }

   //启动转换，完成对应的通道转换
   HAL_ADC_Start(&hadc1); 				  //启动ADC转换
   HAL_ADC_PollForConversion(&hadc1,200); //等待转换结束

   if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC)) //读取ADC完成的标志位
   {
	   return HAL_ADC_GetValue(&hadc1); //完成转换，读取数值
   }
   return 0;
}

//指定通道、指定转换周期、指定转换次数取平均值
uint16_t Get_Adc_Average(uint8_t ch,uint32_t SampleTime,uint8_t times)
{
	uint32_t temp_val = 0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch,SampleTime);
	}
	return temp_val/times;
}

/**************************************************************************
函数功能：读取电池电压
入口参数：无
返回  值：电池电压 单位MV
作    者：平衡小车之家
**************************************************************************/
int Get_battery_volt(void)
{
	int Volt;                                          //电池电压
	Volt=Get_Adc(Battery_Ch,_239CYCLES_5)*3.3*11*100/1.0/4096;	   //电阻分压，具体根据原理图简单分析可以得到
	return Volt;
}


//输入数值返回对应的通道数
uint32_t Get_Adc_Channel(uint8_t ch)
{
   if(ch==0) 		 return  ADC_CHANNEL_0;
   else if (ch==1)   return  ADC_CHANNEL_1;
   else if (ch==2)   return  ADC_CHANNEL_2;
   else if (ch==3)   return  ADC_CHANNEL_3;
   else if (ch==4)   return  ADC_CHANNEL_4;
   else if (ch==5)   return  ADC_CHANNEL_5;
   else if (ch==6)   return  ADC_CHANNEL_6;
   else if (ch==7)   return  ADC_CHANNEL_7;
   else if (ch==8)   return  ADC_CHANNEL_8;
   else if (ch==9)   return  ADC_CHANNEL_9;
   else if (ch==10)  return  ADC_CHANNEL_10;
   else if (ch==11)  return  ADC_CHANNEL_11;
   else if (ch==12)  return  ADC_CHANNEL_12;
   else if (ch==13)  return  ADC_CHANNEL_13;
   else if (ch==14)  return  ADC_CHANNEL_14;
   else if (ch==15)  return  ADC_CHANNEL_15;
   else if (ch==16)  return  ADC_CHANNEL_16;
   else if (ch==17)  return  ADC_CHANNEL_17;
   else return 0;
}
