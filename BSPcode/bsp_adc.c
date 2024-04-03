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

   //����ָ����ͨ��������˳��Ϊ1������ת������
   Adc_Channel = Get_Adc_Channel(ch);
   sConfig.Channel = Adc_Channel;
   sConfig.SamplingTime = SampleTime;
   sConfig.Rank = ADC_REGULAR_RANK_1;
   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
   {
     Error_Handler();
   }

   //����ת������ɶ�Ӧ��ͨ��ת��
   HAL_ADC_Start(&hadc1); 				  //����ADCת��
   HAL_ADC_PollForConversion(&hadc1,200); //�ȴ�ת������

   if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC)) //��ȡADC��ɵı�־λ
   {
	   return HAL_ADC_GetValue(&hadc1); //���ת������ȡ��ֵ
   }
   return 0;
}

//ָ��ͨ����ָ��ת�����ڡ�ָ��ת������ȡƽ��ֵ
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
�������ܣ���ȡ��ص�ѹ
��ڲ�������
����  ֵ����ص�ѹ ��λMV
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int Get_battery_volt(void)
{
	int Volt;                                          //��ص�ѹ
	Volt=Get_Adc(Battery_Ch,_239CYCLES_5)*3.3*11*100/1.0/4096;	   //�����ѹ���������ԭ��ͼ�򵥷������Եõ�
	return Volt;
}


//������ֵ���ض�Ӧ��ͨ����
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
