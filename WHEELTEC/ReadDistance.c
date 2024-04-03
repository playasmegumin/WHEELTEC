/*
 * ReadDistance.c
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */
#include "ReadDistance.h"
#include "tim.h"
//����һ��16λ��TIM3CH3_CAPTURE_STA���������ڴ�Ų���״̬
//����һ��16λ��TIM3CH3_CAPTURE_VAL���������ڴ�Ų������ֵ
u16 TIM3CH3_CAPTURE_STA,TIM3CH3_CAPTURE_VAL;

/**************************************************************************
Function: Ultrasonic receiving echo function
Input   : none
Output  : none
�������ܣ����������ջز�����
��ڲ���: ��
����  ֵ����
**************************************************************************/
void Read_Distane(void)
{
	Trigger_SET;
	 delay_us(15);
	 Trigger_RESET;
	 if(TIM3CH3_CAPTURE_STA&0X80)//�ɹ�������һ�θߵ�ƽ
	 {
		 Distance=TIM3CH3_CAPTURE_STA&0X3F;
		 Distance*=65536;					        //���ʱ���ܺ�
		 Distance+=TIM3CH3_CAPTURE_VAL;		//�õ��ܵĸߵ�ƽʱ��
		 Distance=Distance*170/1000;      //ʱ��*����/2�����أ� һ������0.001ms
		 TIM3CH3_CAPTURE_STA=0;			//������һ�β���
	 }
}


/**************************************************************************
Function: Pulse width reading interruption of ultrasonic echo
Input   : none
Output  : none
�������ܣ��������ز������ȡ�жϢ٣������ж�
��ڲ���: ��
����  ֵ����
**************************************************************************/
//���붨ʱ��3�жϺ��ڶ�ʱ��3�ж����жϳ��ǲ����жϣ�Ȼ�����˻ص�����
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//�����жϷ���ʱִ��
{
	if(htim==&htim3)
	{
		if(htim ->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			 if((TIM3CH3_CAPTURE_STA&0X80)==0)//��δ�ɹ�����
			 {
					if(TIM3CH3_CAPTURE_STA&0X40)  //����һ���½���
					{
					 TIM3CH3_CAPTURE_STA|=0X80;  //��ǳɹ�����һ�θߵ�ƽ����
					 TIM3CH3_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_3);//��ȡ��ǰ�Ĳ���ֵ.
					 __HAL_TIM_DISABLE(&htim3);											//���ö�ʱ��3
					 TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3);  					 //���ԭ��������
					 TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3,TIM_ICPOLARITY_RISING);//����TIM3ͨ��3�����ز���
					 __HAL_TIM_ENABLE(&htim3);//ʹ�ܶ�ʱ��3
					}
					else          //��δ��ʼ,��һ�β���������
					{
					 TIM3CH3_CAPTURE_STA=0;   //���
					 TIM3CH3_CAPTURE_VAL=0;
					 TIM3CH3_CAPTURE_STA|=0X40;  //��ǲ�����������

					 __HAL_TIM_DISABLE(&htim3);        //�رն�ʱ��3
					 __HAL_TIM_SET_COUNTER(&htim3,0);  //������CNT��0
					 TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3);   				//���ԭ��������
					 TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);//��ʱ��3ͨ��3����Ϊ�½��ز���
					 __HAL_TIM_ENABLE(&htim3);//ʹ�ܶ�ʱ��3
					}
			 }
		}
	}
}

/**************************************************************************
Function: Pulse width reading interruption of ultrasonic echo
Input   : none
Output  : none
�������ܣ��������ز������ȡ�жϢڣ������ж�
��ڲ���: ��
����  ֵ����
**************************************************************************/
//��ʱ�������жϣ�����������жϴ���ص������� �ú�����HAL_TIM_IRQHandler�лᱻ����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//�����жϣ����������ʱִ��
{
	if(htim==&htim3)
	{
		 if((TIM3CH3_CAPTURE_STA&0X80)==0)//��δ�ɹ�����
		 {
			if(TIM3CH3_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
			 if((TIM3CH3_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
			 {
				TIM3CH3_CAPTURE_STA|=0X80;  //��ǳɹ�������һ��
				TIM3CH3_CAPTURE_VAL=0XFFFF;
			 }
			 else
				TIM3CH3_CAPTURE_STA++;
			}
		 }
	}

}
