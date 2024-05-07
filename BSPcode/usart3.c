/*
 * usart3.c
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */
#include "usart3.h"
#include "usart.h"

u8 Usart3_Receive_buf[1];          //����3�����ж����ݴ�ŵĻ�����
u8 Usart3_Receive;                 //�Ӵ���3��ȡ������

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) //���ջص�����
{
	if(UartHandle == &huart3)
	{
		static	int uart_receive=0;//����������ر���
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
		uart_receive=Usart3_Receive_buf[0];
		Usart3_Receive=uart_receive;
		if(uart_receive==0x59)  Flag_velocity=2;  //���ٵ���Ĭ��ֵ��
		if(uart_receive==0x58)  Flag_velocity=1;  //���ٵ�

	if(uart_receive>10)																		// Ĭ��ʹ��
    {																						// �Զ��尴��Ϊbcdefghijklmno
			if(uart_receive==0x5A)	    Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//ɲ��
			else if(uart_receive==0x41)	Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;//ǰ
			else if(uart_receive==0x45)	Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;//��
			else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)
																	Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //��
			else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)
																	Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;  //��
			else Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//ɲ��
  	}
	if(uart_receive<10)     //����appΪ��MiniBalanceV1.0  ��ΪMiniBalanceV1.0��ң��ָ��ΪA~H ��HEX��С��10
	{
		Flag_velocity=1;//�л������ٵ�
		if(uart_receive==0x00)	Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//ɲ��
		else if(uart_receive==0x01)	Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;//ǰ
		else if(uart_receive==0x05)	Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;//��
		else if(uart_receive==0x02||uart_receive==0x03||uart_receive==0x04)
													Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //��
		else if(uart_receive==0x06||uart_receive==0x07||uart_receive==0x08)	    //��
													Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;
		else Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//ɲ��
  	}


		if(Usart3_Receive==0x7B) Flag_PID=1;   //APP����ָ����ʼλ
		if(Usart3_Receive==0x7D) Flag_PID=2;   //APP����ָ��ֹͣλ

		 if(Flag_PID==1)  //�ɼ�����
		 {
				Receive[i]=Usart3_Receive;
				i++;
		 }
		 if(Flag_PID==2)  //��������
		 {
			  if(Receive[3]==0x50) 				 PID_Send=1;
			  else if(Receive[1]!=0x23)
				{
					for(j=i;j>=4;j--)
					{
						Data+=(Receive[j-1]-48)*pow(10,i-j);
					}
					switch(Receive[1])
					{
						case 0x30:  Balance_Kp=Data;break;
						case 0x31:  Balance_Kd=Data;break;
						case 0x32:  Velocity_Kp=Data;break;
						case 0x33:  Velocity_Ki=Data;break;
						case 0x34:  Turn_Kp=Data;break;
						case 0x35:  Turn_Kd=Data;break;
						// GUA: �ĳ��˶�Ӧָ���ʱ��
						case 0x36:  job_dir[1][2]=Data;break; // ǰ��ʱ��
						case 0x37:  job_dir[2][2]=Data;break; // ��תʱ��
						case 0x38:  job_dir[3][2]=Data;break; // ��תʱ��
					}
				}
			    	Flag_PID=0;
					i=0;
					j=0;
					Data=0;
					memset(Receive, 0, sizeof(u8)*50);//��������
		 }

		HAL_UART_Receive_IT(&huart3,Usart3_Receive_buf,sizeof(Usart3_Receive_buf));//����3�ص�����ִ�����֮����Ҫ�ٴο��������жϵȴ���һ�ν����жϵķ���
	}
}



