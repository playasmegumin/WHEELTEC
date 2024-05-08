/*
 * usart3.c
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */
#include "usart3.h"
#include "usart.h"

u8 Usart3_Receive_buf[1];          //串口3接收中断数据存放的缓冲区
u8 Usart3_Receive;                 //从串口3读取的数据

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) //接收回调函数
{
	if(UartHandle == &huart3)
	{
		static	int uart_receive=0;//蓝牙接收相关变量
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
		uart_receive=Usart3_Receive_buf[0];
		Usart3_Receive=uart_receive;
		if(uart_receive==0x59)  Flag_velocity=2;  //低速挡（默认值）
		if(uart_receive==0x58)  Flag_velocity=1;  //高速档

	if(uart_receive>10)																		// 默认使用
    {																						// 自定义按键为bcdefghijklmno
			if(uart_receive==0x5A)	    Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
			else if(uart_receive==0x41)	Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;//前
			else if(uart_receive==0x45)	Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;//后
			else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)
																	Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //右
			else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)
																	Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;  //左
			// else Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
			else{
				switch(uart_receive) {
				// GUAHOOK: 按键控制
				// 为了实现暂停效果，遥控队列每个指令输入之后效果也如job_list前面的预设队列这样（如下）
				// int job_list[40] = {0,0,0,1,0,2,0,1,0,2,0,1,0,2,0,1,0};
				// 每一个action后面都会带一个0，这样行动与行动之间会自带一个停止的空档
				// 同时这个自带的停止行动也可以替换成暂停行动
				// 如遥控app输入行动是 2 1 3 1 3 1 2，在遥控队列中会变成 2 0 1 0 3 0 1 0 3 0 1 0 2 0
				case 0x61:												// 清空队列：并使pointer卡在21，pointer在22时正式开始
					for(u8 i = 20; i < 60;i++) job_list[i] = 0;
					job_counter = 0, job_pointer = 20, sin100[0] = 0;
					job_usart_pointer = 22;
					job_list[21] = 4;
					Flag_Left = 0, Flag_Right = 0;
					break;
				case 0x62:		 										// 从头启动队列
					job_list[21] = 0;
					job_counter = 0, job_pointer = 20, sin100[0] = 0;
					Flag_Left = 0, Flag_Right = 0;
					break;
				case 0x63:	 											// 暂停队列：保守的做法，如果当前已经是停止的空档了就进行完下一步行动再暂停
					if(job_pointer > 19)
					{
						if(job_pointer & 1) job_list[job_pointer + 2] = 4;
						else job_list[job_pointer + 1] = 4;
					}
					break;
				case 0x64:		 										// 取消暂停：直接把所有=4变成=0就取消暂停了
					for(u8 i = 21;i < 60;i += 2) job_list[i] = 0;
					break;
				case 0x65:	 		// 停止
					job_list[job_usart_pointer] = 0;
					job_list[job_usart_pointer+1] = 0;
					if(job_usart_pointer < 58) job_usart_pointer += 2;
					break;
				case 0x66:	 		// 躺倒！
					Flag_Stop = 1;
					break;
				case 0x67:			// 左转
					job_list[job_usart_pointer] = 2;
					job_list[job_usart_pointer+1] = 0;
					if(job_usart_pointer < 58) job_usart_pointer += 2;
					break;
				case 0x68:	 		// 前进
					job_list[job_usart_pointer] = 1;
					job_list[job_usart_pointer+1] = 0;
					if(job_usart_pointer < 58) job_usart_pointer += 2;
					break;
				case 0x69:	 		// 右转
					job_list[job_usart_pointer] = 3;
					job_list[job_usart_pointer+1] = 0;
					if(job_usart_pointer < 58) job_usart_pointer += 2;
					break;
				case 0x6F:
					job_pointer = 0;
					break;
				default:
					Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;
					break;
				}
			}
  	}
	if(uart_receive<10)     //备用app为：MiniBalanceV1.0  因为MiniBalanceV1.0的遥控指令为A~H 其HEX都小于10
	{
		Flag_velocity=1;//切换至高速档
		if(uart_receive==0x00)	Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
		else if(uart_receive==0x01)	Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;//前
		else if(uart_receive==0x05)	Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;//后
		else if(uart_receive==0x02||uart_receive==0x03||uart_receive==0x04)
													Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //左
		else if(uart_receive==0x06||uart_receive==0x07||uart_receive==0x08)	    //右
													Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;
		else Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
  	}


		if(Usart3_Receive==0x7B) Flag_PID=1;   //APP参数指令起始位
		if(Usart3_Receive==0x7D) Flag_PID=2;   //APP参数指令停止位

		 if(Flag_PID==1)  //采集数据
		 {
				Receive[i]=Usart3_Receive;
				i++;
		 }
		 if(Flag_PID==2)  //分析数据
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
						// GUA: 改成了对应指令的时长
						case 0x36:  job_dir[1][2]=Data; break; 						// 前进时长
						case 0x37:  job_dir[2][2]=Data; job_dir[3][2]=Data; break; 	// 转向时长
						case 0x38:  job_dir[0][2]=Data; break; 						// 停止时长
					}
				}
			    	Flag_PID=0;
					i=0;
					j=0;
					Data=0;
					memset(Receive, 0, sizeof(u8)*50);//数组清零
		 }

		HAL_UART_Receive_IT(&huart3,Usart3_Receive_buf,sizeof(Usart3_Receive_buf));//串口3回调函数执行完毕之后，需要再次开启接收中断等待下一次接收中断的发生
	}
}



