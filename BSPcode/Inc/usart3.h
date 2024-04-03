/*
 * usart3.h
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */

#ifndef INC_USART3_H_
#define INC_USART3_H_
#include "main.h"

extern u8 Usart3_Receive_buf[1];          //串口3接收中断数据存放的缓冲区
extern u8 Usart3_Receive;                 //从串口3读取的数据

#endif /* INC_USART3_H_ */
