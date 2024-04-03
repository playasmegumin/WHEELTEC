/*
 * LED.c
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */
#include "LED.h"

/**************************************************************************
Function: Led flashing
Input   : time：Flicker frequency
Output  : none
函数功能：LED闪烁
入口参数：闪烁频率
返回  值：无
**************************************************************************/
void Led_Flash(u16 time)
{

	 static int temp;
	 if(0==time) LED_ON;
	 else 	if(++temp==time)	ToggleLED,temp=0;
}


