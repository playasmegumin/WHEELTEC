/*
 * LED.c
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */
#include "LED.h"

/**************************************************************************
Function: Led flashing
Input   : time��Flicker frequency
Output  : none
�������ܣ�LED��˸
��ڲ�������˸Ƶ��
����  ֵ����
**************************************************************************/
void Led_Flash(u16 time)
{

	 static int temp;
	 if(0==time) LED_ON;
	 else 	if(++temp==time)	ToggleLED,temp=0;
}


