/*
 * ReadDistance.h
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */

#ifndef INC_READDISTANCE_H_
#define INC_READDISTANCE_H_
#include "main.h"
extern u16 TIM3CH3_CAPTURE_STA,TIM3CH3_CAPTURE_VAL;

#define Trigger_RESET	 HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET)
#define Trigger_SET	 	 HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET)

void Read_Distane(void);

#endif /* INC_READDISTANCE_H_ */
