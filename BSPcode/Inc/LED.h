/*
 * LED.h
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */

#ifndef INC_LED_H_
#define INC_LED_H_
#include "main.h"

#define LED_ON 		 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_OFF 	 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define ToggleLED	 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)

void Led_Flash(u16 time);

#endif /* INC_LED_H_ */
