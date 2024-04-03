/*
 * key.h
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */

#ifndef INC_KEY_H_
#define INC_KEY_H_
#include "main.h"

#define KEY	 HAL_GPIO_ReadPin(User_key_GPIO_Port, User_key_Pin)
u8 click_N_Double (u8 time);
u8 click(void);
u8 Long_Press(void);


#endif /* INC_KEY_H_ */
