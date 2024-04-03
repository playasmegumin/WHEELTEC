/*
 * filter.h
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_
#include "main.h"
float Kalman_Filter_x(float Accel,float Gyro);
float Complementary_Filter_x(float angle_m, float gyro_m);
float Kalman_Filter_y(float Accel,float Gyro);
float Complementary_Filter_y(float angle_m, float gyro_m);


#endif /* INC_FILTER_H_ */
