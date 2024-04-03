/*
 * adc.h
 *
 *  Created on: Jun 11, 2022
 *      Author: WHEELTEC
 */

#ifndef INC_BSP_ADC_H_
#define INC_BSP_ADC_H_

#include "main.h"

#define Battery_Ch 6


#define _1CYCLE_5     ADC_SAMPLETIME_1CYCLE_5
#define _7CYCLES_5    ADC_SAMPLETIME_7CYCLES_5
#define _13CYCLES_5   ADC_SAMPLETIME_13CYCLES_5
#define _28CYCLES_5   ADC_SAMPLETIME_28CYCLES_5
#define _41CYCLES_5   ADC_SAMPLETIME_41CYCLES_5
#define _55CYCLES_5   ADC_SAMPLETIME_55CYCLES_5
#define _71CYCLES_5   ADC_SAMPLETIME_71CYCLES_5
#define _239CYCLES_5  ADC_SAMPLETIME_239CYCLES_5

uint16_t Get_Adc(uint8_t ch,uint32_t SampleTime);
uint32_t Get_Adc_Channel(uint8_t ch);
uint16_t Get_Adc_Average(uint8_t ch,uint32_t SampleTime,uint8_t times);
int Get_battery_volt(void);

#endif /* INC_BSP_ADC_H_ */
