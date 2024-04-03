#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "main.h"

//使用到的寄存器地址
#define DWT_CTRL        *(uint32_t*)0xE0001000
#define DWT_CYCCNT      *(uint32_t*)0xE0001004
#define DEM_CR          *(uint32_t*)0xE000EDFC

//HAL_Delay()微秒函数溢出数值
//CYCCNT寄存器/主频的千分之一倍，返回的是最大能容纳的微秒数
// 4294967295/(HAL_RCC_GetSysClockFreq()/1000)
#define __HAL_MAX_DELAY  4294967295/(HAL_RCC_GetSysClockFreq()/1000)

void delay_init(void);
void delay_us(u32 us);
void delay_ms(u16 ms);
void HAL_Delay_us(uint32_t us);
uint32_t DWT_CNT_GET(void);

#endif

