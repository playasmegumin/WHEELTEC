#include "delay.h"

//=======================  以下是使用DWT重新编写HAL库延迟相关函数，释放SysTick计时器   ===========================//
//=======================  当SysTick被占用时，可以使用此方法来对HAL库中延迟函数重新配置 ===========================//
/*
	DWT使用步骤：
a.先使能DWT外设，由内核调试寄存器DEM_CR的位24控制，写1使能。   DEM_CR地址0xE000EDFC
b.使能CYCCNT寄存器之前，先清0。						 									CYCCNT地址0xE001004
c.使能CYCCNT寄存器，由DWT_CTRL的位0控制，写1使能。		 				DWT_CTRL地址0xE0001000
*/

//=== 重写HAL库延迟函数后，宏定义  HAL_MAX_DELAY 需要修改成下列式子 === //
//修改完成后需要注释掉下面的语句，防止冲突
//#define HAL_MAX_DELAY  4294967295/(HAL_RCC_GetSysClockFreq()/1000)

//DWT延迟微秒因子
uint32_t dwt_us;

//重写HAL_InitTick()，生成函数后此函数会被自动调用，用户不需要自行再次调用
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
	//1.使用DWT前必须使能DGB的系统跟踪（《Cortex-M3权威指南》）
	DEM_CR |= 1<<24;

	//2.使能CYCCNT计数器前必须先将其清零，CYCCNT计数器是32位的
	DWT_CYCCNT = (uint32_t)0u;

	//3.使能CYCCNT，开启计时
	DWT_CTRL |= 1<<0;

	//4.计算DWT微秒延迟函数的延迟因子
	dwt_us = HAL_RCC_GetSysClockFreq()/1000000;

	return HAL_OK;
}

//重写HAL_GetTick()
uint32_t HAL_GetTick(void)
{
	//把计数值除以内核频率的1000分之一倍，实现1毫秒返回1
	//以C8T6为例：因为计数到72是1微秒（1÷72MHz）
	//能返回的最大的毫秒值是 2^32 - 1 / 72000 = 59652  (取整数)
	return ((uint32_t)DWT_CYCCNT/(HAL_RCC_GetSysClockFreq()/1000));
}

//重写HAL_Delay()
//Delay 范围： 0 ~ ( 2^32-1 / （系统时钟÷1000））
//72MHz时钟： 0~59652
//84MHz时钟： 0~51130
//180MHz时钟：0~23860
//400Mhz时钟：0~10737
void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < __HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  //HAL_Delay()延迟函数重写后，需要考虑溢出的问题，原函数注释
//  while ((HAL_GetTick() - tickstart) < wait)
//  {
//  }

  wait += tickstart;   	   																 //计算所需要的计时时间
  if(wait>__HAL_MAX_DELAY) wait = wait - __HAL_MAX_DELAY;  //大于最大计数值则溢出,计算溢出部分

  //计数没有溢出，直接等待到延迟时间即可
   if(wait>tickstart)
  {
	while(HAL_GetTick()<wait);
  }
  //计数溢出
  else
  {
	while(HAL_GetTick()>wait); //未溢出部分计时
	while(HAL_GetTick()<wait); //溢出部分计时
  }
}

//返回32位计数器CYCCNT的值
uint32_t DWT_CNT_GET(void)
{
	return((uint32_t)DWT_CYCCNT);
}

//使用DWT提供微秒级延迟函数
void HAL_Delay_us(uint32_t us)
{
	uint32_t BeginTime,EndTime,delaytime;

	//获取当前时间戳
	BeginTime = DWT_CNT_GET();

	//计算需要延迟多少微秒
	delaytime = us*dwt_us;

	//起始+需要延迟的微秒 = 需要等待的时间
	EndTime = BeginTime+delaytime;

	//计数没有溢出，直接等待到延迟时间即可
	if(EndTime>BeginTime)
	{
		while(DWT_CNT_GET()<EndTime);
	}

	//计数溢出
	else
	{
		while(DWT_CNT_GET()>EndTime); //未溢出部分计时
		while(DWT_CNT_GET()<EndTime); //溢出部分计时
	}
}

////使用DWT提供微秒级延迟函数,方法2
//void HAL_Delay_us(uint32_t us)
//{
//	//用于保存上一次计数值和当前计数值
//	uint32_t last_count,count,WaitTime;
//
//	//计算需要延迟多少个1微秒
//	uint32_t delaytime = us*dwt_us;
//
//	last_count = DWT_CNT_GET(); //获取当前的计数值
//	WaitTime = 0;
//
//	//循环等待延迟时间
//	while(WaitTime<delaytime)
//	{
//		count = DWT_CNT_GET();
//		//计时未溢出
//		if(count>last_count) WaitTime = count-last_count;
//
//		//计时溢出，加上32位的偏差值
//		else				 WaitTime = count+0xffffffff - last_count;
//	}
//}


//======================= SysTick被释放，以下是对SysTick重新配置利用 ===========================//

u16 i_us;  //微秒因子
u16 i_ms;  //毫秒因子

//SysTick被释放后，使用SysTick编写延迟函数
void delay_init(void)
{
	SysTick->CTRL &= ~(1<<2); 				   //设定Systick时钟源，HCLK/8
	SysTick->CTRL &= ~(1<<1);				     //关闭由HAL库自带的SysTick中断，减少系统资源浪费
	i_us = HAL_RCC_GetSysClockFreq()/8000000;  //计算微秒因子 HCLK/晶振
	i_ms = i_us * 1000 ; 					   //计算毫秒因子
}

// i_us * us 的值不可超过 2^24 -1 = 16777215
// 以72MHz的F103C8T6为例，i_us = 9 , us = 0~1,864,135
void delay_us(u32 us)
{
//	u32 temp;
//	SysTick -> LOAD = i_us * us; //计算需要设定的自动重装值
//	SysTick -> VAL = 0 ;         //清空计数器
//	SysTick -> CTRL |= 1<<0 ;    //开启计时
//	do{
//		temp = SysTick -> CTRL; 		  			//读取CTRL寄存器的状态位，目的是获取第0位和最高位
//	}while((temp&0x01)&&!(temp&(1<<16))); //如果计数器被使能且计时时间未到
//	SysTick->CTRL &= ~(1<<0);  			  		//计时结束，关闭倒数
//	SysTick -> VAL = 0 ;       		      	//清空计数器
}

// i_ms * ms 的值不可超过 2^24 -1 = 16777215
// 以72MHz的F103C8T6为例，i_ms = 9000 , us = 0 ~ 1864
void delay_ms(u16 ms)
{
	u32 temp;
	SysTick -> LOAD = i_ms * ms; //计算需要设定的自动重装值
	SysTick -> VAL = 0 ;         //清空计数器
	SysTick -> CTRL |= 1<<0 ;    //开启计时
	do{
		temp = SysTick -> CTRL; 		  			//读取CTRL寄存器的状态位，目的是获取第0位和最高位
	}while((temp&0x01)&&!(temp&(1<<16))); //如果计数器在使能，且倒数未到，则循环
	SysTick->CTRL &= ~(1<<0);  			  		//计时结束，关闭倒数
	SysTick -> VAL = 0 ;       		      	//清空计数器
}
