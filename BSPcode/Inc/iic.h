/*
 * iic.h
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */

#ifndef INC_IIC_H_
#define INC_IIC_H_
#include "main.h"

//IO方向设置
#define SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=8<<4;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=3<<4;}

//IO操作函数寄存器版本，提高效率
//#define IIC_SCL_SET  	GPIOB->BSRR |= 1<<8
//#define IIC_SCL_RESET	GPIOB->BSRR |= 1<<24
//#define IIC_SDA_SET  	GPIOB->BSRR |= 1<<9
//#define IIC_SDA_RESET	GPIOB->BSRR |= 1<<25
//#define READ_SDA  	((GPIOB->IDR)&(1<<9))

//IO操作函数
#define IIC_SCL_SET  	HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET)
#define IIC_SCL_RESET	HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET)
#define IIC_SDA_SET  	HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_SET)
#define IIC_SDA_RESET	HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_RESET)
#define READ_SDA  		HAL_GPIO_ReadPin(IIC_SDA_GPIO_Port, IIC_SDA_Pin)

//IIC所有操作函数
void IIC_Init(void);           //初始化IIC的IO口
int IIC_Start(void);					 //发送IIC开始信号
void IIC_Stop(void);	  			 //发送IIC停止信号
void IIC_Send_Byte(u8 txd);		 //IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
int IIC_Wait_Ack(void); 			 //IIC等待ACK信号
void IIC_Ack(void);						 //IIC发送ACK信号
void IIC_NAck(void);					 //IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif /* INC_IIC_H_ */
