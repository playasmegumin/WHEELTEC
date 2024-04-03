/*
 * DataScope_DP.h
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */

#ifndef INC_DATASCOPE_DP_H_
#define INC_DATASCOPE_DP_H_

extern unsigned char DataScope_OutPut_Buffer[42];	   //待发送帧数据缓存区


void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // 写通道数据至 待发送帧数据缓存区

unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // 发送帧数据生成函数


#endif /* INC_DATASCOPE_DP_H_ */
