/*
 * DataScope_DP.h
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */

#ifndef INC_DATASCOPE_DP_H_
#define INC_DATASCOPE_DP_H_

extern unsigned char DataScope_OutPut_Buffer[42];	   //������֡���ݻ�����


void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // дͨ�������� ������֡���ݻ�����

unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // ����֡�������ɺ���


#endif /* INC_DATASCOPE_DP_H_ */
