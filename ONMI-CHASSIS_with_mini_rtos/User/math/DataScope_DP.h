#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H

#include "stm32h7xx.h"

//#define MINIBALANCE       //若要使用VOFA+串口上位机，则注释此宏
#define MAX_SEND_NUM 10     //MINIBALANCE最多10个通道，VOFA+可以超过10个通道

void DataWave(UART_HandleTypeDef* huart);
void Float2Byte(float *target,unsigned char *buf,unsigned char beg);

#endif

