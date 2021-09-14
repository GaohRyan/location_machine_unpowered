#ifndef __ENCODE_H__
#define __ENCODE_H__

#include "stm32f10x.h"

#define MAX_COUNT          10000//10000也就是1ms内不会超过10000个脉冲
#define ENCODER_TIM_PERIOD 0xffff//最大预分频值是65536-1 


void TIM4_Init(void);
void TIM3_Init(void);

s16 Enc_GetCount1(void);  
s16 Enc_GetCount2(void);  

#endif
