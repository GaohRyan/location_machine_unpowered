#ifndef __ENCODE_H__
#define __ENCODE_H__

#include "stm32f10x.h"

#define MAX_COUNT          10000//10000Ҳ����1ms�ڲ��ᳬ��10000������
#define ENCODER_TIM_PERIOD 0xffff//���Ԥ��Ƶֵ��65536-1 


void TIM4_Init(void);
void TIM3_Init(void);

s16 Enc_GetCount1(void);  
s16 Enc_GetCount2(void);  

#endif
