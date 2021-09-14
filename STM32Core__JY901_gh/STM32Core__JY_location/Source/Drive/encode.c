
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "encode.h"

static void TIM4_Mode_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/*- ������������������ PB->6   PB->7 -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);                           

	/*- TIM4������ģʽ���� -*/
	TIM_DeInit(TIM4);
//	TIM_TimeBaseStructure.TIM_Period = 799; // 200*4
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD-1; 
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);              
                 
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising);	//���ñ�����ģʽ����Դ�ͼ���
	
	TIM_ICStructInit(&TIM_ICInitStructure);																																		//�����˲���
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
		
	TIM4->CNT = 0;

	TIM_Cmd(TIM4, ENABLE);   //����TIM4��ʱ��
}

void TIM4_Init(void)
{
  TIM4_Mode_Config();
}

static void TIM3_Mode_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/*- ������������������ PA->6   PA->7 -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);                           

	/*- TIM3������ģʽ���� -*/
	TIM_DeInit(TIM3);
//	TIM_TimeBaseStructure.TIM_Period = 799;  // 200*4
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD-1; 
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);              
                 
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising);	//���ñ�����ģʽ����Դ�ͼ���
	
	TIM_ICStructInit(&TIM_ICInitStructure);																																		//�����˲���
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
		
	TIM3->CNT = 0;

	TIM_Cmd(TIM3, ENABLE);   //����TIM3��ʱ��
}

void TIM3_Init(void)
{
  TIM3_Mode_Config();
}



//s16 Enc_GetCount1(void)  
//{  
//    static  u16   lastCount = 0;  
//    u16  curCount = TIM3->CNT; //��ȡ����ֵ
//    s32 dAngle = curCount - lastCount;  
//    if(dAngle >= MAX_COUNT)  
//    {  
//        dAngle -= ENCODER_TIM_PERIOD;  
//    }  
//    else if(dAngle < -MAX_COUNT)  
//    {  
//        dAngle += ENCODER_TIM_PERIOD;  
//    }  
//    lastCount = curCount;  
//    return (s16)dAngle;  
//} 

s16 Enc_GetCount1(void)  
{  
    static  u16   lastCount = 0;  
    u16  curCount = TIM3->CNT; //��ȡ����ֵ

    return (s16)lastCount;  
} 

//s16 Enc_GetCount2(void)  
//{  
//    static  u16   lastCount = 0;  
//    u16  curCount = TIM4->CNT; //��ȡ����ֵ
//    s32 dAngle = curCount - lastCount;  
//    if(dAngle >= MAX_COUNT)  
//    {  
//        dAngle -= ENCODER_TIM_PERIOD;  
//    }  
//    else if(dAngle < -MAX_COUNT)  
//    {  
//        dAngle += ENCODER_TIM_PERIOD;  
//    }  
//    lastCount = curCount;  
//    return (s16)dAngle;  
//} 

s16 Enc_GetCount2(void)  
{  
    static  u16   lastCount = 0;  
    u16  curCount = TIM4->CNT; //��ȡ����ֵ
    s32 dAngle = curCount - lastCount;  
    if(dAngle >= MAX_COUNT)  
    {  
        dAngle -= ENCODER_TIM_PERIOD;  
    }  
    else if(dAngle < -MAX_COUNT)  
    {  
        dAngle += ENCODER_TIM_PERIOD;  
    }  
    lastCount = curCount;  
    return (s16)dAngle;  
} 

