#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"	
#include "lcd.h"
#include "encode.h"
#include <string.h>
#include <stdio.h>
#include "Main.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "UART1.h"
#include "UART2.h"
#include "delay.h"
#include "IOI2C.h"
#include "hw_config.h"
#include "JY901.h"
#include "DIO.h"
#include "math.h"

/*- �ⲿ���� -*/
extern u16	Parameter;



u16	Parameter = 200;																	//�������������
u16 Count,Old_Count;



















struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       stcQ;




#define KEY0  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)//
#define KEY1  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)//
#define KEY2  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)//
#define KEY3  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)//
#define N     200                                     //����������
#define CIRCUM     119.695                                //ȫ�������ӵ��ܳ�����λΪ����

u8  tx[]={"        "};


long int xx=0;

long int yy=0;


double txx=0;

long int tyy=0;
double ax;
double ay;
long int A =0;
long int B =0;
long int C =0;
long int D =0;
long int E =0;
long int F =0;
long int G =0;
long int H =0;
long int I =0;
long int J =0;




//CopeSerialDataΪ����2�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ�����������
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	LED_REVERSE();					//���յ����ݣ�LED����˸һ��
	USB_TxWrite(&ucData,1);			//��USB-HID�˿�ת���յ��Ĵ������ݣ���ֱ���ý���λ������ģ����������ݡ�
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
			case 0x59:	memcpy(&stcQ,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;//��ջ�����
	}
}

void CopeSerial1Data(unsigned char ucData)
{	
	UART2_Put_Char(ucData);//ת������1�յ������ݸ�����2��JYģ�飩
}
void KEY_Init(void) //IO��ʼ��
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
	//��ʼ��KEY0-->GPIOA.13,KEY1-->GPIOA.15  ��������
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//ʹ��PORTA,PORTEʱ��

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;//PE2~4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOE2,3,4

	

}
void EXTIX_Init(void)
{
 
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

    KEY_Init();	 //	�����˿ڳ�ʼ��

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ܸ��ù���ʱ��

	
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource12);
	  EXTI_InitStructure.EXTI_Line=EXTI_Line12;	//KEY2
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource13);
	  EXTI_InitStructure.EXTI_Line=EXTI_Line13;	//KEY2
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	
	
	
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//ʹ�ܰ���WK_UP���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//�����ȼ�3
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel =EXTI15_10_IRQn;			//ʹ�ܰ���KEY2���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//�����ȼ�2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);
	
	
  //GPIOE.2 �ж����Լ��жϳ�ʼ������   �½��ش���
/*  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource2);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line12;	//KEY2
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

 //GPIOE.3	  �ж����Լ��жϳ�ʼ������ �½��ش��� //KEY1
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource3);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line3;
  	EXTI_Init(&EXTI_InitStructure);	  	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

 //GPIOE.4	  �ж����Լ��жϳ�ʼ������  �½��ش���	//KEY0
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource4);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line4;
  	EXTI_Init(&EXTI_InitStructure);	  	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���


  //GPIOA.0	  �ж����Լ��жϳ�ʼ������ �����ش��� PA0  WK_UP
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0); 

 	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���


  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//ʹ�ܰ���WK_UP���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//�����ȼ�3
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//ʹ�ܰ���KEY2���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//�����ȼ�2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);


  	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			//ʹ�ܰ���KEY1���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//�����ȼ�1 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//ʹ�ܰ���KEY0���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//�����ȼ�0 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
   */
}


void EXTI15_10_IRQHandler (void)
{
	
	
	
	if(EXTI_GetITStatus(EXTI_Line12)!=0)
	if(KEY0==1)	 //����KEY0
	{
		
		if(KEY2==1)
     xx++;		
		
		if(KEY2==0)
		xx--;
		
			EXTI_ClearITPendingBit(EXTI_Line12);  //���LINE4�ϵ��жϱ�־λ  
	}		 
if(EXTI_GetITStatus(EXTI_Line13)!=0)
		if(KEY1==1)	 //����KEY0
	{
		
		if(KEY3==1)
     yy++;		
		
		if(KEY3==0)
		yy--;
		
		
		
			EXTI_ClearITPendingBit(EXTI_Line13);  //���LINE4�ϵ��жϱ�־λ  
	}		 
	
	
	
}
 

int main(void)

{  
	char str[100];
	unsigned char len,i;
		
	USB_Config();		//����USB-HID
	SysTick_init(72,10);//����ʱ��Ƶ��
	Initial_UART1(9600);//��PC�Ĵ���
	Initial_UART2(9600);//��JY-901ģ��Ĵ���	
	//NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	//EXTIX_Init();
	TIM4_Init();		
	LED_ON();
	delay_ms(1000);//�ȵ��ҵ������ǳ�ʼ�����
	while(1)
	{		
	 	
		
		
		delay_ms(100);//���ҵ������ǳ�ʼ�����
		//sprintf(str,tx);
	Old_Count = Count;																//��ȡ��ǰ�ļ���ֵ
		Count = TIM4->CNT;												/////				//��ȡ��������ǰ��ֵ
		if(Old_Count != Count)														//�����ǰ�ļ���ֵ�뵱ǰ����ֵ����ȣ�˵����������ת����ӡ��ǰ��ֵ
		{
		UART1_Put_Char('x');
		UART1_Put_Char(':');
			 
		   UART1_Put_Char(Count/10000%10+48);
       UART1_Put_Char(Count/1000%10+48);
	  	 UART1_Put_Char(Count/100%10+48);
	  	 UART1_Put_Char(Count/10%10+48);
	   	 UART1_Put_Char(Count%10+48);
			
		}
		
		
		
		
	/*	
		UART1_Put_Char('x');
		UART1_Put_Char(':');
		
     if(xx<0)
		 {txx=-xx;
		  UART1_Put_Char('-');
		 }
		 
else
{txx=xx;	

UART1_Put_Char('+');
}	 
 txx= (txx/N)*CIRCUM*100000;    //�����ʹ��UART1������ֻ���*100000ȥ���ͺ�
 //֮����㷨txx= txx*cos((double)(stcAngle.Angle[2]/32768)*3.1415926);
  A = (long int)(txx)/10000000000%10+48;
  B = (long int)(txx)/1000000000%10+48;
  C = (long int)(txx)/100000000%10+48;
  D = (long int)(txx)/10000000%10+48;
  E = (long int)(txx)/1000000%10+48;
  F = (long int)(txx)/100000%10+48;
  G = (long int)(txx)/10000%10+48;
 // H = (long int)(txx)/100%10+48;
 // I = (long int)(txx)/10%10+48;
 // J = (long int)(txx)%10+48;
     UART1_Put_Char(A);
     UART1_Put_Char(B);
		 UART1_Put_Char(C);
		 UART1_Put_Char(D);
		 UART1_Put_Char(E);
     UART1_Put_Char(F);
     UART1_Put_Char(46);
     
     UART1_Put_Char(G);
 //    UART1_Put_Char(H);
 //    UART1_Put_Char(I);
 //    UART1_Put_Char(J);
     UART1_Put_Char(109);
		 UART1_Put_Char(109);
		 
		 UART1_Put_Char(0x0d);
		 UART1_Put_Char(0x0a);
		
		
		
		UART1_Put_Char('y');
		UART1_Put_Char(':');
		
		
		if(yy<0)
		 {tyy=-yy;
		  UART1_Put_Char('-');
		 }
		 
     else
    {tyy=yy;	
		UART1_Put_Char('+');
    }	 
		
		
		
		
     UART1_Put_Char(tyy/10000%10+48);
     UART1_Put_Char(tyy/1000%10+48);
		 UART1_Put_Char(tyy/100%10+48);
		 UART1_Put_Char(tyy/10%10+48);
		 UART1_Put_Char(tyy%10+48);
		 UART1_Put_Char(0x0d);
		 UART1_Put_Char(0x0a);
		
		//printf(tx);
		

			delay_ms(500);
		//���ʱ��
	//	sprintf(str,"Time:20%d-%d-%d %d:%d:%.3f\r\n",stcTime.ucYear,stcTime.ucMonth,stcTime.ucDay,stcTime.ucHour,stcTime.ucMinute,(float)stcTime.ucSecond+(float)stcTime.usMiliSecond/1000);
	//		UART1_Put_String(str);		
		//	delay_ms(10);
		//������ٶ�
	//	sprintf(str,"Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);
		//	UART1_Put_String(str);
	//		delay_ms(10);
		//������ٶ�
	//	sprintf(str,"Gyro:%.3f %.3f %.3f\r\n",(float)stcGyro.w[0]/32768*2000,(float)stcGyro.w[1]/32768*2000,(float)stcGyro.w[2]/32768*2000);
		//	UART1_Put_String(str);
		//	delay_ms(10);
		//����Ƕ�
		//sprintf(str,"Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);
		sprintf(str,"Angle: %.3f\r\n",(float)stcAngle.Angle[2]/32768*180);

		UART1_Put_String(str);
			delay_ms(10);
		//����ų�
	//	sprintf(str,"Mag:%d %d %d\r\n",stcMag.h[0],stcMag.h[1],stcMag.h[2]);
	//		UART1_Put_String(str);		
		//	delay_ms(10);
		//�����ѹ���߶�
	//	sprintf(str,"Pressure:%ld Height%.2f\r\n",stcPress.lPressure,(float)stcPress.lAltitude/100);
		//	UART1_Put_String(str); 
	//		delay_ms(10);
		//����˿�״̬
	//	sprintf(str,"DStatus:%d %d %d %d\r\n",stcDStatus.sDStatus[0],stcDStatus.sDStatus[1],stcDStatus.sDStatus[2],stcDStatus.sDStatus[3]);
		//	UART1_Put_String(str);
	//		delay_ms(10);
		//�����γ��
	//	sprintf(str,"Longitude:%ldDeg%.5fm Lattitude:%ldDeg%.5fm\r\n",stcLonLat.lLon/10000000,(double)(stcLonLat.lLon % 10000000)/1e5,stcLonLat.lLat/10000000,(double)(stcLonLat.lLat % 10000000)/1e5);
		//	UART1_Put_String(str);
	//		delay_ms(10);
		//�������
	//	sprintf(str,"GPSHeight:%.1fm GPSYaw:%.1fDeg GPSV:%.3fkm/h\r\n",(float)stcGPSV.sGPSHeight/10,(float)stcGPSV.sGPSYaw/10,(float)stcGPSV.lGPSVelocity/1000);
	//		UART1_Put_String(str);
		//	delay_ms(10);
		//�����Ԫ��
	//	sprintf(str,"Four elements:%.5f %.5f %.5f %.5f\r\n\r\n",(float)stcQ.q[0]/32768,(float)stcQ.q[1]/32768,(float)stcQ.q[2]/32768,(float)stcQ.q[3]/32768);
		//	UART1_Put_String(str);
		
	
		*/
		
		    delay_ms(10);//�ȴ��������
	}//��ѭ��
}













