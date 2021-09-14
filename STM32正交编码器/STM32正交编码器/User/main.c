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

/*- 外部调用 -*/
extern u16	Parameter;



u16	Parameter = 200;																	//输入编码器线数
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
#define N     200                                     //编码器线数
#define CIRCUM     119.695                                //全向轮轮子的周长，单位为毫米

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




//CopeSerialData为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	LED_REVERSE();					//接收到数据，LED灯闪烁一下
	USB_TxWrite(&ucData,1);			//向USB-HID端口转发收到的串口数据，可直接用接上位机看到模块输出的数据。
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
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
		ucRxCnt=0;//清空缓存区
	}
}

void CopeSerial1Data(unsigned char ucData)
{	
	UART2_Put_Char(ucData);//转发串口1收到的数据给串口2（JY模块）
}
void KEY_Init(void) //IO初始化
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
	//初始化KEY0-->GPIOA.13,KEY1-->GPIOA.15  上拉输入
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//使能PORTA,PORTE时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;//PE2~4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOE2,3,4

	

}
void EXTIX_Init(void)
{
 
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

    KEY_Init();	 //	按键端口初始化

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟

	
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource12);
	  EXTI_InitStructure.EXTI_Line=EXTI_Line12;	//KEY2
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
	
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource13);
	  EXTI_InitStructure.EXTI_Line=EXTI_Line13;	//KEY2
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
	
	
	
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//使能按键WK_UP所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//子优先级3
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel =EXTI15_10_IRQn;			//使能按键KEY2所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);
	
	
  //GPIOE.2 中断线以及中断初始化配置   下降沿触发
/*  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource2);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line12;	//KEY2
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

 //GPIOE.3	  中断线以及中断初始化配置 下降沿触发 //KEY1
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource3);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line3;
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

 //GPIOE.4	  中断线以及中断初始化配置  下降沿触发	//KEY0
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource4);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line4;
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器


  //GPIOA.0	  中断线以及中断初始化配置 上升沿触发 PA0  WK_UP
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0); 

 	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器


  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//使能按键WK_UP所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//子优先级3
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//使能按键KEY2所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);


  	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			//使能按键KEY1所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//使能按键KEY0所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//子优先级0 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
   */
}


void EXTI15_10_IRQHandler (void)
{
	
	
	
	if(EXTI_GetITStatus(EXTI_Line12)!=0)
	if(KEY0==1)	 //按键KEY0
	{
		
		if(KEY2==1)
     xx++;		
		
		if(KEY2==0)
		xx--;
		
			EXTI_ClearITPendingBit(EXTI_Line12);  //清除LINE4上的中断标志位  
	}		 
if(EXTI_GetITStatus(EXTI_Line13)!=0)
		if(KEY1==1)	 //按键KEY0
	{
		
		if(KEY3==1)
     yy++;		
		
		if(KEY3==0)
		yy--;
		
		
		
			EXTI_ClearITPendingBit(EXTI_Line13);  //清除LINE4上的中断标志位  
	}		 
	
	
	
}
 

int main(void)

{  
	char str[100];
	unsigned char len,i;
		
	USB_Config();		//配置USB-HID
	SysTick_init(72,10);//设置时钟频率
	Initial_UART1(9600);//接PC的串口
	Initial_UART2(9600);//接JY-901模块的串口	
	//NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	//EXTIX_Init();
	TIM4_Init();		
	LED_ON();
	delay_ms(1000);//等等我的陀螺仪初始化完成
	while(1)
	{		
	 	
		
		
		delay_ms(100);//等我的陀螺仪初始化完成
		//sprintf(str,tx);
	Old_Count = Count;																//获取先前的计数值
		Count = TIM4->CNT;												/////				//获取编码器当前数值
		if(Old_Count != Count)														//如果先前的计数值与当前计数值不相等，说明编码器已转动打印当前数值
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
 txx= (txx/N)*CIRCUM*100000;    //如果不使用UART1，我们只需把*100000去掉就好
 //之后加算法txx= txx*cos((double)(stcAngle.Angle[2]/32768)*3.1415926);
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
		//输出时间
	//	sprintf(str,"Time:20%d-%d-%d %d:%d:%.3f\r\n",stcTime.ucYear,stcTime.ucMonth,stcTime.ucDay,stcTime.ucHour,stcTime.ucMinute,(float)stcTime.ucSecond+(float)stcTime.usMiliSecond/1000);
	//		UART1_Put_String(str);		
		//	delay_ms(10);
		//输出加速度
	//	sprintf(str,"Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);
		//	UART1_Put_String(str);
	//		delay_ms(10);
		//输出角速度
	//	sprintf(str,"Gyro:%.3f %.3f %.3f\r\n",(float)stcGyro.w[0]/32768*2000,(float)stcGyro.w[1]/32768*2000,(float)stcGyro.w[2]/32768*2000);
		//	UART1_Put_String(str);
		//	delay_ms(10);
		//输出角度
		//sprintf(str,"Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);
		sprintf(str,"Angle: %.3f\r\n",(float)stcAngle.Angle[2]/32768*180);

		UART1_Put_String(str);
			delay_ms(10);
		//输出磁场
	//	sprintf(str,"Mag:%d %d %d\r\n",stcMag.h[0],stcMag.h[1],stcMag.h[2]);
	//		UART1_Put_String(str);		
		//	delay_ms(10);
		//输出气压、高度
	//	sprintf(str,"Pressure:%ld Height%.2f\r\n",stcPress.lPressure,(float)stcPress.lAltitude/100);
		//	UART1_Put_String(str); 
	//		delay_ms(10);
		//输出端口状态
	//	sprintf(str,"DStatus:%d %d %d %d\r\n",stcDStatus.sDStatus[0],stcDStatus.sDStatus[1],stcDStatus.sDStatus[2],stcDStatus.sDStatus[3]);
		//	UART1_Put_String(str);
	//		delay_ms(10);
		//输出经纬度
	//	sprintf(str,"Longitude:%ldDeg%.5fm Lattitude:%ldDeg%.5fm\r\n",stcLonLat.lLon/10000000,(double)(stcLonLat.lLon % 10000000)/1e5,stcLonLat.lLat/10000000,(double)(stcLonLat.lLat % 10000000)/1e5);
		//	UART1_Put_String(str);
	//		delay_ms(10);
		//输出地速
	//	sprintf(str,"GPSHeight:%.1fm GPSYaw:%.1fDeg GPSV:%.3fkm/h\r\n",(float)stcGPSV.sGPSHeight/10,(float)stcGPSV.sGPSYaw/10,(float)stcGPSV.lGPSVelocity/1000);
	//		UART1_Put_String(str);
		//	delay_ms(10);
		//输出四元素
	//	sprintf(str,"Four elements:%.5f %.5f %.5f %.5f\r\n\r\n",(float)stcQ.q[0]/32768,(float)stcQ.q[1]/32768,(float)stcQ.q[2]/32768,(float)stcQ.q[3]/32768);
		//	UART1_Put_String(str);
		
	
		*/
		
		    delay_ms(10);//等待传输完成
	}//主循环
}













