#include "main.h"                  

extern float Roll_Theoretical_Value;   
float pitch,roll,yaw;
float measure,calcu;   //测量值和理论值 
int velocity;          //速度测量值（编码器脉冲数，非真实速度）
int PWM;

void Exit_Init(void)
{
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO ,ENABLE); //RCC
	
	GPIO_InitTypeDef GPIO_InitStructure;											 //GPIO
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource8);//AFIO  中断引脚选择
	
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line=EXTI_Line8;    //外部中断通道
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;									//外部中断使能
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;				//外部中断模式（更新中断）
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Falling;		//外部触发模式（上升沿触发）
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);       //终端分组
	
	NVIC_InitTypeDef  NVIC_InitStruct;										
	NVIC_InitStruct.NVIC_IRQChannel=EXTI9_5_IRQn;				//中断通道
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE ;						//中断使能
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;	//优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;					//优先级
	NVIC_Init(&NVIC_InitStruct);
}

void EXTI9_5_IRQHandler(void)
{
  	if(MPU6050_DMP_Get_Data(&pitch,&roll,&yaw)==0)
    { 	
		measure = roll;                                    //测量值
		calcu = Roll_Theoretical_Value;                                  //理论值
			
        velocity = ( Encoder_L_Cnt() + Encoder_R_Cnt() )/2;
//	    PWM = vertical_PID_value(measure, calcu);//直立环测试	 
        PWM = vertical_PID_value(measure, calcu) + velocity_PID_value(velocity);  //PID计算：直立环+速度环
		if(PWM>100)PWM=100;
		if(PWM<-100)PWM=-100;
		
	    if(PWM>0)
		{
			Motor_Dir(1,1);  //左1   正转1 反转0
			Motor_Dir(0,1);  //右0   正转1 反转0
		}
		else if(PWM<0)
		{
			Motor_Dir(1,0);  //左1   正转1 反转0
			Motor_Dir(0,0);  //右0   正转1 反转0
		}
		PWM_Set_Psc_Duty(720,abs(PWM)+5);	
  	}
	EXTI_ClearITPendingBit(EXTI_Line8);
}

