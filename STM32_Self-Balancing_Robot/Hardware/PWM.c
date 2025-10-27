#include "main.h"                  

void PWM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE );   //RCC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE );

	GPIO_InitTypeDef  GPIO_InitStruct;                     //GPIO
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);

	TIM_InternalClockConfig(TIM4);                         
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=100-1;           			    //ARR
	TIM_TimeBaseInitStruct.TIM_Prescaler=0;			     				    //PSC
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);

	TIM_OCInitTypeDef TIM_OCInitStruct; 
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;              //输出比较模式
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;      //极性（不反转）
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;  //输出使能
	TIM_OCInitStruct.TIM_Pulse=0;															//CCR
	TIM_OC1Init(TIM4,&TIM_OCInitStruct);
	TIM_OC2Init(TIM4,&TIM_OCInitStruct);

	TIM_Cmd(TIM4,ENABLE);
}

void PWM_Set_Psc_Duty(uint16_t Prescaler , uint16_t Duty) 
{
	TIM_PrescalerConfig(TIM4,Prescaler,TIM_PSCReloadMode_Update);                   // F= 72M/Psc/ARR(100-1)     PSC=0 F=720K(不分频)（差值1）
	TIM_SetCompare1(TIM4,Duty);    //右       //CCR    占空比=CCR/ARR(100);  
	TIM_SetCompare2(TIM4,Duty);    //左       //CCR    占空比=CCR/ARR(100);
}








