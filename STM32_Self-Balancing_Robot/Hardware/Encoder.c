#include "main.h"                  // Device header

/**
  * @brief  编码器初始化
  * @param  无
  * @retval 无
  */
void Encoder_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3,ENABLE );
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;      //分频系数（不分频）
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;  //向上计数
	TIM_TimeBaseInitStruct.TIM_Period=65536-1;									//ARR
	TIM_TimeBaseInitStruct.TIM_Prescaler=1-1;										//PSC
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0x00;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);

	TIM_ICInitTypeDef TIM_ICInitStruct;
	TIM_ICStructInit(&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel=TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICFilter=0xF;  										 //滤波
	TIM_ICInitStruct.TIM_ICPolarity=TIM_ICPolarity_Rising;   //非上升沿触发（而是极性不反转）									 
	TIM_ICInit(TIM2,&TIM_ICInitStruct);
	TIM_ICInit(TIM3,&TIM_ICInitStruct);

	TIM_ICInitStruct.TIM_Channel=TIM_Channel_2;
	TIM_ICInitStruct.TIM_ICFilter=0xF;
	TIM_ICInitStruct.TIM_ICPolarity=TIM_ICPolarity_Rising; 
	TIM_ICInit(TIM2,&TIM_ICInitStruct);
	TIM_ICInit(TIM3,&TIM_ICInitStruct);
																											//极性不反转         //极性不反转
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);

	TIM_Cmd(TIM2,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
}

/**
  * @brief  编码器获取计数值
  * @param  无
  * @retval uint16_t
  */
int16_t Encoder_L_Cnt(void)
{	
	int16_t Temp;
	Temp=TIM_GetCounter(TIM3);     //Cnt计数值	
	TIM_SetCounter(TIM3,0);
	return Temp;
}
int16_t Encoder_R_Cnt(void)
{	
	int16_t Temp;
	Temp=TIM_GetCounter(TIM2);     //Cnt计数值	
	//Temp=TIM_GetCapture1(TIM3);  //易错点  CCR
	TIM_SetCounter(TIM2,0);
	//TIM_SetCapture1(TIM3);       //易错点   CCR
	return Temp;
	//TIM3会寄，原因不明，编码器模式无法重复复用？
}

//由于平衡小车暂不考虑转弯等复杂功能，目前先保证双边编码器计数平均处理。
//int16_t Encoder_L,Encoder_R,Encoder_velocity;
//void Encoder(void)
//{
//	Encoder_L=Encoder_L_Cnt();
//	Encoder_R=Encoder_R_Cnt();
//	Encoder_velocity=(Encoder_L+Encoder_R)/2;
//}
