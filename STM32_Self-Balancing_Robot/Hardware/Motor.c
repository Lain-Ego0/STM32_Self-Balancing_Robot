#include "main.h" 

void Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
 	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	PWM_Init();
}

void Motor_Dir(uint8_t Motor,uint8_t Dir)
{
	if(Motor==1) //×óÂÖ  A2 A3
	{	 
		if(Dir==0)
		{
			GPIO_SetBits(GPIOA,GPIO_Pin_2);
			GPIO_ResetBits(GPIOA,GPIO_Pin_3);	
		}
		else 
		{
			GPIO_ResetBits(GPIOA,GPIO_Pin_2);
			GPIO_SetBits(GPIOA,GPIO_Pin_3);	
		}		
	}
	else if(Motor==0)
	{
		if(Dir==1)
		{
			GPIO_SetBits(GPIOA,GPIO_Pin_4);
			GPIO_ResetBits(GPIOA,GPIO_Pin_5);	
		}
		else 
		{
			GPIO_ResetBits(GPIOA,GPIO_Pin_4);
			GPIO_SetBits(GPIOA,GPIO_Pin_5);	
		}	
	}
}

