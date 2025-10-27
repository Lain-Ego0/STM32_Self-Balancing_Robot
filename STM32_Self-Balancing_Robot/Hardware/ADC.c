#include "main.h"                  
//多通道（连续转化，扫描模式）+DMA转运（循环模式，ADC硬件触发）
uint16_t AD_Value[4];
void AD_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOA,ENABLE);   //RCC
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);    								 //72M/6=12MHZ
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;        //模拟输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,   1   ,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,   2   ,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2,   3   ,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_3,   4   ,ADC_SampleTime_55Cycles5);
										   	//ADC选择//ADC通道   //序列1 //采集时间（55.5个ADC周期）+12.5ADC周期转化时间
																						 //菜单  //实际转化= 1/（72/6）*（55.5+12.5）=5.6us
	
	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_Mode=ADC_Mode_Independent;	    			//ADC模式（独立ADC）
	ADC_InitStruct.ADC_DataAlign=ADC_DataAlign_Right;				//对齐方式（右对齐）
	ADC_InitStruct.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;	//触发方式（不是外部触发 即软件触发）
	ADC_InitStruct.ADC_ContinuousConvMode=ENABLE;					//是否开启连续触发（否 即单次触发）
	ADC_InitStruct.ADC_ScanConvMode=ENABLE;                         //是否开启扫描模式（非扫描模式）
	ADC_InitStruct.ADC_NbrOfChannel=4;	 							//扫描模式的菜单（序列数（菜单的前四个））
	ADC_Init(ADC1,&ADC_InitStruct);
	
	DMA_InitTypeDef DMA_InitStruct;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&ADC1->DR;                 //外设地址
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;     //数据长度（字节）
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;  		       //地址不自增
	DMA_InitStruct.DMA_MemoryBaseAddr=(uint32_t)AD_Value;					   //目的地址
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;             //数据长度（字节）
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;                         //地址不自增
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralSRC;							   //方向（外设——>寄存器）
	DMA_InitStruct.DMA_BufferSize=4;										   //转运数量
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;								   //转运数量自动重装
	DMA_InitStruct.DMA_M2M=DMA_M2M_Disable;									   //硬件触发
	DMA_InitStruct.DMA_Priority=DMA_Priority_Medium;						   //优先等级（中等）
	DMA_Init(DMA1_Channel1,&DMA_InitStruct);
	//关于DMA的原理可复习https://blog.csdn.net/weixin_62179882/article/details/130002589
	
	ADC_Cmd(ADC1,ENABLE);											//ADC使能
	ADC_DMACmd(ADC1,ENABLE);										//ADC_DMA使能
	ADC_SoftwareStartConvCmd(ADC1,ENABLE );                         //ADC软件触发
	DMA_Cmd(DMA1_Channel1,ENABLE);                                  //DMA使能

	ADC_ResetCalibration(ADC1);	                                    //复位校准
	while( ADC_GetResetCalibrationStatus(ADC1)==SET);               //获取复位校准状态
	ADC_StartCalibration(ADC1);                                     //开始校准
	while( ADC_GetCalibrationStatus(ADC1)==SET);                    //获取开始校准状态	                                                             
}



