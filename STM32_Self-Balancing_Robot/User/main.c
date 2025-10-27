#include "main.h"                  

extern float pitch,roll,yaw;  //欧拉角测量值
float Roll_Theoretical_Value=-3.1;          
//roll理论值（小车平衡时的角度）
//由机械结构决定问题，消差，调整至左侧最后的差数参数在小车平衡时为0即可
extern int velocity;          //速度测量值（编码器脉冲数，非真实速度）
extern float velocity_sum;    //速度积分
int motor_flag;               //电机使能标志：1使能  0失能

/*MPU6050的偏航角是不正常的，即使静止不动也会跳动。
这个纯粹是硬件问题，就算算法怎么牛逼也解决不了。
只能外加磁力计解决，也可以直接使用MPU9250，这个是自带磁力计的。
使用时注意不要和电机靠太近，磁场很容易受到干扰。*/

/*以下为pid调参
pid调参稍微总结了一下，实际上就几句话。
疯狂调大直立环P直到出现高幅度低频震荡。
疯狂调大直立环D直到出现低幅度高频震荡。
疯狂调大速度环P（200倍比例），另一参数D除以200即可。
出现过大距离或者无法维持平衡之后再回调直立环P。
重复以上过程*/

//直立环PD参数:
//float Kp=0,Ki=0,Kd=0;//初始化无PID  
//float Kp=10,Ki=0,Kd=3;  
//float Kp=4,Ki=0,Kd=3.2;  
//float Kp=30,Ki=0,Kd=150;//出现低幅度高频震荡，参数*0.6即可
float Kp=18,Ki=0,Kd=90; //成功
//直立环调完后x0.6

//速度环PI参数: 
//float VKp=0,VKi=0;//初始化无PID   
//float VKp=2,VKi=0.01;   
float VKp=10,VKi=0.05; //大概成功（5分钟内可自稳，估计是陀螺仪的0飘问题）

extern int PWM; //电机pwm输出参数  

int main(void)
{
	delay_init();
	OLED_Init();
	MPU6050_DMP_Init();
	Motor_Init();
	Encoder_Init();
	Exit_Init();
	
	while(1)
	{		
		OLED_ShowSignedNum(1,1,pitch,3);
		OLED_ShowSignedNum(2,1,roll,3);
		OLED_ShowSignedNum(3,1,yaw,3);//三个欧拉角
		OLED_ShowSignedNum(1,10,Encoder_L_Cnt(),6);
		OLED_ShowSignedNum(2,10,Encoder_R_Cnt(),6);
		OLED_ShowSignedNum(3,10,velocity,6);
		OLED_ShowNum(4,1,abs(PWM),5);//Cnt计数值
	}
}

