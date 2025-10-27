#include "main.h"                  

extern float pitch,roll,yaw;  //ŷ���ǲ���ֵ
float Roll_Theoretical_Value=-3.1;          
//roll����ֵ��С��ƽ��ʱ�ĽǶȣ�
//�ɻ�е�ṹ�������⣬���������������Ĳ���������С��ƽ��ʱΪ0����
extern int velocity;          //�ٶȲ���ֵ��������������������ʵ�ٶȣ�
extern float velocity_sum;    //�ٶȻ���
int motor_flag;               //���ʹ�ܱ�־��1ʹ��  0ʧ��

/*MPU6050��ƫ�����ǲ������ģ���ʹ��ֹ����Ҳ��������
���������Ӳ�����⣬�����㷨��ôţ��Ҳ������ˡ�
ֻ����Ӵ����ƽ����Ҳ����ֱ��ʹ��MPU9250��������Դ������Ƶġ�
ʹ��ʱע�ⲻҪ�͵����̫�����ų��������ܵ����š�*/

/*����Ϊpid����
pid������΢�ܽ���һ�£�ʵ���Ͼͼ��仰��
������ֱ����Pֱ�����ָ߷��ȵ�Ƶ�𵴡�
������ֱ����Dֱ�����ֵͷ��ȸ�Ƶ�𵴡�
�������ٶȻ�P��200������������һ����D����200���ɡ�
���ֹ����������޷�ά��ƽ��֮���ٻص�ֱ����P��
�ظ����Ϲ���*/

//ֱ����PD����:
//float Kp=0,Ki=0,Kd=0;//��ʼ����PID  
//float Kp=10,Ki=0,Kd=3;  
//float Kp=4,Ki=0,Kd=3.2;  
//float Kp=30,Ki=0,Kd=150;//���ֵͷ��ȸ�Ƶ�𵴣�����*0.6����
float Kp=18,Ki=0,Kd=90; //�ɹ�
//ֱ���������x0.6

//�ٶȻ�PI����: 
//float VKp=0,VKi=0;//��ʼ����PID   
//float VKp=2,VKi=0.01;   
float VKp=10,VKi=0.05; //��ųɹ���5�����ڿ����ȣ������������ǵ�0Ʈ���⣩

extern int PWM; //���pwm�������  

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
		OLED_ShowSignedNum(3,1,yaw,3);//����ŷ����
		OLED_ShowSignedNum(1,10,Encoder_L_Cnt(),6);
		OLED_ShowSignedNum(2,10,Encoder_R_Cnt(),6);
		OLED_ShowSignedNum(3,10,velocity,6);
		OLED_ShowNum(4,1,abs(PWM),5);//Cnt����ֵ
	}
}

