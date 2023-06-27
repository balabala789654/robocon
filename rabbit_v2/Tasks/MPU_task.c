#include "main.h"
/*
	a = Range * G * ACC / 32768 ĳ����ļ��ٶ�  ����G�ǵ����������ٶȣ�ACC�Ǹ���ļ��ٶ�ԭʼ���ݣ�Range�������ü��ٶ�����ʱд��Ĳ���������
	g = Range * Gyro / 32768 	ĳ����Ľ��ٶ� 	Gyro�������Ľ��ٶ�ԭʼ���ݣ�Range�������ý��ٶ�����ʱд��Ĳ���������
*/

float old_yaw, new_yaw;
float pitch=0;
float yaw=0;
float roll=0;

short accx,accy,accz; 	//������ļ��ٶ�ԭʼֵ
float ax,ay,az;		//������ļ����ļ��ٶ�ֵ

short gyrox,gyroy,gyroz;
int a;

void MPU_task(void *pvParameters)
{
	
	while(1)
	{
		
		a = mpu_dmp_get_data(&pitch,&roll,&new_yaw);
		MPU_Get_Accelerometer(&accx, &accy, &accz);
		
		float Difference=new_yaw-old_yaw;
		ax = 9.8*accx*2/32768.0f;
		ay = 9.8*accy*2/32768.0f;
		az = 9.8*accz*2/32768.0f;
		
//		_vx += ax * (double)0.00001;
//		_vy += ay * (double)0.00001;
		
		/* ����mpu_dmp_get_data()������ȡֵ��ΧΪ(-180,180)������̬����ͻ�䣬���������д���ʹ��̬���ۼӼ��ɽ�� */
		if(Difference<-300) Difference = 360+new_yaw-old_yaw;
		else if(Difference>300) Difference = (new_yaw-360)-old_yaw;
		
		yaw += Difference;
		
		old_yaw=new_yaw;
		
		current_yaw = yaw*PI/180.0f;//���ڼ���yaw�仯����
		
		vTaskDelay(1);

	}
}

