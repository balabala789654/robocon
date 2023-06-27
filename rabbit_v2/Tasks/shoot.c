#include "main.h"

float motor_pid_param[6] = {7.0, 1.0, 0.0, 5000, 200, 200};
extern RC_ctrl_t rc_ctrl;

int target_2006_speed_1=0;
int target_2006_speed_2=-0;
int target_2006_speed_3=2000;

long int target_angle_L=0;
long int target_angle_R=0;

int initial_angle_L=0;
int initial_angle_R=0;

char loop_loading_flag=0;
int shoot_speed=0;

int speed_set=400;
char fflag=1;
void shoot_task(void *pvParameters)
{
	
//	delay_ms(5000);
//	LED1=0;
//	PID_init(&rabbit.shoot.motor_2006[0].pid_param, PID_POSITION, motor_pid_param[0], motor_pid_param[1], motor_pid_param[2], motor_pid_param[3], motor_pid_param[4], motor_pid_param[5]);
//	PID_init(&rabbit.shoot.motor_2006[1].pid_param, PID_POSITION, motor_pid_param[0], motor_pid_param[1], motor_pid_param[2], motor_pid_param[3], motor_pid_param[4], motor_pid_param[5]);
//	PID_init(&rabbit.shoot.motor_2006[2].pid_param, PID_POSITION, motor_pid_param[0], motor_pid_param[1], motor_pid_param[2], motor_pid_param[3], motor_pid_param[4], motor_pid_param[5]);


	while(1)
	{
		
		
//		if(fflag==1)
//		{
//			initial_angle_L=rabbit.shoot.motor_2006[0].feedback.ecd;
//			initial_angle_R=rabbit.shoot.motor_2006[1].feedback.ecd;
//			target_angle_L=294876;
//			target_angle_R=-294876+initial_angle_R;
//			fflag=0;
//		}
//		if(rc_ctrl.rc.s[0]==2 && rc_ctrl.rc.s[1]==2)
//		{
//			motor_driver_can2(0x200, 0, 0, 0, 0);
//			TIM_SetCompare1(TIM5,1000);
//			TIM_SetCompare2(TIM5,1000);
//		}
//		else 
//		{
//			if(rc_ctrl.rc.s[0]==1)
//			{
//				shoot_speed = rc_ctrl.rc.ch[4]*speed_set/660;
//			}				
//			else if(rc_ctrl.rc.s[0]==3)
//			{
//				target_2006_speed_3 = rc_ctrl.rc.ch[3]*2000/660;
//				
//				if(rc_ctrl.rc.ch[1]!=0) loop_loading_flag=1;
//				else if(rc_ctrl.rc.ch[1]==0) loop_loading_flag=0;
//				
//			}
//			TIM_SetCompare1(TIM5,shoot_speed+1000);
//			TIM_SetCompare2(TIM5,shoot_speed+1000);
//			loop_loading(target_angle_L, target_angle_R, loop_loading_flag);

		}
		vTaskDelay(1);
	}
	
}

void loop_loading(int _angle_L, int _angle_R, char flag)
{	
	PID_calc(&rabbit.shoot.motor_2006[0].pid_param, rabbit.shoot.motor_2006[0].feedback.speed_rpm, target_2006_speed_1);
	PID_calc(&rabbit.shoot.motor_2006[1].pid_param, rabbit.shoot.motor_2006[1].feedback.speed_rpm, target_2006_speed_2);
	PID_calc(&rabbit.shoot.motor_2006[2].pid_param, rabbit.shoot.motor_2006[2].feedback.speed_rpm, target_2006_speed_3);
	
	motor_driver_can2(0x200, rabbit.shoot.motor_2006[0].pid_param.out, rabbit.shoot.motor_2006[1].pid_param.out, rabbit.shoot.motor_2006[2].pid_param.out, 0);
	
	if(rabbit.shoot.motor_2006[0].feedback.all_ecd >= target_angle_L)
	{
		
		target_2006_speed_1=0;
		rabbit.shoot.motor_2006[0].feedback.count=0;
		rabbit.shoot.motor_2006[0].feedback.all_ecd=0;
//		if(rabbit.shoot.motor_2006[0].feedback.ecd>=initial_angle_L)
//		{
//			target_2006_speed_1=0;
//			rabbit.shoot.motor_2006[0].feedback.count=0;
//		}
	}
	else 
	{
		if(flag) target_2006_speed_1=2000;
	}
	
	if(rabbit.shoot.motor_2006[1].feedback.all_ecd <= target_angle_R)
	{
		target_2006_speed_2=0;
		rabbit.shoot.motor_2006[1].feedback.count=0;
		rabbit.shoot.motor_2006[1].feedback.all_ecd=0;
		
//		if(rabbit.shoot.motor_2006[1].feedback.ecd<=initial_angle_R)
//		{
//			target_2006_speed_2=0;
//			rabbit.shoot.motor_2006[1].feedback.count=0;
//		}
	}
	else 
	{
		if(flag) target_2006_speed_2=-2000;
	}
}

