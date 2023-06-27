//#include "chassis.h"
#include "main.h"

double empty_address_chassis = 0;
float chassis_pid_params[4][6]={
									{7.0f, 1.0f, 0.0f, 8000, 200, 200},
									{7.0f, 1.0f, 0.0f, 8000, 200, 200},
									{7.0f, 1.0f, 0.0f, 8000, 200, 200},
									{7.0f, 1.0f, 0.0f, 8000, 200, 200},
								};



void chassis_task(void *pvParameters)
{
	char flag=0;
	chassis_init();
	
	set_chassis_max_speed(&rabbit.chassis, 4000, 2000);//speed, spin
	

	while(1)
	{
		if(rabbit.rabbit_state)
		{
			if(rabbit.chassis.chassis_switch==ON&&flag==0)
			{
				chassis_address_transform(&rabbit.chassis, (double* )&rc_ctrl.rc.ch[2], (double* )&rc_ctrl.rc.ch[3], (double* )&rc_ctrl.rc.ch[0]);
				flag++;
			}
			else if(rabbit.chassis.chassis_switch!=ON&&flag==1)
			{		
				chassis_address_transform(&rabbit.chassis, (double* )&empty_address_chassis, (double* )&empty_address_chassis, (double* )&empty_address_chassis);
				flag--;
			}
		
			remote_contorl_speed(&rc_ctrl,&rabbit.chassis);
			motor_speed_compute(&rabbit.chassis);
		
			motor_driver_can1(	chassis_motor_StdId,
								rabbit.chassis.chassis_motor[0].pid_param.out,
								rabbit.chassis.chassis_motor[1].pid_param.out,
								rabbit.chassis.chassis_motor[2].pid_param.out,
								rabbit.chassis.chassis_motor[3].pid_param.out	);


		}
		else
		{
			flag=0;
			chassis_address_transform(&rabbit.chassis, (double* )&empty_address_chassis, (double* )&empty_address_chassis, (double* )&empty_address_chassis);
			motor_driver_can1(chassis_motor_StdId, 0, 0, 0, 0);			
		}
		vTaskDelay(1);
	}
}

/**
	*@brief 麦轮解算 
	*@param 遥控器结构体地址
	*@param	rabbit底盘结构体地址
	*@retval none
**/
void remote_contorl_speed(RC_ctrl_t* RC, CHASSIS* _chassis)
{
	int x,y=0;
	int vx,vy;

	y = *_chassis->chassis_targer_vy * _chassis->max_chassis_speed/660;
	x = *_chassis->chassis_targer_vx * _chassis->max_chassis_speed/660;
	
	vy = y*cos(PI/4)-x*cos(PI/4);
	vx = y*sin(PI/4)+x*sin(PI/4);
	
	if(RC->rc.s[0]==2&&RC->rc.s[1]==2)
	{
		_chassis->chassis_motor[0].target_rpm=0;
		_chassis->chassis_motor[1].target_rpm=0;
		_chassis->chassis_motor[2].target_rpm=0;
		_chassis->chassis_motor[3].target_rpm=0;
	}
	else
	{
		if(RC->rc.ch[2]==0&&RC->rc.ch[3]==0&&RC->rc.ch[0]==0&&RC->rc.ch[1]==0)
		{
			_chassis->chassis_motor[0].target_rpm=0;
			_chassis->chassis_motor[1].target_rpm=0;
			_chassis->chassis_motor[2].target_rpm=0;
			_chassis->chassis_motor[3].target_rpm=0;
		}

		else if(RC->rc.ch[2]!=0||RC->rc.ch[3]!=0||RC->rc.ch[0]!=0||RC->rc.ch[1]!=0)
		{			
			_chassis->chassis_motor[0].target_rpm=-vy + *_chassis->chassos_targer_vz * _chassis->max_chassis_spin/660;
			_chassis->chassis_motor[2].target_rpm=vy + *_chassis->chassos_targer_vz * _chassis->max_chassis_spin/660;
			
			_chassis->chassis_motor[1].target_rpm=vx + *_chassis->chassos_targer_vz * _chassis->max_chassis_spin/660;
			_chassis->chassis_motor[3].target_rpm=-vx + *_chassis->chassos_targer_vz *_chassis->max_chassis_spin/660;
		}
	}

}

/**
	*@brief 没有注释
	*@param 没有注释
	*@param	没有注释
	*@retval none
**/
void motor_speed_compute(CHASSIS* _chassis)
{
	PID_calc(&_chassis->chassis_motor[0].pid_param,_chassis->chassis_motor[0].feedback.speed_rpm,_chassis->chassis_motor[0].target_rpm);
	PID_calc(&_chassis->chassis_motor[1].pid_param,_chassis->chassis_motor[1].feedback.speed_rpm,_chassis->chassis_motor[1].target_rpm);
	PID_calc(&_chassis->chassis_motor[2].pid_param,_chassis->chassis_motor[2].feedback.speed_rpm,_chassis->chassis_motor[2].target_rpm);
	PID_calc(&_chassis->chassis_motor[3].pid_param,_chassis->chassis_motor[3].feedback.speed_rpm,_chassis->chassis_motor[3].target_rpm);
}

/**
	*@brief 没有注释
	*@param 没有注释
	*@param	没有注释
	*@retval none
**/
void chassis_address_transform(CHASSIS* _chassis, double* _remote_massge_vx, double* _remote_massge_vy, double* _remote_massge_vz)
{
	_chassis->chassis_targer_vx=_remote_massge_vx;
	_chassis->chassis_targer_vy=_remote_massge_vy;
	_chassis->chassos_targer_vz=_remote_massge_vz;
}   


/**
	*@brief 没有注释
	*@param 没有注释
	*@param	没有注释
	*@retval none
**/
void set_chassis_max_speed(CHASSIS* _chassis, int _speed, int _spin)
{
	_chassis->max_chassis_speed=_speed;
	_chassis->max_chassis_spin=_spin;
	
	
}

void chassis_pid_init(CHASSIS* chassis, float params[4][6])
{
	for(int i=0;i<4;i++)
	{
		PID_init(&chassis->chassis_motor[i].pid_param,PID_POSITION,params[i][0],params[i][1],params[i][2],params[i][3],params[i][4],params[i][5]);
	}
}

void chassis_init(void)
{
	chassis_pid_init(&rabbit.chassis, chassis_pid_params);
}


