#include "main.h"
static double empty_shoot_friction=0;
static double empty_shoot_pitch=0;
static double empty_shoot_enter=0;

float shoot_enter_motor_pid_params[2][6]={
										{7.0, 1.0, 0.0, 5000, 200, 200},//speed
										{0.1, 0.1, 0.1, 1000, 200, 200} //angle
																			};
float shoot_pitch_motor_pid_params[2][6]={
										{15.0, 1.0, 0.0, 30000, 200, 200},//speed
										{0.1, 0.1, 0.1, 1000, 200, 200} //angle
																			};

																			
/**
	*@brief 
	*@param 
	*@param	
	*@retval 
**/
void shoot_init(void)
{
	enter_motor_init(&rabbit.shoot, shoot_enter_motor_pid_params);
	shoot_pitch_motor_init(&rabbit.shoot, shoot_pitch_motor_pid_params);
	
}	
void shoot_task(void *pvParameters)
{
	char flag=0;
	set_shoot_max_speed(&rabbit.shoot, 500, 1000, 1000);//_speed, _enter_speed, _pitch_speed
	while(1)
	{
		if(rabbit.rabbit_state)
		{
			if(rabbit.shoot.shoot_switch==ON&&flag==0)
			{
				shoot_address_transform(&rabbit.shoot, (double* )&rc_ctrl.rc.ch[4], (double* )&rc_ctrl.rc.ch[1], (double* )&rc_ctrl.rc.ch[3]);
				flag++;
			}
			else if(rabbit.shoot.shoot_switch!=ON&&flag==1)
			{
				empty_shoot_friction = *rabbit.shoot.address_shoot_friction_rpm;
				
				shoot_address_transform(&rabbit.shoot, &empty_shoot_friction, &empty_shoot_pitch, &empty_shoot_enter);
				flag--;
			}
			
			friction_start(&rabbit.shoot, rabbit.shoot.address_shoot_friction_rpm);
			//enter_start(&rabbit.shoot, rabbit.shoot.address_shoot_motor_enter);
			//shoot_pitch_start(&rabbit.shoot, rabbit.shoot.address_shoot_motor_pitch);
			
			
		}
		else
		{
			TIM_SetCompare1(TIM5,0);
			TIM_SetCompare2(TIM5,0);
		}
		vTaskDelay(1);
	}
	
}

/**
	*@brief 
	*@param 
	*@param	
	*@retval 
**/
void set_shoot_max_speed(SHOOT* _shoot, int _speed, int _enter_speed, int _pitch_speed)
{
	_shoot->max_friction_speed=_speed;
	_shoot->max_enter_speed=_enter_speed;
	_shoot->max_pitch_speed=_pitch_speed;
}


/**
	*@brief 
	*@param 
	*@param	
	*@retval 
**/
void friction_start(SHOOT* _shoot, double* _speed)
{
	_shoot->shoot_friction_rpm = *_speed*_shoot->max_friction_speed/660;
	_shoot->shoot_friction_rpm += 1000;
	
	TIM_SetCompare1(TIM5,_shoot->shoot_friction_rpm);
	TIM_SetCompare2(TIM5,_shoot->shoot_friction_rpm);
}


/**
	*@brief 
	*@param 
	*@param	
	*@retval 
**/
void enter_start(SHOOT* _shoot, double* remote_ch_1)
{
	_shoot->enter_motor.target_rpm = *remote_ch_1*_shoot->max_enter_speed/660;
	
	PID_calc(	&rabbit.shoot.enter_motor.pid_param, 
				rabbit.shoot.enter_motor.feedback.speed_rpm, 
				rabbit.shoot.enter_motor.target_rpm
																		);	

}


/**
	*@brief 
	*@param 
	*@param	
	*@retval 
**/
void shoot_pitch_start(SHOOT* _shoot, double* remote_ch_3)
{
	_shoot->shoot_pitch_motor.target_rpm = *remote_ch_3*_shoot->max_pitch_speed/660;
	
	PID_calc(	&rabbit.shoot.shoot_pitch_motor.pid_param, 
				rabbit.shoot.shoot_pitch_motor.feedback.speed_rpm, 
				rabbit.shoot.shoot_pitch_motor.target_rpm
																		);	

}


/**
	*@brief µØÖ·´«µÝ
	*@param 
	*@param	
	*@retval 
**/
void shoot_address_transform(SHOOT* _shoot, double* remote_ch_4, double* remote_ch_1, double* remote_ch_3)
{
	_shoot->address_shoot_friction_rpm = remote_ch_4;
	_shoot->address_shoot_motor_pitch = remote_ch_3;
	_shoot->address_shoot_motor_enter = remote_ch_1;
}
	



void enter_motor_init(SHOOT* _shoot, float params[2][6])
{
	PID_init(&_shoot->enter_motor.pid_param,PID_POSITION,params[0][0],params[0][1],params[0][2],params[0][3],params[0][4],params[0][5]);
	PID_init(&_shoot->enter_motor.angle_pid_param,PID_POSITION,params[1][0],params[1][1],params[1][2],params[1][3],params[1][4],params[1][5]);
}



void shoot_pitch_motor_init(SHOOT* _shoot, float params[2][6])
{
	PID_init(&_shoot->shoot_pitch_motor.pid_param,PID_POSITION,params[0][0],params[0][1],params[0][2],params[0][3],params[0][4],params[0][5]);
	PID_init(&_shoot->shoot_pitch_motor.angle_pid_param,PID_POSITION,params[1][0],params[1][1],params[1][2],params[1][3],params[1][4],params[1][5]);
}


