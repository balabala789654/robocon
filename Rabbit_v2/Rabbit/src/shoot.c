#include "rabbit.h"
#include "status.h"

static float shoot_speed_pid_param[6]={10.0f, 1.0f, 0.0f, 5000.0f, 500.0f, 500.0f};
static float shoot_angle_pid_param[6]={0.3f, 1.0f, 0.0f, 2000.0f, 500.0f, 500.0f};
static char flag2=0;
static char flag3=1;

void shoot_task(void *pvParameters)
{
	delay_ms(5000);
	LED2=0;
	shoot_init();
	while(1)
	{		
		if(status_change_callback(&rc_ctrl)!=Reset_system)
		{
			shoot_lifting(shoot_lifting_set_callback);
			shoot_fly_motor_speed_set(&rabbit.shoot, status_mode_change_callback, status_change_callback, status_remote_callback);
			shoot_enter_motor_loop(&rabbit.shoot, status_remote_callback, status_mode_change_callback);
			shoot_motor_pid_cal(&rabbit.shoot, motor_speed_pid_cal_callback, motor_angle_pid_cal_callback);
			shoot_fly_motor_drive(rabbit.shoot.shoot_rpm);
		}
		else 
		{
			shoot_read_initial_ecd(&rabbit.shoot);
			shoot_fly_motor_drive(1000);
			shoot_motor_close(&rabbit.shoot, motor_close_callback);
		}
		
		shoot_motor_drive(&rabbit.shoot, 0x1ff, 0x200, motor_driver_can1, motor_driver_can2);

		vTaskDelay(1);
	}
}


void shoot_init(void)
{
	shoot_fly_motor_drive(1100);
	shoot_read_initial_ecd(&rabbit.shoot);
	shoot_set_enter_motor_speeds(&rabbit.shoot, 2000);
	shoot_motor_pid_init(&rabbit.shoot, pid_speed_init_callback, pid_angle_init_callback);
}



void shoot_set_enter_motor_speeds(SHOOT *_shoot, int _set_speed1)
{
	_shoot->motor_speed_1 = _set_speed1;
	
	_shoot->motor[0].target_rpm = 0;
	_shoot->motor[1].target_rpm = -0;
	_shoot->motor[2].target_rpm = 0;
	
	_shoot->motor[0].target_angle = 294876;
	_shoot->motor[1].target_angle = -294876+_shoot->motor[1].initial_ecd;
	_shoot->motor[2].target_angle = 294876;
}


/**
	*@brief 
	*@param 
	*@param	remote
	*@param	mode
	*@retval 
**/
void shoot_enter_motor_loop(SHOOT *_shoot, REMOTE (*callback1)(RC_ctrl_t *_rc) , char (*callback2)(RC_ctrl_t *_rc))
{			
	
	if(_shoot->motor[0].feedback.all_ecd >= _shoot->motor[0].target_angle )
	{
		_shoot->motor[0].target_rpm=0;
		_shoot->motor[0].feedback.count=0;
		_shoot->motor[0].feedback.all_ecd=0;		
	}
	else if((*callback2)(&rc_ctrl)==mode_1 && (*callback1)(&rc_ctrl).ch[1]==0)
	{
		_shoot->motor[0].target_rpm=_shoot->motor_speed_1;
	}
	
	
	if(_shoot->motor[1].feedback.all_ecd <= _shoot->motor[1].target_angle )
	{
		_shoot->motor[1].target_rpm=0;
		_shoot->motor[1].feedback.count=0;
		_shoot->motor[1].feedback.all_ecd=0;
		
	}
	else if((*callback2)(&rc_ctrl)==mode_1 && (*callback1)(&rc_ctrl).ch[1]==0)
	{
		_shoot->motor[1].target_rpm=-_shoot->motor_speed_1;
	}
	
	
	if(_shoot->motor[2].feedback.all_ecd >= _shoot->motor[2].target_angle)
	{
		_shoot->motor[2].target_rpm=0;
		_shoot->motor[2].feedback.count=0;
		_shoot->motor[2].feedback.all_ecd=0;
	}
	else if((*callback2)(&rc_ctrl)==mode_1 && (*callback1)(&rc_ctrl).ch[1]==0)
	{
		_shoot->motor[2].target_rpm=_shoot->motor_speed_1;
	}
}

/**
	*@brief 
	*@param 
	*@param mode
	*@param status
	*@param remote
	*@retval 
**/
void shoot_fly_motor_speed_set(SHOOT *_shoot, char (*callback1)(RC_ctrl_t *_rc), char (*callback2)(RC_ctrl_t *_rc), REMOTE (*callback3)(RC_ctrl_t *_rc))
{
	static char last_status;
	if(last_status!=(*callback2)(&rc_ctrl) && (*callback1)(&rc_ctrl)==mode_2)
	{
		if((*callback2)(&rc_ctrl)==Status_shoot_high)
			_shoot->shoot_rpm = shoot_rpm_set_high;
		
		else if((*callback2)(&rc_ctrl)==Status_shoot_mid)
			_shoot->shoot_rpm = shoot_rpm_set_mid;
		
		else if((*callback2)(&rc_ctrl)==Status_shoot_low)
			_shoot->shoot_rpm = shoot_rpm_set_low;
		
		else if((*callback2)(&rc_ctrl)==Status_shoot_close)
			_shoot->shoot_rpm = 1000;
		
	}
	last_status = (*callback2)(&rc_ctrl);
	
	if((*callback1)(&rc_ctrl)==mode_2)
	{
		if((*callback3)(&rc_ctrl).ch[1]>0 && flag2==1)
		{
			_shoot->shoot_rpm+=5;
			flag2=0;
		}
		else if((*callback3)(&rc_ctrl).ch[1]<0 && flag2==1)
		{
			_shoot->shoot_rpm-=5;
			flag2=0;
		}
		else if((*callback3)(&rc_ctrl).ch[1]==0 && flag2==0)
		{
			flag2=1;
		}
	}
		
	
}
	
void shoot_motor_pid_init(SHOOT *_shoot, void (*callback1)(MOTOR *_motor, float _arr[6]), void (*callback2)(MOTOR *_motor, float _arr[6]))
{
	for(int i=0; i<3; i++)
	{
		(*callback1)(&_shoot->motor[i], shoot_speed_pid_param);
		(*callback2)(&_shoot->motor[i], shoot_angle_pid_param);
	}
		

}

void shoot_motor_pid_cal(SHOOT *_shoot, void (*callback1)(MOTOR *_motor), void (*callback2)(MOTOR *_motor))
{
	(*callback1)(&_shoot->motor[0]);
	(*callback1)(&_shoot->motor[1]);//Ë«»·
	(*callback1)(&_shoot->motor[2]);
}

void shoot_motor_close(SHOOT *_shoot, void (*callback)(MOTOR *_motor))
{
	(*callback)(&_shoot->motor[0]);
	(*callback)(&_shoot->motor[1]);
	(*callback)(&_shoot->motor[2]);
}




/**
	*@brief 
	*@param 
	*@param	id1
	*@param	id2
	*@param	can1
	*@param	can2
	*@retval 
**/
void shoot_motor_drive(SHOOT *_shoot, uint16_t std_id1, uint16_t std_id2, void (*callback1)(uint16_t stid, int i1,int i2,int i3,int i4), void (*callback2)(uint16_t stid, int i1,int i2,int i3,int i4))
{
	(*callback1)(std_id1, 		_shoot->motor[2].pid_param_speed.out,
								0,
								0,
								0									 );
	
	(*callback2)(std_id2, 		_shoot->motor[0].pid_param_speed.out,
								_shoot->motor[1].pid_param_speed.out,
								0,
								0									 );
		
	
}

void shoot_read_initial_ecd(SHOOT *_shoot)
{
	_shoot->motor[0].initial_ecd = _shoot->motor[0].feedback.ecd;
	_shoot->motor[0].feedback.count=0;
	_shoot->motor[0].feedback.all_ecd=0;
	
	_shoot->motor[1].initial_ecd = _shoot->motor[1].feedback.ecd;
	_shoot->motor[1].feedback.count=0;
	_shoot->motor[1].feedback.all_ecd=0;

	
}
	
void shoot_fly_motor_drive(int _speed)
{
	TIM_SetCompare1(TIM5, _speed);
	TIM_SetCompare2(TIM5, _speed);
}


/**
	*@brief 
	*@param 
	*@param	mode
	*@param	remote
	*@retval 
**/
char shoot_lifting_set_callback(char (*callback1)(RC_ctrl_t *_rc), REMOTE (*callback2)(RC_ctrl_t *_rc))
{
	if((*callback1)(&rc_ctrl)==mode_1 && (*callback2)(&rc_ctrl).ch[1]>=650 && flag3)
	{
		flag3=0;
		return ON;
	}
	else if((*callback1)(&rc_ctrl)==mode_1 && (*callback2)(&rc_ctrl).ch[1]<=-650 && flag3!=1)
	{
		flag3=1;
		return OFF;
	}
	else return 2;
}


void shoot_lifting(char (*callback)(char (*callback1)(RC_ctrl_t *_rc), REMOTE (*callback2)(RC_ctrl_t *_rc)))
{
	if((*callback)(status_mode_change_callback, status_remote_callback)!=2)
		Lifting=(*callback)(status_mode_change_callback, status_remote_callback);
}



