#include "rabbit.h"
#include "status.h"

static float shoot_pid_param[6]={10.0f, 1.0f, 0.0f, 5000.0f, 500.0f, 500.0f};
static char flag1=1;
static char flag2=0;
static char flag3=1;

void shoot_task(void *pvParameters)
{
	delay_ms(5000);
	LED2=0;
	shoot_init();
	while(1)
	{
		if(flag1 && (rabbit.status==Status_shoot_high || rabbit.status==Status_shoot_mid || rabbit.status==Status_shoot_low))
		{
			shoot_address_transform(&rabbit.shoot, &rc_ctrl.rc.ch[3], &rc_ctrl.rc.ch[1]);
			flag1=0;
		}
		else if(flag1!=1 && (rabbit.status!=Status_shoot_high && rabbit.status!=Status_shoot_mid && rabbit.status!=Status_shoot_low))
		{
			flag1=1;
		}
		
		if(rabbit.status!=Reset_system)
		{
			shoot_lifting_set(&rabbit.shoot, &rabbit, &rc_ctrl);
			shoot_lifting(&rabbit.shoot);
			shoot_fly_motor_speed_set(&rabbit.shoot, &rabbit, &rc_ctrl);
			shoot_enter_motor_loop(&rabbit.shoot, &rabbit, &rc_ctrl);
			shoot_motor_pid_cal(&rabbit.shoot);
			shoot_motor_drive(&rabbit.shoot, 0x200, 0x1ff);
			shoot_fly_motor_drive(rabbit.shoot.shoot_rpm);
		}
		else 
		{
			shoot_fly_motor_drive(1000);
			shoot_motor_close(&rabbit.shoot);
			shoot_motor_drive(&rabbit.shoot, 0x200, 0x1ff);
		}
		vTaskDelay(1);
	}
}


void shoot_init(void)
{
	//shoot_lifting_init(&rabbit.shoot);
	shoot_fly_motor_drive(1000);
	shoot_read_initial_ecd(&rabbit.shoot);
	shoot_set_rpm(&rabbit.shoot, 1000, 1200, 1400, 1000);
	shoot_set_enter_motor_speeds(&rabbit.shoot, 2000, -2000);
	shoot_motor_pid_init(&rabbit.shoot);
}

void shoot_set_rpm(SHOOT *_shoot, int _speed1, int _speed2, int _speed3, int _speed4)
{
	_shoot->shoot_rpm_set_low = _speed1;
	_shoot->shoot_rpm_set_mid = _speed2;
	_shoot->shoot_rpm_set_high = _speed3;
	_shoot->shoot_rpm = _speed4;
}

void shoot_set_enter_motor_speeds(SHOOT *_shoot, int _set_speed1, int _set_speed2)
{
	_shoot->motor_speed_1 = _set_speed1;
	
	_shoot->motor[0].target_rpm = 0;
	_shoot->motor[1].target_rpm = -0;
	_shoot->motor[2].target_rpm = 0;
	
	_shoot->motor[0].target_angle = 294876;
	_shoot->motor[1].target_angle = -294876+_shoot->motor[1].initial_ecd-150;
	_shoot->motor[2].target_angle = 294876;
}

void shoot_enter_motor_loop(SHOOT *_shoot, RABBIT *_rabbit, RC_ctrl_t *_rc)
{		
	if(_shoot->motor[0].feedback.all_ecd >= _shoot->motor[0].target_angle )
	{
		_shoot->motor[0].target_rpm=0;
		_shoot->motor[0].feedback.count=0;
		_shoot->motor[0].feedback.all_ecd=0;
	}
	else if(_rabbit->mode==mode_1 && _rc->rc.ch[1]==0)
	{
		_shoot->motor[0].target_rpm=_shoot->motor_speed_1;
	}
	
	if(_shoot->motor[1].feedback.all_ecd <= _shoot->motor[1].target_angle)
	{
		_shoot->motor[1].target_rpm=0;
		_shoot->motor[1].feedback.count=0;
		_shoot->motor[1].feedback.all_ecd=0;
	}
	else if(_rabbit->mode==mode_1 && _rc->rc.ch[1]==0)
	{
		_shoot->motor[1].target_rpm=-_shoot->motor_speed_1;
	}
	
	if(_shoot->motor[2].feedback.all_ecd >= _shoot->motor[2].target_angle)
	{
		_shoot->motor[2].target_rpm=0;
		_shoot->motor[2].feedback.count=0;
		_shoot->motor[2].feedback.all_ecd=0;
	}
	else if(_rabbit->mode==mode_1 && _rc->rc.ch[2]==0)
	{
		_shoot->motor[2].target_rpm=_shoot->motor_speed_1;
	}

}


void shoot_fly_motor_speed_set(SHOOT *_shoot, RABBIT *_rabbit, RC_ctrl_t *_rc)
{
	static char last_status;
	if(last_status!=_rabbit->status && _rabbit->mode==mode_2)
	{
		if(_rabbit->status==Status_shoot_high)
			_shoot->shoot_rpm = _shoot->shoot_rpm_set_high;
		else if(_rabbit->status==Status_shoot_mid)
			_shoot->shoot_rpm = _shoot->shoot_rpm_set_mid;
		else if(_rabbit->status==Status_shoot_low)
			_shoot->shoot_rpm = _shoot->shoot_rpm_set_low;
		else if(_rabbit->status==Status_shoot_close)
			_shoot->shoot_rpm = 1000;
		
	}
	last_status = _rabbit->status;
	
	if(_rabbit->mode==mode_2)
	{
		if(_rc->rc.ch[1]>0 && flag2==1)
		{
			_shoot->shoot_rpm+=5;
			flag2=0;
		}
		else if(_rc->rc.ch[1]<0 && flag2==1)
		{
			_shoot->shoot_rpm-=5;
			flag2=0;
		}
		else if(_rc->rc.ch[1]==0 && flag2==0)
		{
			flag2=1;
		}
	}
		
	
}
	
void shoot_motor_pid_init(SHOOT *_shoot)
{
	for(int i=0; i<3; i++)
	{
		PID_init(&_shoot->motor[i].pid_param_speed, PID_POSITION, shoot_pid_param[0], shoot_pid_param[1], shoot_pid_param[2], shoot_pid_param[3], shoot_pid_param[4], shoot_pid_param[5]);
	}

}

void shoot_motor_pid_cal(SHOOT *_shoot)
{
	motor_speed_pid_cal(&_shoot->motor[0]);
	motor_speed_pid_cal(&_shoot->motor[1]);
	motor_speed_pid_cal(&_shoot->motor[2]);
}

void shoot_motor_close(SHOOT *_shoot)
{
	motor_close(&_shoot->motor[0]);
	motor_close(&_shoot->motor[1]);
	motor_close(&_shoot->motor[2]);
}


void shoot_motor_drive(SHOOT *_shoot, uint16_t std_id, uint16_t std_id_2)
{
	motor_driver_can2(std_id, 	_shoot->motor[0].pid_param_speed.out,
								_shoot->motor[1].pid_param_speed.out,
								_shoot->motor[2].pid_param_speed.out,
								0									 );
	
	motor_driver_can1(std_id_2, _shoot->motor[2].pid_param_speed.out,
								0,
								0,
								0									 );
	
	
}

void shoot_address_transform(SHOOT *_shoot, __packed double *_arr1, __packed double *_arr2)
{
	_shoot->motor_speed_2 = _arr1;
	_shoot->motor_speed_3 = _arr2;
}
void shoot_read_initial_ecd(SHOOT *_shoot)
{
	_shoot->motor[0].initial_ecd = _shoot->motor[0].feedback.ecd;
	_shoot->motor[1].initial_ecd = _shoot->motor[1].feedback.ecd;
}
	
void shoot_fly_motor_drive(int _speed)
{
	TIM_SetCompare1(TIM5, _speed);
	TIM_SetCompare2(TIM5, _speed);
}

void shoot_reset_status(SHOOT *_shoot)
{
	shoot_fly_motor_drive(1000);
	
}

void shoot_lifting_set(SHOOT *_shoot, RABBIT *_rabbit, RC_ctrl_t *_rc)
{
	if(_rabbit->mode==mode_1 && _rc->rc.ch[1]>=650 && flag3)
	{
		_shoot->lifting = ON;
		flag3=0;
	}
	if(_rabbit->mode==mode_1 && _rc->rc.ch[1]<=-650 && flag3!=1)
	{
		_shoot->lifting = OFF;
		flag3=1;
	}
}

void shoot_lifting(SHOOT *_shoot)
{
	Lifting=_shoot->lifting;
}

void shoot_lifting_init(SHOOT *_shoot)
{
	_shoot->lifting = OFF;
	Lifting=_shoot->lifting;
}



