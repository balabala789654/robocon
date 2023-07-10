#include "rabbit.h"


static char  pick_up_motor_state=MID;

static float pick_up_speed_pid_param[6]={20.0f, 1.0f, 0.0f, 40000.0f, 500.0f, 500.0f};
static float pick_up_angle_pid_param[6]={0.3f, 1.0f, 0.0f, 2000.0f, 500.0f, 500.0f};

static char flag=1;
static char p3_RU_last;

void pick_up_task(void *pvParameters)
{
	pick_up_init(&rabbit.pick_up);
	while(1)
	{
		pick_up_clamping(&rabbit.pick_up, status_p3_key_callback, &p3_data);
		pick_up_motor_speed_set(&rabbit.pick_up, status_p3_key_callback, 300, 100, status_p3_ch_callback);
		pick_up_motor_pid_cal(&rabbit.pick_up, motor_speed_pid_cal_callback);
		pick_up_motor_drive(&rabbit.pick_up, 0x1ff, motor_driver_can1, &rabbit.shoot);
		
		vTaskDelay(1);
	}
}

void pick_up_init(PICK_UP *_pick_up)
{
	_pick_up->clamping=pick_up_Clamping;
	_pick_up->clamping(OFF);
	pick_up_pid_init(&rabbit.pick_up, pid_speed_init_callback, pid_angle_init_callback);
	
	_pick_up->motor[0].initial_ecd = _pick_up->motor[0].feedback.ecd;
	_pick_up->motor[1].initial_ecd = _pick_up->motor[1].feedback.ecd;
}


/**
    	*@brief 
    	*@param
    	*@param	low_pass
		*@param	mode
		*@param remote
		*@param	max_speed
    	*@retval 
    **/    	
void pick_up_motor_speed_set(PICK_UP *_pick_up, P3_key (*callback)(p3 *_key), int _up_speed, int _down_speed, P3_ch (*callback2)(p3 *_ch))
{
	
	
	if((*callback2)(&p3_data).ch[1]>=120 && pick_up_motor_state==MID)
	{
		_pick_up->motor[0].target_rpm = _up_speed;
		_pick_up->motor[1].target_rpm = -_up_speed;
		pick_up_motor_state=UP;
	}
	else if((*callback2)(&p3_data).ch[1]<=-120 && pick_up_motor_state==MID)
	{
		_pick_up->motor[0].target_rpm = -_down_speed;
		_pick_up->motor[1].target_rpm = _down_speed;
		pick_up_motor_state=DOWN;
	}
	else if((*callback)(&p3_data).R_key==1)
	{
		_pick_up->motor[0].target_rpm = 0;
		_pick_up->motor[1].target_rpm = 0;
		pick_up_motor_state=MID;
	}
		
}





/**
	*@brief 
	*@param mode
	*@param	remote
	*@retval 
**/
void pick_up_clamping(PICK_UP *_pick_up, P3_key  (*callback)(p3 *_key), p3 *_p3)
{

	if((*callback)(&p3_data).RU && flag==1 && (p3_RU_last!=_p3->RU))
	{
		_pick_up->clamping(ON);
		flag=0;
	}
	else if((*callback)(&p3_data).RU && flag==0 && ((p3_RU_last!=_p3->RU)))
	{
		_pick_up->clamping(OFF);
		
		flag=1;
	}
	p3_RU_last=(*callback)(&p3_data).RU;
}
	


/**
	*@brief 
	*@param 
	*@param pid_speed_init_callback
	*@param pid_angle_init_callback
	*@retval 
**/
void pick_up_pid_init(PICK_UP *_pick_up, void (*callback1)(MOTOR *_motor, float _arr[6]), void (*callback2)(MOTOR *_motor, float _arr[6]))
{
	(*callback1)(&_pick_up->motor[0], pick_up_speed_pid_param);//speed callback1
	(*callback2)(&_pick_up->motor[0], pick_up_angle_pid_param);//angle callback2
	(*callback1)(&_pick_up->motor[1], pick_up_speed_pid_param);//speed callback1
	(*callback2)(&_pick_up->motor[1], pick_up_angle_pid_param);//angle callback2
}
	
void pick_up_motor_pid_cal(PICK_UP *_pick_up, void (*callback)(MOTOR *_motor, float _target))
{
	(*callback)(&_pick_up->motor[0], _pick_up->motor[0].target_rpm);
	(*callback)(&_pick_up->motor[1], _pick_up->motor[1].target_rpm);
}

void pick_up_motor_drive(PICK_UP *_pick_up, uint16_t std_id, void (*callback)(uint16_t stid, int i1,int i2,int i3,int i4), SHOOT *_shoot)
{
	(*callback)(std_id,	_pick_up->motor[0].pid_param_speed.out,
						_pick_up->motor[1].pid_param_speed.out, 
						0, 
						0 );
}

void pick_up_Clamping(int x)
{
	Clamping=x;
}





