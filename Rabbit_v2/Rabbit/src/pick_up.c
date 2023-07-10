#include "rabbit.h"

static float pick_up_speed_pid_param[6]={20.0f, 1.0f, 0.0f, 40000.0f, 500.0f, 500.0f};
static float pick_up_angle_pid_param[6]={0.3f, 1.0f, 0.0f, 2000.0f, 500.0f, 500.0f};


void pick_up_task(void *pvParameters)
{
	pick_up_init();
	while(1)
	{
		pick_up_clamping(status_mode_change_callback, status_remote_callback);
		pick_up_motor_speed_set(&rabbit.pick_up, Low_pass_output, status_mode_change_callback, status_remote_callback, 1000);
		pick_up_motor_pid_cal(&rabbit.pick_up, motor_speed_pid_cal_callback);
		pick_up_motor_drive(&rabbit.pick_up, 0x200, motor_driver_can2);
	}
}

void pick_up_init(void)
{
	clamping=OFF;
	pick_up_pid_init(&rabbit.pick_up, pid_speed_init_callback, pid_angle_init_callback);
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
void pick_up_motor_speed_set(PICK_UP *_pick_up, LOW_PASS (*callback1)(DEAD_ZONE (*input)(RC_ctrl_t *_rc, REMOTE (*callback1)(RC_ctrl_t *_rc))), char (*callback2)(RC_ctrl_t *_rc), REMOTE (*callback3)(RC_ctrl_t *_rc), int _max_speed)
{
	if((*callback3)(&rc_ctrl).s[0]==3 && (*callback2)(&rc_ctrl)==mode_1)
	{
		_pick_up->motor[0].target_rpm = (*callback1)(dead_zone_output).ch[3]*_max_speed/660;
//		_pick_up->motor[0].target_rpm=0;
		_pick_up->motor[1].target_rpm = -(*callback1)(dead_zone_output).ch[3]*_max_speed/660;
	}
	else 
	{
		_pick_up->motor[0].target_rpm=0;
		_pick_up->motor[1].target_rpm=0;
	}
		
}





/**
	*@brief 
	*@param mode
	*@param	remote
	*@retval 
**/
void pick_up_clamping(char (*callback1)(RC_ctrl_t *_rc), REMOTE (*callback2)(RC_ctrl_t *_rc))
{
	if((*callback1)(&rc_ctrl)==mode_0 && (*callback2)(&rc_ctrl).s[0]==3)
	{
		if((*callback2)(&rc_ctrl).s[1]==3)
			clamping=OFF;
		else if((*callback2)(&rc_ctrl).s[1]==1)
			clamping=ON;
	}
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
	(*callback1)(&_pick_up->motor[0], pick_up_speed_pid_param);
	(*callback2)(&_pick_up->motor[0], pick_up_angle_pid_param);
	(*callback1)(&_pick_up->motor[1], pick_up_speed_pid_param);
	(*callback2)(&_pick_up->motor[1], pick_up_angle_pid_param);
}
	
void pick_up_motor_pid_cal(PICK_UP *_pick_up, void (*callback)(MOTOR *_motor))
{
	(*callback)(&_pick_up->motor[0]);
	(*callback)(&_pick_up->motor[1]);
}

void pick_up_motor_drive(PICK_UP *_pick_up, uint16_t std_id, void (*callback)(uint16_t stid, int i1,int i2,int i3,int i4))
{
	(*callback)(std_id, 0, 
						0, 
						_pick_up->motor[0].pid_param_speed.out, 
						_pick_up->motor[1].pid_param_speed.out);
}




