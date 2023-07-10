#include "rabbit.h"
#include "status.h"

static float chassis_pid_param[6]={10.0f, 1.0f, 0.0f, 15000.0f, 500.0f, 500.0f};

static char flag=1;

__packed double *empty_arr_1;
__packed double *empty_arr_2;
__packed double *empty_arr_3;

void chassis_task(void *pvParameters)
{
	chassis_motor_pid_init(&rabbit.chassis);
	while(1)
	{
		if(flag && rabbit.status==Status_chassis) 
		{
			chassis_address_transform(&rabbit.chassis, &rc_ctrl.rc.ch[2], &rc_ctrl.rc.ch[3],&rc_ctrl.rc.ch[0]);
			flag=0;
		}
		else if(flag!=1 && rabbit.status!=Status_chassis) 
		{
			chassis_address_transform(&rabbit.chassis, empty_arr_1, empty_arr_2, empty_arr_3);
			flag=1;
		}
		
		chassis_slow_mode(&rabbit.chassis, &rabbit);
		chassis_solve(&rabbit.chassis);
		chassis_motor_pid_cal(&rabbit.chassis);
		chassis_motor_drive(&rabbit.chassis, 0x200);
		
		vTaskDelay(1);
	}
}


void chassis_max_speed_set(CHASSIS *_chassis, int _max_straight_speed, int _max_spin_speed)
{
	_chassis->max_straight_speed =_max_straight_speed;
	_chassis->max_spin_speed = _max_spin_speed;
}

void chassis_solve(CHASSIS *_chassis)
{
	float32_t _speed_x, _speed_y, _speed_w;
	
	float32_t vx, vx_1, vx_2;
	float32_t vy, vy_1, vy_2;
	float32_t vw;
	
	float32_t cos_arr= arm_cos_f32(3.14159/4);
	float32_t sin_arr= arm_sin_f32(3.14159/4);
	
	_speed_x = *_chassis->speed_x*(float)(_chassis->max_straight_speed)/660;
	_speed_y = *_chassis->speed_y*(float)(_chassis->max_straight_speed)/660;
	_speed_w = *_chassis->speed_w*(float)(_chassis->max_spin_speed)/660;
	
	arm_mult_f32(&sin_arr, &_speed_y, &vx_1, 1);
	arm_mult_f32(&cos_arr, &_speed_x, &vx_2, 1);
	arm_add_f32(&vx_1, &vx_2, &vx, 1);
	
	arm_mult_f32(&cos_arr, &_speed_y, &vy_1, 1);
	arm_mult_f32(&sin_arr, &_speed_x, &vy_2, 1);
	arm_sub_f32(&vy_1, &vy_2, &vy, 1);
	
	vw = _speed_w;
	
	_chassis->motor[0].target_rpm = -vy+vw;
	_chassis->motor[2].target_rpm = vy+vw;
	
	_chassis->motor[1].target_rpm = vx+vw;
	_chassis->motor[3].target_rpm = -vx+vw;
}

void chassis_address_transform(CHASSIS *_chassis, __packed double *_arr1, __packed double *_arr2, __packed double *_arr3)
{
	_chassis->speed_x = _arr1;
	_chassis->speed_y = _arr2;
	_chassis->speed_w = _arr3;
	
}

void chassis_motor_pid_cal(CHASSIS *_chassis)
{
	motor_speed_pid_cal(&_chassis->motor[0]);
	motor_speed_pid_cal(&_chassis->motor[1]);
	motor_speed_pid_cal(&_chassis->motor[2]);
	motor_speed_pid_cal(&_chassis->motor[3]);
}

void chassis_motor_drive(CHASSIS *_chassis, uint16_t std_id)
{
	motor_driver_can1(std_id, 	_chassis->motor[0].pid_param_speed.out, 
								_chassis->motor[1].pid_param_speed.out, 
								_chassis->motor[2].pid_param_speed.out, 
								_chassis->motor[3].pid_param_speed.out );
	
}
void chassis_motor_pid_init(CHASSIS *_chassis)
{
	for(int i=0; i<4; i++)
	{
		PID_init(&_chassis->motor[i].pid_param_speed, PID_POSITION, chassis_pid_param[0], chassis_pid_param[1], chassis_pid_param[2], chassis_pid_param[3], chassis_pid_param[4], chassis_pid_param[5]);
	}
	
}

void chassis_slow_mode(CHASSIS *_chassis, RABBIT *_rabbit)
{
	if(_rabbit->mode==mode_0)
	{
		chassis_max_speed_set(&rabbit.chassis, 10000, 3000);
	}
	else if(_rabbit->mode==mode_2)
	{
		chassis_max_speed_set(&rabbit.chassis, 1000, 1000);
	}
	else if(_rabbit->mode==mode_1)
	{
		chassis_max_speed_set(&rabbit.chassis, 0, 0);
	}
}




