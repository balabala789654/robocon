#include "rabbit.h"

static float chassis_pid_param[6]={10.0f, 1.0f, 0.0f, 15000.0f, 500.0f, 500.0f};


void chassis_task(void *pvParameters)
{
	chassis_motor_pid_init(&rabbit.chassis, pid_speed_init_callback);
	chassis_max_speed_set(&rabbit.chassis, 10000, 5000);
	while(1)
	{		
		chassis_solve(&rabbit.chassis, Low_pass_output);		
		
		chassis_motor_pid_cal(&rabbit.chassis, motor_speed_pid_cal_callback);
		chassis_motor_drive(&rabbit.chassis, 0x200, motor_driver_can1);
		
		
		/*debug*/
//		printf("ch: [0]: %d,[1]: %d, [2]: %d, [3]: %d\r\n",p3_data.ch[0],p3_data.ch[1],p3_data.ch[2],p3_data.ch[3] );
//		printf("ch: LU: %d,LD: %d, LL: %d, LR: %d, RU: %d, RD: %d, RL: %d, RR: %d\r\n",p3_data.LU,p3_data.LD,p3_data.LL,p3_data.LR, p3_data.RU,p3_data.RD,p3_data.RL,p3_data.RR );
//		printf("ch:RU: %d,RD: %d, RL: %d, RR: %d\r\n",p3_data.RU,p3_data.RD,p3_data.RL,p3_data.RR );
//		printf("ch:L1: %d,L2: %d, R1: %d, R2: %d\r\n",p3_data.L1,p3_data.L2,p3_data.R1,p3_data.R2);

		vTaskDelay(1);
	}
}


int chassis_max_speed_set(CHASSIS *_chassis, int _max_straight_speed, int _max_spin_speed)
{
	_chassis->max_straight_speed =_max_straight_speed;
	_chassis->max_spin_speed = _max_spin_speed;
	return _chassis->max_straight_speed;
}

/**
	*@brief 
	*@param 
	*@param	dead_zone
	*@retval 
**/
void chassis_solve(CHASSIS *_chassis,LOW_PASS (*callback)(DEAD_ZONE (*input)(P3_ch (*callback)(p3 *_ch))))
{
	float32_t _speed_x, _speed_y, _speed_w;
	
	float32_t vx, vx_1, vx_2;
	float32_t vy, vy_1, vy_2;
	float32_t vw;
	
	float32_t cos_arr= arm_cos_f32(3.14159/4);
	float32_t sin_arr= arm_sin_f32(3.14159/4);
	
	_speed_x = (*callback)(dead_zone_output).ch[2]*(float)(_chassis->max_straight_speed)/127;
	_speed_y = (*callback)(dead_zone_output).ch[3]*(float)(_chassis->max_straight_speed)/127;
	_speed_w = (*callback)(dead_zone_output).ch[0]*(float)(_chassis->max_spin_speed)/127;
	
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
	
	/*debug*/
	//printf("%f,%f\r\n", (*callback)(dead_zone_output).ch[3], rc_ctrl.rc.ch[3]);
}


void chassis_motor_pid_cal(CHASSIS *_chassis, void (*callback)(MOTOR *_motor, float  target))
{
	(*callback)(&_chassis->motor[0], _chassis->motor[0].target_rpm);
	(*callback)(&_chassis->motor[1], _chassis->motor[1].target_rpm);
	(*callback)(&_chassis->motor[2], _chassis->motor[2].target_rpm);
	(*callback)(&_chassis->motor[3], _chassis->motor[3].target_rpm);
}

void chassis_motor_drive(CHASSIS *_chassis, uint16_t std_id, void (*callback)(uint16_t stid, int i1,int i2,int i3,int i4))
{
	(*callback)(std_id, 	_chassis->motor[0].pid_param_speed.out, 
								_chassis->motor[1].pid_param_speed.out, 
								_chassis->motor[2].pid_param_speed.out, 
								_chassis->motor[3].pid_param_speed.out );
	
}
void chassis_motor_pid_init(CHASSIS *_chassis, void (*callback)(MOTOR *_motor, float _arr[6]))
{
	for(int i=0; i<4; i++)
	{
		(*callback)(&_chassis->motor[i], chassis_pid_param);
	}
	
}

//void chassis_slow_mode(CHASSIS *_chassis, P3_key (*callback)(p3 *_key)
//)
//{
//	if((*callback)(&p3_data).L_key==1)
//	{
//		chassis_max_speed_set(&rabbit.chassis, 10000, 3000);
//		
//	}
//	else if((*callback)(&p3_data)==mode_2)
//	{
//		chassis_max_speed_set(&rabbit.chassis, 1000, 1000);
//	}
//	else if((*callback)(&p3_data)==mode_1)
//	{
//		chassis_max_speed_set(&rabbit.chassis, 0, 0);
//	}
//}

void chassis_close(CHASSIS *_chassis)
{
	for(int i=0; i<4; i++)
	{
		_chassis->motor[i].target_rpm=0;
	}
	
}



