#include "rabbit.h"


static float shoot_speed_pid_param[6]={30.0f, 1.0f, 0.0f, 20000.0f, 500.0f, 500.0f};
static float shoot_angle_pid_param[6]={0.3f, 1.0f, 0.0f, 2000.0f, 500.0f, 500.0f};

static char flag1=0;
static char flag2=1;

static char p3_RD_last;
void shoot_task(void *pvParameters)
{
	shoot_init(&rabbit.shoot, 294876);
	LED2=0;
	while(1)
	{		
		shoot_push(&rabbit.shoot, status_p3_key_callback, 2000);
		shoot_motor_pid_cal(&rabbit.shoot, motor_speed_pid_cal_callback, motor_angle_pid_cal_callback);
		shoot_motor_drive(&rabbit.shoot, &rabbit.pick_up, 0x200, 0x200, motor_driver_can1, motor_driver_can2);
		
		shoot_lifting(&rabbit.shoot, status_p3_key_callback);
		
		shoot_fly_motor_speed_set(&rabbit.shoot, status_p3_key_callback, &p3_data);
		shoot_fly_motor_drive(rabbit.shoot.shoot_rpm);
		vTaskDelay(1);
	}
}


void shoot_init(SHOOT *_shoot, int _target_angle)
{
	delay_ms(5000);
	_shoot->push=shoot_Push;
	_shoot->lifting=shoot_Lifting;	
	shoot_motor_pid_init(_shoot, pid_speed_init_callback, pid_angle_init_callback);
	shoot_fly_motor_drive(1000);
}




/**
	*@brief 
	*@param 
	*@param mode
	*@param status
	*@param remote
	*@retval 
**/
void shoot_fly_motor_speed_set(SHOOT *_shoot, P3_key (*callback)(p3 *_key), p3 *_p3)
{
	if(_p3->LU)
		_shoot->shoot_rpm = shoot_rpm_set_high;
	
	else if(_p3->LL)
		_shoot->shoot_rpm = shoot_rpm_set_mid;
	
	else if(_p3->LR)
		_shoot->shoot_rpm = shoot_rpm_set_low;		

	else if(_p3->LD)
		_shoot->shoot_rpm = shoot_rpm_set_cls;
	
	if((*callback)(&p3_data).L1 && flag1==1)
	{
		_shoot->shoot_rpm+=1;
		flag1=0;
	}
	else if((*callback)(&p3_data).R1 && flag1==1)
	{
		_shoot->shoot_rpm-=1;
		flag1=0;
	}
	else if(((*callback)(&p3_data).L1==0 && (*callback)(&p3_data).R1==0) && flag1==0)
	{
		flag1=1;
	}
	
	
}



/**
	*@brief 
	*@param 
	*@param mode
	*@param remote
	*@param motor_speed
	*@retval 
**/
void shoot_push(SHOOT *_shoot, P3_key (*callback)(p3 *_key), int _speed)
{
	if((*callback)(&p3_data).L2==1)
		_shoot->push(ON);
	else if((*callback)(&p3_data).L2==0)
		_shoot->push(OFF);
	
	if((*callback)(&p3_data).R2==1)
	{
		_shoot->motor.target_angle = 147438;
		_shoot->motor.target_rpm = _speed;
	}
	
	if(_shoot->motor.feedback.all_ecd >= _shoot->motor.target_angle)
	{
		_shoot->motor.target_angle = 0;
		_shoot->motor.target_rpm = 0;
		_shoot->motor.feedback.count=0;
		_shoot->motor.feedback.all_ecd=0;
	}
	
}



/**
	*@brief 
	*@param 
	*@param pid_speed_init_callback
	*@param pid_angle_init_callback
	*@retval 
**/
void shoot_motor_pid_init(SHOOT *_shoot, void (*callback1)(MOTOR *_motor, float _arr[6]), void (*callback2)(MOTOR *_motor, float _arr[6]))
{
	(*callback1)(&_shoot->motor, shoot_speed_pid_param);
	(*callback2)(&_shoot->motor, shoot_angle_pid_param);
}


/**
	*@brief 
	*@param 
	*@param motor_speed_pid_cal_callback
	*@param motor_angle_pid_cal_callback
	*@retval 
**/
void shoot_motor_pid_cal(SHOOT *_shoot, void (*callback1)(MOTOR *_motor, float _target), void (*callback2)(MOTOR *_motor, float _target))
{
	(*callback1)(&_shoot->motor, _shoot->motor.target_rpm);
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
void shoot_motor_drive(SHOOT *_shoot, PICK_UP *_pick_up,  uint16_t std_id1, uint16_t std_id2, void (*callback1)(uint16_t stid, int i1,int i2,int i3,int i4), void (*callback2)(uint16_t stid, int i1,int i2,int i3,int i4))
{	
	/*worning!! 可能会造成某些bug*/
	(*callback2)(std_id2, 		_shoot->motor.pid_param_speed.out,
								0,
								0,
								0									 );
}


/**
	*@brief 发射驱动 
	*@param 发射速度
	*@retval none
**/
static int _speed_last;
void shoot_fly_motor_drive(int _speed)
{
	if(_speed!=_speed_last)
	{
		TIM_SetCompare1(TIM5, _speed);
		TIM_SetCompare2(TIM5, _speed);
		_speed_last=_speed;
	}
	
}

void shoot_lifting(SHOOT *_shoot, P3_key (*callback)(p3 *_key))
{
	if((*callback)(&p3_data).RD==1 && flag2==1 && (p3_RD_last!=(*callback)(&p3_data).RD))
	{
		_shoot->lifting(ON);
		flag2=0;
	}
	else if((*callback)(&p3_data).RD==1 && flag2==0 && (p3_RD_last!=(*callback)(&p3_data).RD))
	{
		_shoot->lifting(OFF);
		flag2=1;
	}
	p3_RD_last=(*callback)(&p3_data).RD;
}


void shoot_Push(int x)
{
	Push=x;
}

void shoot_Lifting(int x)
{
	Lifting=x;
}





