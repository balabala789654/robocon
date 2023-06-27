#include "main.h"


static double empty_address_pitch=-20;
static double empty_address_door;
static double empty_address_lifting=20;
static double empty_address_clamping=0;

float door_motor_pid_params[2][6]={
										{7.0, 1.0, 0.0, 5000, 200, 200},//speed
										{0.1, 0.1, 0.1, 1000, 200, 200}	//angle
																			};
float lifting_motor_pid_params[2][6]={
										{7.0, 1.0, 0.0, 7000, 200, 200},//speed
										{0.1, 0.1, 0.1, 1000, 200, 200} //angle
																			};
float pitch_motor_pid_params[2][6]={
										{15.0, 1.0, 0.0, 30000, 200, 200},//speed
										{0.1, 0.1, 0.1, 1000, 200, 200} //angle
																			};

																			
/**
	*@brief 
	*@param 
	*@param	
	*@retval 
**/
void clamping_init(void)
{
	
	pitch_motor_init(&rabbit.clamping, pitch_motor_pid_params);
	lifting_motor_init(&rabbit.clamping, lifting_motor_pid_params);
	door_motor_init(&rabbit.clamping, door_motor_pid_params);
	
}


void clamping_task(void *pvParameters)
{
	
	
	char flag=0;
	clamping_init();
	set_clamping_motor_rpm(&rabbit.clamping, 3000, 4000, 5000);//_lifting, _door, _pitch
	while(1)
	{
	
		if(rabbit.rabbit_state)
		{
			
			if(rabbit.clamping.clamping_switch==ON&&flag==0)//传入遥控器地址
			{
				clamping_address_transform(&rabbit.clamping, (double* )&rc_ctrl.rc.ch[3], (double* )&rc_ctrl.rc.ch[1], (double* )&rc_ctrl.rc.ch[4]);
				flag++;
			}
			else if(rabbit.clamping.clamping_switch!=ON&&flag==1)//传入空地址
			{
				clamping_address_transform(&rabbit.clamping, &empty_address_pitch, &empty_address_lifting, &empty_address_door);
				flag--;
			}
			
			lifting_start(&rabbit.clamping, rabbit.clamping.clamping_height);
			door_start(&rabbit.clamping, rabbit.clamping.clamping_door);
			pitch_start(&rabbit.clamping, rabbit.clamping.clamping_pitch);
			
			motor_driver_can2(	clamping_motor_StdId,
								rabbit.clamping.lifting_motor[0].pid_param.out,
								rabbit.clamping.lifting_motor[1].pid_param.out,
								rabbit.clamping.door_motor[0].pid_param.out,
								rabbit.clamping.door_motor[1].pid_param.out
																				);
			
			motor_driver_can2(pitch_motor_StdId, rabbit.clamping.pitch_motor.pid_param.out, 0, 0, 0);
		}
		else
		{
			clamping_address_transform(&rabbit.clamping, &empty_address_clamping, &empty_address_clamping, &empty_address_clamping);
			
			motor_driver_can2(clamping_motor_StdId, 0, 0, 0, 0);
			motor_driver_can2(pitch_motor_StdId, 0, 0, 0, 0);
		}
		vTaskDelay(1);
	}
	
	
}

/*   双开门电冰箱    */
void door_open_or_close(CLAMPING* _clamping, char state)
{
	
	if(state) {_clamping->door_motor[0].target_angle = 8191; _clamping->door_motor[1].target_angle=-8191;}
	else {_clamping->door_motor[0].target_angle=0; _clamping->door_motor[1].target_angle=0;}
}


/**
	*@brief 
	*@param 
	*@param	
	*@retval 
**/
void clamping_address_transform(CLAMPING* _clamping, double* remote_ch_3, double* remote_ch_1, double* remote_ch_4)
{
	_clamping->clamping_pitch = remote_ch_3;
	_clamping->clamping_height = remote_ch_1;
	_clamping->clamping_door = remote_ch_4;
}



/**
	*@brief 
	*@param 
	*@param	
	*@retval 
**/
void lifting_start(CLAMPING* _clamping, double* _height)
{
//	static double last_height,current_height;
//	static double all_height;

//	current_height = *_height;
//	
//	all_height+=(current_height-last_height);
	
	_clamping->lifting_motor[0].target_rpm = *_height*_clamping->max_lifting_motor_rpm/660;
	_clamping->lifting_motor[1].target_rpm = -(*_height*_clamping->max_lifting_motor_rpm/660);
	
//	PID_calc(	&rabbit.clamping.lifting_motor[0].angle_pid_param, 
//				rabbit.clamping.lifting_motor[0].feedback.all_ecd, 
//				rabbit.clamping.lifting_motor[0].target_angle				
//																		);
	
	
	PID_calc(	&rabbit.clamping.lifting_motor[0].pid_param, 
				rabbit.clamping.lifting_motor[0].feedback.speed_rpm, 
				rabbit.clamping.lifting_motor[0].target_rpm
																		);	
	
//	PID_calc(	&rabbit.clamping.lifting_motor[1].angle_pid_param, 
//				rabbit.clamping.lifting_motor[1].feedback.all_ecd, 
//				rabbit.clamping.lifting_motor[1].target_angle				
//																		);
//	
//	
	PID_calc(	&rabbit.clamping.lifting_motor[1].pid_param, 
				rabbit.clamping.lifting_motor[1].feedback.speed_rpm, 
				rabbit.clamping.lifting_motor[1].target_rpm
																		);	
	
//	last_height = current_height;
}



/**
	*@brief 
	*@param 
	*@param	
	*@retval 
**/
void door_start(CLAMPING* _clamping, double* _door)
{
	
	_clamping->door_motor[0].target_rpm = *_door*_clamping->max_door_motor_rpm/660;
	_clamping->door_motor[1].target_rpm = -(*_door*_clamping->max_door_motor_rpm/660);
	
//	PID_calc(	&_clamping->door_motor[0].angle_pid_param, 
//				_clamping->door_motor[0].feedback.all_ecd, 
//				_clamping->door_motor[0].target_angle
//																		);
	
	
	PID_calc(	&_clamping->door_motor[0].pid_param, 
				_clamping->door_motor[0].feedback.speed_rpm, 
				_clamping->door_motor[0].target_rpm
																		);
	
	
//	PID_calc(	&_clamping->door_motor[1].angle_pid_param, 
//				_clamping->door_motor[1].feedback.all_ecd, 
//				_clamping->door_motor[1].target_angle
//																		);
	
	
	PID_calc(	&_clamping->door_motor[1].pid_param, 
				_clamping->door_motor[1].feedback.speed_rpm, 
				_clamping->door_motor[1].target_rpm
																		);

}

/**
	*@brief 
	*@param 
	*@param	
	*@retval 
**/
void pitch_start(CLAMPING* _clamping, double* _pitch)
{
	
	_clamping->pitch_motor.target_rpm = (*_pitch*_clamping->max_pitch_motor_rpm/660)-50;
	
//	PID_calc(	&_clamping->pitch_motor.angle_pid_param, 
//				_clamping->pitch_motor.feedback.all_ecd, 
//				_clamping->pitch_motor.target_angle
//																		);
	
	
	PID_calc(	&_clamping->pitch_motor.pid_param, 
				_clamping->pitch_motor.feedback.speed_rpm, 
				_clamping->pitch_motor.target_rpm
																		);

	
}

void set_clamping_motor_rpm(CLAMPING* _clamping, int _lifting, int _door, int _pitch)
{
	_clamping->max_door_motor_rpm = _door;
	_clamping->max_lifting_motor_rpm = _lifting;
	_clamping->max_pitch_motor_rpm = _pitch;
}





//又臭又长，建议别看
void pitch_motor_init(CLAMPING* _clamping, float params[2][6])
{
	PID_init(&_clamping->pitch_motor.pid_param,PID_POSITION,params[0][0],params[0][1],params[0][2],params[0][3],params[0][4],params[0][5]);
	PID_init(&_clamping->pitch_motor.angle_pid_param,PID_POSITION,params[1][0],params[1][1],params[1][2],params[1][3],params[1][4],params[1][5]);
}



void lifting_motor_init(CLAMPING* _clamping, float params[2][6])
{
	_clamping->lifting_motor[0].target_angle=0;
	PID_init(&_clamping->lifting_motor[0].pid_param,PID_POSITION,params[0][0],params[0][1],params[0][2],params[0][3],params[0][4],params[0][5]);
	PID_init(&_clamping->lifting_motor[0].angle_pid_param,PID_POSITION,params[1][0],params[1][1],params[1][2],params[1][3],params[1][4],params[1][5]);
	
	_clamping->lifting_motor[1].target_angle=0;
	PID_init(&_clamping->lifting_motor[1].pid_param,PID_POSITION,params[0][0],params[0][1],params[0][2],params[0][3],params[0][4],params[0][5]);
	PID_init(&_clamping->lifting_motor[1].angle_pid_param,PID_POSITION,params[1][0],params[1][1],params[1][2],params[1][3],params[1][4],params[1][5]);

}

void door_motor_init(CLAMPING* _clamping, float params[2][6])
{
	for(int i=0;i<2;i++)
	{
		PID_init(&_clamping->door_motor[i].pid_param,PID_POSITION,params[0][0],params[0][1],params[0][2],params[0][3],params[0][4],params[0][5]);
	}
	
	for(int i=0;i<2;i++)
	{
		PID_init(&_clamping->door_motor[i].angle_pid_param,PID_POSITION,params[1][0],params[1][1],params[1][2],params[1][3],params[1][4],params[1][5]);
	}
	
}

