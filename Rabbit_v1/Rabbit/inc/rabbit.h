#ifndef _RABBIT_H
#define _RABBIT_H

#include "stdlib.h"
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "motor_feedback.h"
#include "Remote_Control.h"
#include "arm_math.h"
#include "gpio.h"
#include "can.h"
#include "usart.h"
#include "timer.h"
#include "delay.h"
#include "sys.h"
#include "stdio.h"
#include "dead_zone.h"

typedef struct
{
	char lifting;
    int shoot_rpm_set_low;
	int shoot_rpm_set_mid;
	int shoot_rpm_set_high;
	int shoot_rpm;
		
	int motor_speed_1;
	__packed double *motor_speed_2;
	__packed double *motor_speed_3;
	
	MOTOR motor[3];
	
	
}SHOOT;

typedef struct
{
	int a;
}PICK_UP;

typedef struct
{
	__packed double *speed_y;
	__packed double *speed_x;
	__packed double *speed_w;
	
	int max_straight_speed;
	int max_spin_speed;
	
	MOTOR motor[4];
	
}CHASSIS;

typedef struct
{
	char status;
	char mode;
    SHOOT shoot;
    PICK_UP pick_up;
    CHASSIS chassis;

}RABBIT;



extern RABBIT rabbit;

/*****************************   rabbit.c   **********************************/
void chassis_task(void *pvParameters);
void shoot_task(void *pvParameters);
void pick_up_task(void *pvParameters);
void system_init(void);
void status_task_start(void);

/*****************************************************************************/


/**************************************  chassis.c   **************************************/
void chassis_max_speed_set(CHASSIS *_chassis, int _max_straight_speed, int _max_spin_speed);
void chassis_solve(CHASSIS *_chassis);
void chassis_address_transform(CHASSIS *_chassis, __packed double *_arr1, __packed double *_arr2, __packed double *_arr3);
void chassis_motor_pid_cal(CHASSIS *_chassis);
void chassis_motor_drive(CHASSIS *_chassis, uint16_t std_id);
void chassis_motor_pid_init(CHASSIS *_chassis);
void chassis_slow_mode(CHASSIS *_chassis, RABBIT *_rabbit);

/******************************************************************************************/


/**************************************  pick_up.c   **************************************/
/******************************************************************************************/


/**************************************  shoot.c   ****************************************/
void shoot_set_enter_motor_speeds(SHOOT *_shoot, int _set_speed1, int _set_speed2);
void shoot_set_rpm(SHOOT *_shoot, int _speed1, int _speed2, int _speed3, int _speed4);
void shoot_fly_motor_speed_set(SHOOT *_shoot, RABBIT *_rabbit, RC_ctrl_t *_rc);
void shoot_enter_motor_loop(SHOOT *_shoot, RABBIT *_rabbit, RC_ctrl_t *_rc);
void shoot_motor_pid_cal(SHOOT *_shoot);
void shoot_motor_drive(SHOOT *_shoot, uint16_t std_id, uint16_t std_id_2);
void shoot_address_transform(SHOOT *_shoot, __packed double *_arr1, __packed double *_arr2);
void shoot_motor_pid_init(SHOOT *_shoot);
void shoot_read_initial_ecd(SHOOT *_shoot);
void shoot_fly_motor_drive(int _speed);
void shoot_motor_close(SHOOT *_shoot);
void shoot_lifting(SHOOT *_shoot);
void shoot_lifting_set(SHOOT *_shoot, RABBIT *_rabbit, RC_ctrl_t *_rc);
void shoot_lifting_init(SHOOT *_shoot);
void shoot_init(void);

/******************************************************************************************/
#endif



