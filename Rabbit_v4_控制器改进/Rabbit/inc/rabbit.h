#ifndef _RABBIT_H
#define _RABBIT_H

#include "stdlib.h"
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "motor_feedback.h"
#include "arm_math.h"
#include "gpio.h"
#include "can.h"
#include "usart.h"
#include "timer.h"
#include "delay.h"
#include "sys.h"
#include "stdio.h"
#include "dead_zone.h"
#include "status.h"
#include "Low_pass.h"
#include "usart.h"
#include "stdio.h"
#include "status.h"
#include "p3.h"





typedef struct
{
	void (*push)(int x);
	void (*lifting)(int x);
	int shoot_rpm;
	MOTOR motor;
}SHOOT;

typedef struct
{
	void (*clamping)(int x);
	MOTOR motor[2];
}PICK_UP;

typedef struct
{	
	int max_straight_speed;
	int max_spin_speed;
	
	MOTOR motor[4];
	
}CHASSIS;

typedef struct
{
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

/*****************************************************************************/


/**************************************  chassis.c   **************************************/




int chassis_max_speed_set(CHASSIS *_chassis, int _max_straight_speed, int _max_spin_speed);
void chassis_solve(CHASSIS *_chassis,LOW_PASS (*callback)(DEAD_ZONE (*input)(P3_ch (*callback)(p3 *_ch))));
void chassis_address_transform(CHASSIS *_chassis, __packed double *_arr1, __packed double *_arr2, __packed double *_arr3);
void chassis_motor_pid_cal(CHASSIS *_chassis, void (*callback)(MOTOR *_motor, float  target));
void chassis_motor_drive(CHASSIS *_chassis, uint16_t std_id, void (*callback)(uint16_t stid, int i1,int i2,int i3,int i4));
void chassis_motor_pid_init(CHASSIS *_chassis, void (*callback)(MOTOR *_motor, float _arr[6]));
void chassis_close(CHASSIS *_chassis);

/******************************************************************************************/


/**************************************  pick_up.c   **************************************/




void pick_up_init(PICK_UP *_pick_up);
void pick_up_clamping(PICK_UP *_pick_up, P3_key  (*callback)(p3 *_key), p3 *_p3);
void pick_up_pid_init(PICK_UP *_pick_up, void (*callback1)(MOTOR *_motor, float _arr[6]), void (*callback2)(MOTOR *_motor, float _arr[6]));
void pick_up_motor_pid_cal(PICK_UP *_pick_up, void (*callback)(MOTOR *_motor, float _target));
void pick_up_motor_drive(PICK_UP *_pick_up, uint16_t std_id, void (*callback)(uint16_t stid, int i1,int i2,int i3,int i4), SHOOT *_shoot);
void pick_up_motor_speed_set(PICK_UP *_pick_up, P3_key (*callback)(p3 *_key), int _up_speed, int _down_speed, P3_ch (*callback2)(p3 *_ch));
void pick_up_Clamping(int x);

/******************************************************************************************/


/**************************************  shoot.c   ****************************************/

#define shoot_rpm_set_high 200
#define shoot_rpm_set_mid 160
#define shoot_rpm_set_low 130
#define shoot_rpm_set_cls 100
void shoot_Push(int x);
void shoot_Lifting(int x);
void shoot_init(SHOOT *_shoot, int _target_angle);
void shoot_motor_pid_init(SHOOT *_shoot, void (*callback1)(MOTOR *_motor, float _arr[6]), void (*callback2)(MOTOR *_motor, float _arr[6]));
void shoot_motor_pid_cal(SHOOT *_shoot, void (*callback1)(MOTOR *_motor, float _target), void (*callback2)(MOTOR *_motor, float _target));
void shoot_motor_drive(SHOOT *_shoot, PICK_UP *_pick_up, uint16_t std_id1, uint16_t std_id2, void (*callback1)(uint16_t stid, int i1,int i2,int i3,int i4), void (*callback2)(uint16_t stid, int i1,int i2,int i3,int i4));
void shoot_lifting(SHOOT *_shoot, P3_key (*callback)(p3 *_key));
void shoot_push(SHOOT *_shoot, P3_key (*callback)(p3 *_key), int _speed);
void shoot_fly_motor_speed_set(SHOOT *_shoot, P3_key (*callback)(p3 *_key),p3 *_p3);
void shoot_fly_motor_drive(int _speed);


/******************************************************************************************/
#endif



