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
#include "status.h"
#include "Low_pass.h"
#include "usart.h"
#include "stdio.h"

typedef struct
{
	int shoot_rpm;
	int motor_speed_1;	
	MOTOR motor[3];
	
}SHOOT;

typedef struct
{
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
void chassis_max_speed_set(CHASSIS *_chassis, int _max_straight_speed, int _max_spin_speed);
void chassis_solve(CHASSIS *_chassis, LOW_PASS (*callback)(DEAD_ZONE (*input)(RC_ctrl_t *_rc, REMOTE (*callback)(RC_ctrl_t *_rc))));
void chassis_address_transform(CHASSIS *_chassis, __packed double *_arr1, __packed double *_arr2, __packed double *_arr3);
void chassis_motor_pid_cal(CHASSIS *_chassis, void (*callback)(MOTOR *_motor));
void chassis_motor_drive(CHASSIS *_chassis, uint16_t std_id, void (*callback)(uint16_t stid, int i1,int i2,int i3,int i4));
void chassis_motor_pid_init(CHASSIS *_chassis, void (*callback)(MOTOR *_motor, float _arr[6]));
void chassis_slow_mode(CHASSIS *_chassis, char (*callback)(RC_ctrl_t *_rc));
void chassis_close(CHASSIS *_chassis);

/******************************************************************************************/


/**************************************  pick_up.c   **************************************/
void pick_up_init(void);
void pick_up_motor_speed_set(PICK_UP *_pick_up, LOW_PASS (*callback1)(DEAD_ZONE (*input)(RC_ctrl_t *_rc, REMOTE (*callback1)(RC_ctrl_t *_rc))), char (*callback2)(RC_ctrl_t *_rc), REMOTE (*callback3)(RC_ctrl_t *_rc), int _max_speed);
void pick_up_clamping(char (*callback1)(RC_ctrl_t *_rc), REMOTE (*callback2)(RC_ctrl_t *_rc));
void pick_up_pid_init(PICK_UP *_pick_up, void (*callback1)(MOTOR *_motor, float _arr[6]), void (*callback2)(MOTOR *_motor, float _arr[6]));
void pick_up_motor_pid_cal(PICK_UP *_pick_up, void (*callback)(MOTOR *_motor));
void pick_up_motor_drive(PICK_UP *_pick_up, uint16_t std_id, void (*callback)(uint16_t stid, int i1,int i2,int i3,int i4));

/******************************************************************************************/


/**************************************  shoot.c   ****************************************/

#define shoot_rpm_set_high 1500
#define shoot_rpm_set_mid 1400
#define shoot_rpm_set_low 1000

void shoot_set_enter_motor_speeds(SHOOT *_shoot, int _set_speed1);
void shoot_fly_motor_speed_set(SHOOT *_shoot, char (*callback1)(RC_ctrl_t *_rc), char (*callback2)(RC_ctrl_t *_rc), REMOTE (*callback3)(RC_ctrl_t *_rc));
void shoot_enter_motor_loop(SHOOT *_shoot, REMOTE (*callback1)(RC_ctrl_t *_rc) , char (*callback2)(RC_ctrl_t *_rc));
void shoot_motor_pid_cal(SHOOT *_shoot, void (*callback1)(MOTOR *_motor), void (*callback2)(MOTOR *_motor));
void shoot_motor_drive(SHOOT *_shoot, uint16_t std_id1, uint16_t std_id2, void (*callback1)(uint16_t stid, int i1,int i2,int i3,int i4), void (*callback2)(uint16_t stid, int i1,int i2,int i3,int i4));
void shoot_address_transform(SHOOT *_shoot, __packed double *_arr1, __packed double *_arr2);
void shoot_motor_pid_init(SHOOT *_shoot, void (*callback1)(MOTOR *_motor, float _arr[6]), void (*callback2)(MOTOR *_motor, float _arr[6]));
void shoot_read_initial_ecd(SHOOT *_shoot);
void shoot_fly_motor_drive(int _speed);
void shoot_motor_close(SHOOT *_shoot, void (*callback)(MOTOR *_motor));
void shoot_lifting(char (*callback)(char (*callback1)(RC_ctrl_t *_rc), REMOTE (*callback2)(RC_ctrl_t *_rc)));
char shoot_lifting_set_callback(char (*callback1)(RC_ctrl_t *_rc), REMOTE (*callback2)(RC_ctrl_t *_rc));
void shoot_init(void);

/******************************************************************************************/
#endif



