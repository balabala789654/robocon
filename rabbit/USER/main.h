#ifndef _MAIN_H
#define _MAIN_H

#include "stm32f4xx.h"                  // Device header
#include "usart.h"
#include "Remote_Control.h"
#include "can.h"
#include "pid.h"
#include "motor_feedback.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "FreeRTOSConfig.h"
#include "math.h"
#include "rc.h"
#include <string.h>
#include "delay.h"
#include "sys.h"
#include "timer.h"


#include "rabbit.h"

#include "shoot.h"
#include "clamping.h"
#include "chassis.h"
#include "function.h"

#define ON 1
#define OFF 0

#define FreeRtos ON

#define PI 3.14159


#define chassis_motor_StdId 0x200
#define clamping_motor_StdId 0x200
#define shoot_motor_StdId 0x200
#define pitch_motor_StdId 0x1ff

#if clamping_motor_StdId==0x200

#define lifting_motor1_id 0x201
#define lifting_motor2_id 0x202
#define door_motor1_id   0x203
#define door_motor2_id 	 0x204
#define pitch_motor_id   0x205


#else 

#define lifting_motor_id 0x205
#define door_motor1_id   0x206
#define door_motor2_id 	 0x207
#define pitch_motor_id   0x208


#endif


enum 
{
	rabbit_locked=1,
	rabbit_unlocked,
	rabbit_chassis_on,
	rabbit_chassis_off,
	rabbit_clamping_on,
	rabbit_clamping_off,
	rabbit_shoot_on,
	rabbit_shoot_off
	
};

#endif

