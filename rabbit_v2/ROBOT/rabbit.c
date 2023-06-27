#include "main.h"

#define START_TASK_PRIO	10					//优先级
#define START_STK_SIZE	128					//堆栈
static TaskHandle_t StartTask_Handler;		//句柄
void start_task(void *pvParameters);


#define CHASSIS_TASK_PRIO	25				//优先级
#define CHASSIS_STK_SIZE	128				//堆栈
static TaskHandle_t ChssisTask_Handler;	//句柄

#define SHOOT_TASK_PRIO	25					//优先级
#define SHOOT_STK_SIZE	128					//堆栈
static TaskHandle_t ShootTask_Handler;		//句柄

#define CLAMPING_TASK_PRIO	25				//优先级
#define CLAMPING_STK_SIZE	128				//堆栈
static TaskHandle_t ClampingTask_Handler;	//句柄



robot rabbit;

float chassis_pid_params[4][6]={
									{7.0f, 1.0f, 0.0f, 8000, 200, 200},
									{7.0f, 1.0f, 0.0f, 8000, 200, 200},
									{7.0f, 1.0f, 0.0f, 8000, 200, 200},
									{7.0f, 1.0f, 0.0f, 8000, 200, 200},
								};

void chassis_pid_init(CHASSIS* chassis, float params[4][6])
{
	for(int i=0;i<4;i++)
	{
		PID_init(&chassis->chassis_motor[i].pid_param,PID_POSITION,params[i][0],params[i][1],params[i][2],params[i][3],params[i][4],params[i][5]);
	}
}

void rabbit_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4	
	TIM3_Int_Init(5000-1, 8400-1);		//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5000次为500ms
	TIM5_PWM_Init(5000-1, 84 - 1);	//定时器时钟84M，分频系数84，所以84M/84=1Mhz的计数频率，计数5000次为5ms
	delay_init(168);					//初始化延时函数
	gpio_Init();
	remote_control_init();
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	
	
	
	chassis_pid_init(&rabbit.chassis, chassis_pid_params);
}

void FreeRtos_start(void)
{
	
	xTaskCreate((TaskFunction_t)start_task,			//任务函数
				(const char*   )"start_task",		//名称
				(uint16_t	   )START_STK_SIZE,		//堆栈
				(void*         )NULL,				//传递参数
				(UBaseType_t   )START_TASK_PRIO,	//优先级
				(TaskHandle_t* )&StartTask_Handler);//句柄
	vTaskStartScheduler();		//开启调度

	
}

void start_task(void *pvParameters)
{
	
	taskENTER_CRITICAL();           //进入临界区
	shoot_task_start();
	
	vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区


}

void chssis_task_start(void)
{
		xTaskCreate((TaskFunction_t)chassis_task,			//任务函数
				(const char*   )"chassis_task",			//名称
				(uint16_t	   )CHASSIS_STK_SIZE,			//堆栈
				(void*         )NULL,						//传递参数
				(UBaseType_t   )CHASSIS_TASK_PRIO,			//优先级
				(TaskHandle_t* )&ChssisTask_Handler);		//句柄

}
void shoot_task_start(void)
{
		xTaskCreate((TaskFunction_t)shoot_task,				//任务函数
				(const char*   )"shoot_task",				//名称
				(uint16_t	   )SHOOT_STK_SIZE,				//堆栈
				(void*         )NULL,						//传递参数
				(UBaseType_t   )SHOOT_TASK_PRIO,			//优先级
				(TaskHandle_t* )&ShootTask_Handler);		//句柄

}
void clamping_task_start(void)
{
		xTaskCreate((TaskFunction_t)clamping_task,			//任务函数
				(const char*   )"clamping_task",			//名称
				(uint16_t	   )CLAMPING_STK_SIZE,			//堆栈
				(void*         )NULL,						//传递参数
				(UBaseType_t   )CLAMPING_TASK_PRIO,			//优先级
				(TaskHandle_t* )&ClampingTask_Handler);		//句柄

}

