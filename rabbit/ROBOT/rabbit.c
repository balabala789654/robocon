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

#define FUNCTION_TASK_PRIO	30				//优先级
#define FUNCTION_STK_SIZE	128				//堆栈
static TaskHandle_t FunctionTask_Handler;	//句柄


//兔子
robot rabbit;


void rabbit_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	//设置系统中断优先级分组4	
	TIM3_Int_Init(5000-1, 8400-1);					//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5000次为500ms
	TIM5_PWM_Init(5000-1, 84 - 1);					//定时器时钟84M，分频系数84，所以84M/84=1Mhz的计数频率，计数5000次为5ms
	delay_init(168);								//初始化延时函数
	gpio_Init();
	remote_control_init();
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
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
	
	function_task_start();

	//chssis_task_start();
	shoot_task_start();
	//clamping_task_start();
	
	
	vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区


}

void chssis_task_start(void)
{
	xTaskCreate((TaskFunction_t)chassis_task,				//任务函数
				(const char*   )"chassis_task",			//名称
				(uint16_t	   )CHASSIS_STK_SIZE,			//堆栈
				(void*         )NULL,						//传递参数
				(UBaseType_t   )CHASSIS_TASK_PRIO,			//优先级
				(TaskHandle_t* )&ChssisTask_Handler);		//句柄

}
void shoot_task_start(void)
{
	xTaskCreate((TaskFunction_t)shoot_task,					//任务函数
				(const char*   )"shoot_task",				//名称
				(uint16_t	   )SHOOT_STK_SIZE,				//堆栈
				(void*         )NULL,						//传递参数
				(UBaseType_t   )SHOOT_TASK_PRIO,			//优先级
				(TaskHandle_t* )&ShootTask_Handler);		//句柄

}
void clamping_task_start(void)
{
	xTaskCreate((TaskFunction_t)clamping_task,				//任务函数
				(const char*   )"clamping_task",			//名称
				(uint16_t	   )CLAMPING_STK_SIZE,			//堆栈
				(void*         )NULL,						//传递参数
				(UBaseType_t   )CLAMPING_TASK_PRIO,			//优先级
				(TaskHandle_t* )&ClampingTask_Handler);		//句柄

}
void function_task_start(void)
{
	xTaskCreate((TaskFunction_t)function_task,				//任务函数
				(const char*   )"function_task",			//名称
				(uint16_t	   )FUNCTION_STK_SIZE,			//堆栈
				(void*         )NULL,						//传递参数
				(UBaseType_t   )FUNCTION_TASK_PRIO,			//优先级
				(TaskHandle_t* )&FunctionTask_Handler);		//句柄

}
