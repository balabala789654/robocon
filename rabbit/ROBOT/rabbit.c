#include "main.h"

#define START_TASK_PRIO	10					//���ȼ�
#define START_STK_SIZE	128					//��ջ
static TaskHandle_t StartTask_Handler;		//���
void start_task(void *pvParameters);


#define CHASSIS_TASK_PRIO	25				//���ȼ�
#define CHASSIS_STK_SIZE	128				//��ջ
static TaskHandle_t ChssisTask_Handler;	//���

#define SHOOT_TASK_PRIO	25					//���ȼ�
#define SHOOT_STK_SIZE	128					//��ջ
static TaskHandle_t ShootTask_Handler;		//���

#define CLAMPING_TASK_PRIO	25				//���ȼ�
#define CLAMPING_STK_SIZE	128				//��ջ
static TaskHandle_t ClampingTask_Handler;	//���

#define FUNCTION_TASK_PRIO	30				//���ȼ�
#define FUNCTION_STK_SIZE	128				//��ջ
static TaskHandle_t FunctionTask_Handler;	//���


//����
robot rabbit;


void rabbit_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	//����ϵͳ�ж����ȼ�����4	
	TIM3_Int_Init(5000-1, 8400-1);					//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����5000��Ϊ500ms
	TIM5_PWM_Init(5000-1, 84 - 1);					//��ʱ��ʱ��84M����Ƶϵ��84������84M/84=1Mhz�ļ���Ƶ�ʣ�����5000��Ϊ5ms
	delay_init(168);								//��ʼ����ʱ����
	gpio_Init();
	remote_control_init();
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
}

void FreeRtos_start(void)
{
	
	xTaskCreate((TaskFunction_t)start_task,			//������
				(const char*   )"start_task",		//����
				(uint16_t	   )START_STK_SIZE,		//��ջ
				(void*         )NULL,				//���ݲ���
				(UBaseType_t   )START_TASK_PRIO,	//���ȼ�
				(TaskHandle_t* )&StartTask_Handler);//���
	vTaskStartScheduler();		//��������

	
}

void start_task(void *pvParameters)
{
	
	taskENTER_CRITICAL();           //�����ٽ���
	
	function_task_start();

	//chssis_task_start();
	shoot_task_start();
	//clamping_task_start();
	
	
	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���


}

void chssis_task_start(void)
{
	xTaskCreate((TaskFunction_t)chassis_task,				//������
				(const char*   )"chassis_task",			//����
				(uint16_t	   )CHASSIS_STK_SIZE,			//��ջ
				(void*         )NULL,						//���ݲ���
				(UBaseType_t   )CHASSIS_TASK_PRIO,			//���ȼ�
				(TaskHandle_t* )&ChssisTask_Handler);		//���

}
void shoot_task_start(void)
{
	xTaskCreate((TaskFunction_t)shoot_task,					//������
				(const char*   )"shoot_task",				//����
				(uint16_t	   )SHOOT_STK_SIZE,				//��ջ
				(void*         )NULL,						//���ݲ���
				(UBaseType_t   )SHOOT_TASK_PRIO,			//���ȼ�
				(TaskHandle_t* )&ShootTask_Handler);		//���

}
void clamping_task_start(void)
{
	xTaskCreate((TaskFunction_t)clamping_task,				//������
				(const char*   )"clamping_task",			//����
				(uint16_t	   )CLAMPING_STK_SIZE,			//��ջ
				(void*         )NULL,						//���ݲ���
				(UBaseType_t   )CLAMPING_TASK_PRIO,			//���ȼ�
				(TaskHandle_t* )&ClampingTask_Handler);		//���

}
void function_task_start(void)
{
	xTaskCreate((TaskFunction_t)function_task,				//������
				(const char*   )"function_task",			//����
				(uint16_t	   )FUNCTION_STK_SIZE,			//��ջ
				(void*         )NULL,						//���ݲ���
				(UBaseType_t   )FUNCTION_TASK_PRIO,			//���ȼ�
				(TaskHandle_t* )&FunctionTask_Handler);		//���

}
