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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	
	TIM3_Int_Init(5000-1, 8400-1);		//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����5000��Ϊ500ms
	TIM5_PWM_Init(5000-1, 84 - 1);	//��ʱ��ʱ��84M����Ƶϵ��84������84M/84=1Mhz�ļ���Ƶ�ʣ�����5000��Ϊ5ms
	delay_init(168);					//��ʼ����ʱ����
	gpio_Init();
	remote_control_init();
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	
	
	
	chassis_pid_init(&rabbit.chassis, chassis_pid_params);
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
	shoot_task_start();
	
	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���


}

void chssis_task_start(void)
{
		xTaskCreate((TaskFunction_t)chassis_task,			//������
				(const char*   )"chassis_task",			//����
				(uint16_t	   )CHASSIS_STK_SIZE,			//��ջ
				(void*         )NULL,						//���ݲ���
				(UBaseType_t   )CHASSIS_TASK_PRIO,			//���ȼ�
				(TaskHandle_t* )&ChssisTask_Handler);		//���

}
void shoot_task_start(void)
{
		xTaskCreate((TaskFunction_t)shoot_task,				//������
				(const char*   )"shoot_task",				//����
				(uint16_t	   )SHOOT_STK_SIZE,				//��ջ
				(void*         )NULL,						//���ݲ���
				(UBaseType_t   )SHOOT_TASK_PRIO,			//���ȼ�
				(TaskHandle_t* )&ShootTask_Handler);		//���

}
void clamping_task_start(void)
{
		xTaskCreate((TaskFunction_t)clamping_task,			//������
				(const char*   )"clamping_task",			//����
				(uint16_t	   )CLAMPING_STK_SIZE,			//��ջ
				(void*         )NULL,						//���ݲ���
				(UBaseType_t   )CLAMPING_TASK_PRIO,			//���ȼ�
				(TaskHandle_t* )&ClampingTask_Handler);		//���

}

