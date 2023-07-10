#include "rabbit.h"
#include "status.h"

#define START_TASK_PRIO	1					//���ȼ�
#define START_STK_SIZE	128					//��ջ
static TaskHandle_t Start_Task_Handler;		//���
void start_task(void *pvParameters);

#define CHASSIS_TASK_PRIO	25				//���ȼ�
#define CHASSIS_STK_SIZE	128				//��ջ
static TaskHandle_t Chssis_Task_Handler;	//���

#define SHOOT_TASK_PRIO	25					//���ȼ�
#define SHOOT_STK_SIZE	128					//��ջ
static TaskHandle_t Shoot_Task_Handler;		//���

#define PICK_UP_TASK_PRIO	25				//���ȼ�
#define PICK_UP_STK_SIZE	128				//��ջ
static TaskHandle_t Pick_up_Task_Handler;	//���

#define STATUS_TASK_PRIO	20				//���ȼ�
#define STATUS_STK_SIZE		64			//��ջ
static TaskHandle_t Status_Task_Handler;	//���

void chssis_task_start(void);
void shoot_task_start(void);
void pick_up_task_start(void);


RABBIT rabbit;

void start_task(void *pvParameters)
{

    taskENTER_CRITICAL();           //�����ٽ���

    chssis_task_start();
    shoot_task_start();
    //pick_up_task_start();
	status_task_start();
	
    vTaskDelete(Start_Task_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���


}

void tasks_start(void)
{

    xTaskCreate((TaskFunction_t)start_task,			//������
                (const char*   )"start_task",		//����
                (uint16_t	   )START_STK_SIZE,		//��ջ
                (void*         )NULL,				//���ݲ���
                (UBaseType_t   )START_TASK_PRIO,	//���ȼ�
                (TaskHandle_t* )&Start_Task_Handler);//���
    vTaskStartScheduler();		//��������


}

void chssis_task_start(void)
{
    xTaskCreate((TaskFunction_t)chassis_task,				//������
                (const char*   )"chassis_task",			//����
                (uint16_t	   )CHASSIS_STK_SIZE,			//��ջ
                (void*         )NULL,						//���ݲ���
                (UBaseType_t   )CHASSIS_TASK_PRIO,			//���ȼ�
                (TaskHandle_t* )&Chssis_Task_Handler);		//���

}
void shoot_task_start(void)
{
    xTaskCreate((TaskFunction_t)shoot_task,					//������
                (const char*   )"shoot_task",				//����
                (uint16_t	   )SHOOT_STK_SIZE,				//��ջ
                (void*         )NULL,						//���ݲ���
                (UBaseType_t   )SHOOT_TASK_PRIO,			//���ȼ�
                (TaskHandle_t* )&Shoot_Task_Handler);		//���

}
void pick_up_task_start(void)
{
    xTaskCreate((TaskFunction_t)pick_up_task,				//������
                (const char*   )"pick_up_task",			//����
                (uint16_t	   )PICK_UP_STK_SIZE,			//��ջ
                (void*         )NULL,						//���ݲ���
                (UBaseType_t   )PICK_UP_TASK_PRIO,			//���ȼ�
                (TaskHandle_t* )&Pick_up_Task_Handler);		//���

}

void status_task_start(void)
{
    xTaskCreate((TaskFunction_t)status_task,				//������
                (const char*   )"status_task",			//����
                (uint16_t	   )STATUS_STK_SIZE,			//��ջ
                (void*         )NULL,						//���ݲ���
                (UBaseType_t   )STATUS_TASK_PRIO,			//���ȼ�
                (TaskHandle_t* )&Status_Task_Handler);		//���

}




void system_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	//����ϵͳ�ж����ȼ�����4	
	TIM3_Int_Init(5000-1, 8400-1);					//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����5000��Ϊ500ms
	TIM5_PWM_Init(5000-1, 84 - 1);					//��ʱ��ʱ��84M����Ƶϵ��84������84M/84=1Mhz�ļ���Ƶ�ʣ�����5000��Ϊ5ms

	delay_init(168);								//��ʼ����ʱ����
	usart1_init(115200);
	gpio_Init();
	remote_control_init();
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);

	
    tasks_start();
}


