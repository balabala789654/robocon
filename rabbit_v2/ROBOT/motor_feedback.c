//#include "motor_feedback.h"
#include "main.h"

void motor_driver_can1(uint16_t stid, int i1,int i2,int i3,int i4)
{
	CanTxMsg TxMessage;
	
	TxMessage.DLC=8;
	TxMessage.IDE=CAN_Id_Standard;
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.StdId=stid;
	
	TxMessage.Data[0]=i1 >> 8;
	TxMessage.Data[1]=i1;
	TxMessage.Data[2]=i2 >> 8;
	TxMessage.Data[3]=i2;
	TxMessage.Data[4]=i3 >> 8;
	TxMessage.Data[5]=i3;
	TxMessage.Data[6]=i4 >> 8;
	TxMessage.Data[7]=i4; 
	
	CAN_Transmit(CAN1,&TxMessage);
}

void motor_driver_can2(uint16_t stid, int i1,int i2,int i3,int i4)
{
	
	CanTxMsg TxMessage;	
	TxMessage.DLC=8;
	TxMessage.IDE=CAN_Id_Standard;
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.StdId=stid;
	
	TxMessage.Data[0]=i1 >> 8;
	TxMessage.Data[1]=i1;
	TxMessage.Data[2]=i2 >> 8;
	TxMessage.Data[3]=i2;
	TxMessage.Data[4]=i3 >> 8;
	TxMessage.Data[5]=i3;
	TxMessage.Data[6]=i4 >> 8;
	TxMessage.Data[7]=i4; 
	
	CAN_Transmit(CAN2,&TxMessage);

}

CanRxMsg Rx1Message;
void CAN1_RX0_IRQHandler(void)
{
	CAN_Receive(CAN1,CAN_FIFO0,&Rx1Message);
	switch(Rx1Message.StdId)
	{
		case 0x201: get_motor_measure(&rabbit.chassis.chassis_motor[0].feedback, Rx1Message);break;
		case 0x202: get_motor_measure(&rabbit.chassis.chassis_motor[1].feedback, Rx1Message);break;
		case 0x203: get_motor_measure(&rabbit.chassis.chassis_motor[2].feedback, Rx1Message);break;
		case 0x204: get_motor_measure(&rabbit.chassis.chassis_motor[3].feedback, Rx1Message);break;
	}

}

CanRxMsg Rx2Message;
void CAN2_RX1_IRQHandler(void)
{
	CAN_Receive(CAN2,CAN_FIFO1,&Rx2Message);
	switch(Rx2Message.StdId)
	{
		case 0x201: get_motor_measure(&rabbit.shoot.motor_2006[0].feedback, Rx2Message);break;
		case 0x202: get_motor_measure(&rabbit.shoot.motor_2006[1].feedback, Rx2Message);break;
		case 0x203: get_motor_measure(&rabbit.shoot.motor_2006[2].feedback, Rx2Message);break;
	}

}
