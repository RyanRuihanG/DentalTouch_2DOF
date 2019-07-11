#include "stdafx.h"
#include "CANCmd.h"


#ifndef _CANMODE
#define _CANMODE
	#define MODE_STARTCAN 0
	#define MODE_CANMODE 1
	#define MODE_OPERATIONMODE 2
	#define MODE_PROFILETYPE 3
	#define MODE_ENABLEMOTION 4
	#define MODE_STARTMOTION 5
	#define MODE_POSITION 6
	#define MODE_VELOCITY 7
	#define MODE_ACCELERATION 8
	#define MODE_DECELERATION 9
	#define	MODE_SETPOSITIONZERO 10 
#endif

#ifndef _PI
#define	_PI
	#define PI 3.141592654f//Բ����
#endif

#ifndef _MOTOR_RATIO
#define	_MOTOR_RATIO
	#define MOTOR_RATIO 50.0f	//���ٱ�
#endif

#ifndef _MOTOR_ENCODER_COUNTS
#define	_MOTOR_ENCODER_COUNTS
	#define MOTOR_ENCODER_COUNTS 10000.0f	//������������������ı�Ƶ��
#endif

CCANCmd::CCANCmd(void)
{
	StartCAN();	//����CAN
	CANMode();//����CANģʽ
	OperationMode();	//����ģʽ��λ�ù켣ģʽ
	MotionProfileType();//����
	EnableMotion();//����һ���µ��˶�ǰ����Enable
	StartMotion();//�����˶���Ϊ 3Fh ����������У������˶����ο�p194�������켣����
	SetPositionZero();


	m_Frame_TargetPosition.RemoteFlag = 0;//Ŀ��λ��
	m_Frame_TargetPosition.ExternFlag = 0;
	m_Frame_TargetPosition.ID = 0x0601;
	m_Frame_TargetPosition.SendType = 0;
	m_Frame_TargetPosition.DataLen = 8;
	m_Frame_TargetPosition.Data[0] = 0x23;
	m_Frame_TargetPosition.Data[1] = 0x7A;
	m_Frame_TargetPosition.Data[2] = 0x60;
	m_Frame_TargetPosition.Data[3] = 0x00;


	m_Frame_ProfileVelocity.RemoteFlag = 0;
	m_Frame_ProfileVelocity.ExternFlag = 0;
	m_Frame_ProfileVelocity.ID = 0x0601;
	m_Frame_ProfileVelocity.SendType = 0;
	m_Frame_ProfileVelocity.DataLen = 8;
	m_Frame_ProfileVelocity.Data[0] = 0x23;
	m_Frame_ProfileVelocity.Data[1] = 0x81; 
	m_Frame_ProfileVelocity.Data[2] = 0x60; 
	m_Frame_ProfileVelocity.Data[3] = 0x00;


	m_Frame_ProfileAcceleration.RemoteFlag = 0;
	m_Frame_ProfileAcceleration.ExternFlag = 0;
	m_Frame_ProfileAcceleration.ID = 0x0601;
	m_Frame_ProfileAcceleration.SendType = 0;
	m_Frame_ProfileAcceleration.DataLen = 8;
	m_Frame_ProfileAcceleration.Data[0] = 0x23; 
	m_Frame_ProfileAcceleration.Data[1] = 0x83; 
	m_Frame_ProfileAcceleration.Data[2] = 0x60; 
	m_Frame_ProfileAcceleration.Data[3] = 0x00;

	m_Frame_ProfileDeceleration.RemoteFlag = 0;
	m_Frame_ProfileDeceleration.ExternFlag = 0;
	m_Frame_ProfileDeceleration.ID = 0x0601;
	m_Frame_ProfileDeceleration.SendType = 0;
	m_Frame_ProfileDeceleration.DataLen = 8;
	m_Frame_ProfileDeceleration.Data[0] = 0x23; 
	m_Frame_ProfileDeceleration.Data[1] = 0x84; 
	m_Frame_ProfileDeceleration.Data[2] = 0x60; 
	m_Frame_ProfileDeceleration.Data[3] = 0x00;




}


CCANCmd::~CCANCmd(void)
{
}


CAN_OBJ * CCANCmd::GetFrame(BYTE mode)//��ȡ֡������Ϊģʽ
{
	switch(mode)
	{
	case MODE_STARTCAN:
		return &m_Frame_StartCAN;
		break;
	case MODE_CANMODE:
		return &m_Frame_CANMode;
		break;
	case MODE_OPERATIONMODE:
		return &m_Frame_OperationMode;
		break;
	case MODE_PROFILETYPE:
		return &m_Frame_MotionProfileType;
		break;
	case MODE_ENABLEMOTION:
		return &m_Frame_EnableMotion;
		break;
	case MODE_STARTMOTION:
		return &m_Frame_StartMotion;
		break;
	case MODE_SETPOSITIONZERO:
		return &m_Frame_SetPositionZero;
		break;
	default:
		return &m_Frame_StartCAN;//��������can
		break;
	}
}


CAN_OBJ * CCANCmd::GetFrame(BYTE mode, DOUBLE v)//��ȡ֡������Ϊģʽ
{
	switch(mode)
	{
	case MODE_POSITION:
		SetData(MODE_POSITION,v);
		return &m_Frame_TargetPosition;
		break;
	case MODE_VELOCITY:
		SetData(MODE_VELOCITY,v);
		return &m_Frame_ProfileVelocity;
		break;
	case MODE_ACCELERATION:
		SetData(MODE_ACCELERATION,v);
		return &m_Frame_ProfileAcceleration;
		break;
	case MODE_DECELERATION:
		SetData(MODE_DECELERATION,v);
		return &m_Frame_ProfileDeceleration;
		break;
	default:
		return &m_Frame_StartCAN;//��������can
		break;
	}
}


BOOL CCANCmd::SetData(BYTE mode, DOUBLE data)//������λ��Ϊrad��ת��Ϊrad/s���Ӽ��ٶ�Ϊrad/s2
{
	INT counts;
	switch(mode)
	{
	case MODE_POSITION:
		counts = (INT) (data * MOTOR_RATIO / (2.0f*PI)  * MOTOR_ENCODER_COUNTS);//�ɻ���ת�������壬ָ��ȳ��Լ��ٱ�Ϊ������ȣ�����2PI��һȦ�ķֱ���
		UpdateTargetPosition(counts);
		return TRUE;
		break;
	case MODE_VELOCITY:
		counts = (INT) (10.0*(data * MOTOR_RATIO / (2.0f*PI)  * MOTOR_ENCODER_COUNTS));//�ٶ�����1000 counts/sʵ����100counts/s��
		UpdateProfileVelocity(counts);
		return TRUE;
		break;
	case MODE_ACCELERATION:
		counts = (INT) (0.1*(data * MOTOR_RATIO / (2.0f*PI)  * MOTOR_ENCODER_COUNTS));//���ٶ�����10 counts/s2ʵ����100 counts/s��
		UpdateProfileAcceleration(counts);
		return TRUE;
		break;
	case MODE_DECELERATION:
		counts = (INT) (0.1*(data * MOTOR_RATIO / (2.0f*PI)  * MOTOR_ENCODER_COUNTS));//���ٶ�����10 counts/s2ʵ����100 counts/s��
		UpdateProfileDeceleration(counts);
		return TRUE;
		break;
	default:
		return FALSE;
		break;
	}

	
}

void CCANCmd::UpdateTargetPosition(INT tp)
{
	m_Frame_TargetPosition.Data[4] = (BYTE) (0x000000ff & tp);	//���ֽڣ����Խ����ȷ
	m_Frame_TargetPosition.Data[5] = (BYTE) ((0x0000ff00 & tp) >> 8);
	m_Frame_TargetPosition.Data[6] = (BYTE) ((0x00ff0000 & tp) >> 16);
	m_Frame_TargetPosition.Data[7] = (BYTE) ((0xff000000 & tp) >> 24);//���ֽ�

}
void CCANCmd::UpdateProfileVelocity(INT pv)
{
	m_Frame_ProfileVelocity.Data[4] = (BYTE) (0x000000ff & pv);	//���ֽڣ����Խ����ȷ
	m_Frame_ProfileVelocity.Data[5] = (BYTE) ((0x0000ff00 & pv) >> 8);
	m_Frame_ProfileVelocity.Data[6] = (BYTE) ((0x00ff0000 & pv) >> 16);
	m_Frame_ProfileVelocity.Data[7] = (BYTE) ((0xff000000 & pv) >> 24);//���ֽ�
}
void CCANCmd::UpdateProfileAcceleration(INT pa)
{
	m_Frame_ProfileAcceleration.Data[4] = (BYTE) (0x000000ff & pa);	//���ֽڣ����Խ����ȷ
	m_Frame_ProfileAcceleration.Data[5] = (BYTE) ((0x0000ff00 & pa) >> 8);
	m_Frame_ProfileAcceleration.Data[6] = (BYTE) ((0x00ff0000 & pa) >> 16);
	m_Frame_ProfileAcceleration.Data[7] = (BYTE) ((0xff000000 & pa) >> 24);//���ֽ�
}
void CCANCmd::UpdateProfileDeceleration(INT pd)
{
	m_Frame_ProfileDeceleration.Data[4] = (BYTE) (0x000000ff & pd);	//���ֽڣ����Խ����ȷ 
	m_Frame_ProfileDeceleration.Data[5] = (BYTE) ((0x0000ff00 & pd) >> 8); 
	m_Frame_ProfileDeceleration.Data[6] = (BYTE) ((0x00ff0000 & pd) >> 16);
	m_Frame_ProfileDeceleration.Data[7] = (BYTE) ((0xff000000 & pd) >> 24);//���ֽ�
}


void CCANCmd::StartCAN()	//
{
	//ģʽ
	m_Frame_StartCAN.RemoteFlag = 0;
	m_Frame_StartCAN.ExternFlag = 0;

	//֡ͷ
	m_Frame_StartCAN.ID = 0;

	//����
	m_Frame_StartCAN.SendType = 0;
	m_Frame_StartCAN.DataLen = 2;//��ʾ����֡ͷ�⣬���������ֽڵĳ��Ⱥ�
	m_Frame_StartCAN.Data[0] = 0x01;
	m_Frame_StartCAN.Data[1] = 0x00;

}
void CCANCmd::CANMode()	//
{
	//ģʽ
	m_Frame_CANMode.RemoteFlag = 0;
	m_Frame_CANMode.ExternFlag = 0;

	//֡ͷ
	m_Frame_CANMode.ID = 0x0601;

	//����
	m_Frame_CANMode.SendType = 0;
	m_Frame_CANMode.DataLen = 6;

	m_Frame_CANMode.Data[0] = 0x2B;
	m_Frame_CANMode.Data[1] = 0x00;
	m_Frame_CANMode.Data[2] = 0x23;
	m_Frame_CANMode.Data[3] = 0x00;
	m_Frame_CANMode.Data[4] = 0x1E;
	m_Frame_CANMode.Data[5] = 0x00;

}
void CCANCmd::OperationMode()	//
{
	//ģʽ
	m_Frame_OperationMode.RemoteFlag = 0;
	m_Frame_OperationMode.ExternFlag = 0;

	//֡ͷ
	m_Frame_OperationMode.ID = 0x0601;

	//����
	m_Frame_OperationMode.SendType = 0;

	m_Frame_OperationMode.DataLen = 5;

	m_Frame_OperationMode.Data[0] = 0x2F;
	m_Frame_OperationMode.Data[1] = 0x60;
	m_Frame_OperationMode.Data[2] = 0x60;
	m_Frame_OperationMode.Data[3] = 0x00;
	m_Frame_OperationMode.Data[4] = 0x01;


}
void CCANCmd::MotionProfileType()	//
{
		//ģʽ
	m_Frame_MotionProfileType.RemoteFlag = 0;
	m_Frame_MotionProfileType.ExternFlag = 0;

	//֡ͷ
	m_Frame_MotionProfileType.ID = 0x0601;

	//����
	m_Frame_MotionProfileType.SendType = 0;
	m_Frame_MotionProfileType.DataLen = 6;

	m_Frame_MotionProfileType.Data[0] = 0x2B;
	m_Frame_MotionProfileType.Data[1] = 0x86;
	m_Frame_MotionProfileType.Data[2] = 0x60;
	m_Frame_MotionProfileType.Data[3] = 0x00;
	m_Frame_MotionProfileType.Data[4] = 0x00;
	m_Frame_MotionProfileType.Data[5] = 0x00;

}
void CCANCmd::EnableMotion()	//
{
		//ģʽ
	m_Frame_EnableMotion.RemoteFlag = 0;
	m_Frame_EnableMotion.ExternFlag = 0;

	//֡ͷ
	m_Frame_EnableMotion.ID = 0x0601;

	//����
	m_Frame_EnableMotion.SendType = 0;

	m_Frame_EnableMotion.DataLen = 6;

	m_Frame_EnableMotion.Data[0] = 0x2B;
	m_Frame_EnableMotion.Data[1] = 0x40;
	m_Frame_EnableMotion.Data[2] = 0x60;
	m_Frame_EnableMotion.Data[3] = 0x00;
	m_Frame_EnableMotion.Data[4] = 0x2F;
	m_Frame_EnableMotion.Data[5] = 0x00;

}
void CCANCmd::StartMotion()	//
{
	//ģʽ
	m_Frame_StartMotion.RemoteFlag = 0;
	m_Frame_StartMotion.ExternFlag = 0;

	//֡ͷ
	m_Frame_StartMotion.ID = 0x0601;

	//����
	m_Frame_StartMotion.SendType = 0;

	m_Frame_StartMotion.DataLen = 6;

	m_Frame_StartMotion.Data[0] = 0x2B;
	m_Frame_StartMotion.Data[1] = 0x40;
	m_Frame_StartMotion.Data[2] = 0x60;
	m_Frame_StartMotion.Data[3] = 0x00;
	m_Frame_StartMotion.Data[4] = 0x3F;
	m_Frame_StartMotion.Data[5] = 0x00;
}
void CCANCmd::SetPositionZero()	//
{
	m_Frame_SetPositionZero.RemoteFlag = 0;
	m_Frame_SetPositionZero.ExternFlag = 0;
	m_Frame_SetPositionZero.ID = 0x0601;
	m_Frame_SetPositionZero.SendType = 0;
	m_Frame_SetPositionZero.DataLen = 8;
	m_Frame_SetPositionZero.Data[0] = 0x23; 
	m_Frame_SetPositionZero.Data[1] = 0x63; 
	m_Frame_SetPositionZero.Data[2] = 0x60; 
	m_Frame_SetPositionZero.Data[3] = 0x00;
	m_Frame_SetPositionZero.Data[4] = 0x00; 
	m_Frame_SetPositionZero.Data[5] = 0x00;
	m_Frame_SetPositionZero.Data[6] = 0x00; 
	m_Frame_SetPositionZero.Data[7] = 0x00;
}