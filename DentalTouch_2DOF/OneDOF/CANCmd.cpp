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
	#define MODE_ENABLEPDO	11
	#define	MODE_ENABLEALLAXIS 12
	#define MODE_EXECUTEALLMOVING 13
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
	//��
	m_Frame.RemoteFlag = 0;
	m_Frame.ExternFlag = 0;
	m_Frame.SendType = 0;

}


CCANCmd::~CCANCmd(void)
{
}



//��
CAN_OBJ * CCANCmd::GetFrame(BYTE addr, BYTE mode)//��ȡ֡������Ϊģʽ������Ϊ֡
{
	switch(mode)
	{
	case MODE_STARTCAN:
		Set_StartCAN(addr);
		return &m_Frame;
		break;
	case MODE_CANMODE:
		Set_CANMode(addr);
		return &m_Frame;
		break;
	case MODE_OPERATIONMODE:
		Set_OperationMode(addr);
		return &m_Frame;
		break;
	case MODE_PROFILETYPE:
		Set_MotionProfileType(addr);
		return &m_Frame;
		break;
	case MODE_ENABLEMOTION:
		Set_EnableMotion(addr);
		return &m_Frame;
		break;
	case MODE_STARTMOTION:
		Set_StartMotion(addr);
		return &m_Frame;
		break;
	case MODE_SETPOSITIONZERO:
		Set_SetPositionZero(addr);
		return &m_Frame;
		break;
	case MODE_ENABLEPDO:
		Set_EnablePDO(addr);
		return &m_Frame;
		break;
	case MODE_ENABLEALLAXIS:
		Set_EnableAllAxis();
		return &m_Frame;
		break;
	case MODE_EXECUTEALLMOVING:
		Set_ExecuteAllMoving();
		return &m_Frame;
		break;
	default:
		Set_StartCAN(addr);//��������can
		return &m_Frame;
		break;
	}
}
CAN_OBJ * CCANCmd::GetFrame(BYTE addr, BYTE mode, DOUBLE v)//���أ���Ҫ���������dpcλ�á��ٶȺͼ��ٶ�
{
	switch(mode)
	{
	case MODE_POSITION:
		Set_Position(addr, v);
		return &m_Frame;
		break;
	case MODE_VELOCITY:
		Set_Velocity(addr, v);
		return &m_Frame;
		break;
	case MODE_ACCELERATION:
		Set_Acceleration(addr, v);
		return &m_Frame;
		break;
	case MODE_DECELERATION:
		Set_Deceleration(addr, v);
		return &m_Frame;
		break;
	default:
		Set_StartCAN(addr);//��������can
		return &m_Frame;
		break;
	}
}
void CCANCmd::Set_EnableAllAxis()
{
		//֡ͷ
	m_Frame.ID = 0x0500;

	//����
	m_Frame.DataLen = 2;//��ʾ����֡ͷ�⣬���������ֽڵĳ��Ⱥ�
	m_Frame.Data[0] = 0x2F;
	m_Frame.Data[1] = 0x00;
	m_Frame.Data[2] = 0x00;
	m_Frame.Data[3] = 0x00;
	m_Frame.Data[4] = 0x00;
	m_Frame.Data[5] = 0x00;
	m_Frame.Data[6] = 0x00;
	m_Frame.Data[7] = 0x00;

}
void CCANCmd::Set_ExecuteAllMoving()
{
		//֡ͷ
	m_Frame.ID = 0x0500;

	//����
	m_Frame.DataLen = 2;//��ʾ����֡ͷ�⣬���������ֽڵĳ��Ⱥ�
	m_Frame.Data[0] = 0x3F;
	m_Frame.Data[1] = 0x00;
	m_Frame.Data[2] = 0x00;
	m_Frame.Data[3] = 0x00;
	m_Frame.Data[4] = 0x00;
	m_Frame.Data[5] = 0x00;
	m_Frame.Data[6] = 0x00;
	m_Frame.Data[7] = 0x00;

}

void CCANCmd::Set_StartCAN(BYTE addr)
{
	//֡ͷ
	m_Frame.ID = addr;

	//����
	m_Frame.DataLen = 2;//��ʾ����֡ͷ�⣬���������ֽڵĳ��Ⱥ�
	m_Frame.Data[0] = 0x01;
	m_Frame.Data[1] = 0x00;
	m_Frame.Data[2] = 0x00;
	m_Frame.Data[3] = 0x00;
	m_Frame.Data[4] = 0x00;
	m_Frame.Data[5] = 0x00;
	m_Frame.Data[6] = 0x00;
	m_Frame.Data[7] = 0x00;
}
void CCANCmd::Set_CANMode(BYTE addr)
{
	//֡ͷ
	m_Frame.ID = 0x0600 + addr;

	//����
	m_Frame.DataLen = 6;
	m_Frame.Data[0] = 0x2B;
	m_Frame.Data[1] = 0x00;
	m_Frame.Data[2] = 0x23;
	m_Frame.Data[3] = 0x00;
	m_Frame.Data[4] = 0x1E;
	m_Frame.Data[5] = 0x00;
	m_Frame.Data[6] = 0x00;
	m_Frame.Data[7] = 0x00;
}
void CCANCmd::Set_EnablePDO(BYTE addr)
{
	//֡ͷ
	m_Frame.ID = 0x0600 + addr;

	//����
	m_Frame.DataLen = 8;
	m_Frame.Data[0] = 0x23;
	m_Frame.Data[1] = 0x04;
	m_Frame.Data[2] = 0x14;
	m_Frame.Data[3] = 0x01;
	m_Frame.Data[4] = 0x00;
	m_Frame.Data[5] = 0x05;
	m_Frame.Data[6] = 0x00;
	m_Frame.Data[7] = 0x00;
}

void CCANCmd::Set_OperationMode(BYTE addr)
{
	//֡ͷ
	m_Frame.ID = 0x0600 + addr;
	//����
	m_Frame.SendType = 0;
	m_Frame.DataLen = 5;
	m_Frame.Data[0] = 0x2F;
	m_Frame.Data[1] = 0x60;
	m_Frame.Data[2] = 0x60;
	m_Frame.Data[3] = 0x00;
	m_Frame.Data[4] = 0x01;
	m_Frame.Data[5] = 0x00;
	m_Frame.Data[6] = 0x00;
	m_Frame.Data[7] = 0x00;
}
void CCANCmd::Set_MotionProfileType(BYTE addr)
{
	//֡ͷ
	m_Frame.ID = 0x0600 + addr;
	//����
	m_Frame.DataLen = 6;
	m_Frame.Data[0] = 0x2B;
	m_Frame.Data[1] = 0x86;
	m_Frame.Data[2] = 0x60;
	m_Frame.Data[3] = 0x00;
	m_Frame.Data[4] = 0x00;
	m_Frame.Data[5] = 0x00;
	m_Frame.Data[6] = 0x00;
	m_Frame.Data[7] = 0x00;
}
void CCANCmd::Set_EnableMotion(BYTE addr)
{
	//֡ͷ
	m_Frame.ID = 0x0600 + addr;

	//����
	m_Frame.DataLen = 6;

	m_Frame.Data[0] = 0x2B;
	m_Frame.Data[1] = 0x40;
	m_Frame.Data[2] = 0x60;
	m_Frame.Data[3] = 0x00;
	m_Frame.Data[4] = 0x2F;
	m_Frame.Data[5] = 0x00;
	m_Frame.Data[6] = 0x00;
	m_Frame.Data[7] = 0x00;

}
void CCANCmd::Set_StartMotion(BYTE addr)
{
	//֡ͷ
	m_Frame.ID = 0x0600 + addr;

	//����
	m_Frame.DataLen = 6;
	m_Frame.Data[0] = 0x2B;
	m_Frame.Data[1] = 0x40;
	m_Frame.Data[2] = 0x60;
	m_Frame.Data[3] = 0x00;
	m_Frame.Data[4] = 0x3F;
	m_Frame.Data[5] = 0x00;
	m_Frame.Data[6] = 0x00;
	m_Frame.Data[7] = 0x00;

}
void CCANCmd::Set_SetPositionZero(BYTE addr)
{

	//֡ͷ
	m_Frame.ID = 0x0600 + addr;

	m_Frame.DataLen = 8;
	m_Frame.Data[0] = 0x23; 
	m_Frame.Data[1] = 0x63; 
	m_Frame.Data[2] = 0x60; 
	m_Frame.Data[3] = 0x00;
	m_Frame.Data[4] = 0x00; 
	m_Frame.Data[5] = 0x00;
	m_Frame.Data[6] = 0x00; 
	m_Frame.Data[7] = 0x00;

}
void CCANCmd::Set_Position(BYTE addr, DOUBLE v)
{
	INT counts = (INT) (v * MOTOR_RATIO / (2.0f*PI)  * MOTOR_ENCODER_COUNTS);//�ɻ���ת�������壬ָ��ȳ��Լ��ٱ�Ϊ������ȣ�����2PI��һȦ�ķֱ���

	//֡ͷ
	m_Frame.ID = 0x0600 + addr;

	m_Frame.DataLen = 8;
	m_Frame.Data[0] = 0x23;
	m_Frame.Data[1] = 0x7A;
	m_Frame.Data[2] = 0x60;
	m_Frame.Data[3] = 0x00;
	m_Frame.Data[4] = (BYTE) (0x000000ff & counts);	//���ֽڣ����Խ����ȷ
	m_Frame.Data[5] = (BYTE) ((0x0000ff00 & counts) >> 8);
	m_Frame.Data[6] = (BYTE) ((0x00ff0000 & counts) >> 16);
	m_Frame.Data[7] = (BYTE) ((0xff000000 & counts) >> 24);//���ֽ�
}
void CCANCmd::Set_Velocity(BYTE addr, DOUBLE v)
{
	INT counts = (INT) (10.0*(v * MOTOR_RATIO / (2.0f*PI)  * MOTOR_ENCODER_COUNTS));//�ٶ�����1000 counts/sʵ����100counts/s��

	//֡ͷ
	m_Frame.ID = 0x0600 + addr;

	m_Frame.DataLen = 8;
	m_Frame.Data[0] = 0x23;
	m_Frame.Data[1] = 0x81; 
	m_Frame.Data[2] = 0x60; 
	m_Frame.Data[3] = 0x00;
	m_Frame.Data[4] = (BYTE) (0x000000ff & counts);	//���ֽڣ����Խ����ȷ
	m_Frame.Data[5] = (BYTE) ((0x0000ff00 & counts) >> 8);
	m_Frame.Data[6] = (BYTE) ((0x00ff0000 & counts) >> 16);
	m_Frame.Data[7] = (BYTE) ((0xff000000 & counts) >> 24);//���ֽ�
}
void CCANCmd::Set_Acceleration(BYTE addr, DOUBLE v)
{
	INT counts = (INT) (0.1*(v * MOTOR_RATIO / (2.0f*PI)  * MOTOR_ENCODER_COUNTS));//���ٶ�����10 counts/s2ʵ����100 counts/s��

	//֡ͷ
	m_Frame.ID = 0x0600 + addr;

	m_Frame.DataLen = 8;
	m_Frame.Data[0] = 0x23; 
	m_Frame.Data[1] = 0x83; 
	m_Frame.Data[2] = 0x60; 
	m_Frame.Data[3] = 0x00;
	m_Frame.Data[4] = (BYTE) (0x000000ff & counts);	//���ֽڣ����Խ����ȷ
	m_Frame.Data[5] = (BYTE) ((0x0000ff00 & counts) >> 8);
	m_Frame.Data[6] = (BYTE) ((0x00ff0000 & counts) >> 16);
	m_Frame.Data[7] = (BYTE) ((0xff000000 & counts) >> 24);//���ֽ�
}
void CCANCmd::Set_Deceleration(BYTE addr, DOUBLE v)
{
	INT counts = (INT) (0.1*(v * MOTOR_RATIO / (2.0f*PI)  * MOTOR_ENCODER_COUNTS));//���ٶ�����10 counts/s2ʵ����100 counts/s��
	
	//֡ͷ
	m_Frame.ID = 0x0600 + addr;

	m_Frame.DataLen = 8;
	m_Frame.Data[0] = 0x23; 
	m_Frame.Data[1] = 0x84; 
	m_Frame.Data[2] = 0x60; 
	m_Frame.Data[3] = 0x00;
	m_Frame.Data[4] = (BYTE) (0x000000ff & counts);	//���ֽڣ����Խ����ȷ 
	m_Frame.Data[5] = (BYTE) ((0x0000ff00 & counts) >> 8); 
	m_Frame.Data[6] = (BYTE) ((0x00ff0000 & counts) >> 16);
	m_Frame.Data[7] = (BYTE) ((0xff000000 & counts) >> 24);//���ֽ�
}





