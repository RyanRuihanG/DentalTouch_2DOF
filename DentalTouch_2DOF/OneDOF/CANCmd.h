#pragma once
//����CANָ�

#include "EcanVci.h"//can��ͷ�ļ�

class CCANCmd
{
public:
	CCANCmd(void);
	~CCANCmd(void);
public:
	//��
	CAN_OBJ * GetFrame(BYTE addr, BYTE mode);//��ȡ֡������Ϊģʽ������Ϊ֡
	CAN_OBJ * GetFrame(BYTE addr, BYTE mode, DOUBLE v);//���أ���Ҫ���������dpcλ�á��ٶȺͼ��ٶ�

private:

	CAN_OBJ m_Frame;
	void Set_StartCAN(BYTE addr);
	void Set_CANMode(BYTE addr);
	void Set_OperationMode(BYTE addr);
	void Set_EnableAllAxis();
	void Set_ExecuteAllMoving();
	void Set_MotionProfileType(BYTE addr);
	void Set_EnableMotion(BYTE addr);
	void Set_EnablePDO(BYTE addr);
	void Set_StartMotion(BYTE addr);
	void Set_SetPositionZero(BYTE addr);
	void Set_Position(BYTE addr, DOUBLE v);
	void Set_Velocity(BYTE addr, DOUBLE v);
	void Set_Acceleration(BYTE addr, DOUBLE v);
	void Set_Deceleration(BYTE addr, DOUBLE v);
	
};

