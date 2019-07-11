#pragma once
//����CANָ�

#include "EcanVci.h"//can��ͷ�ļ�

class CCANCmd
{
public:
	CCANCmd(void);
	~CCANCmd(void);
public:
	CAN_OBJ * GetFrame( BYTE mode);//��ȡ֡������Ϊģʽ������Ϊ֡
	CAN_OBJ * GetFrame( BYTE mode, DOUBLE v);//���أ���Ҫ���������dpcλ�á��ٶȺͼ��ٶ�
	


private:
	CAN_OBJ m_Frame_StartCAN;	//����CAN��GetFrameģʽ=0
	CAN_OBJ m_Frame_CANMode;//����CANģʽ��GetFrameģʽ=1
	CAN_OBJ m_Frame_OperationMode;	//����ģʽ��λ�ù켣ģʽ��GetFrameģʽ=2
	CAN_OBJ m_Frame_MotionProfileType;//���Σ�GetFrameģʽ=3
	CAN_OBJ m_Frame_EnableMotion;//����һ���µ��˶�ǰ����Enable��ÿ���µ����ж���ҪEnableһ�Σ�GetFrameģʽ=4
	CAN_OBJ m_Frame_StartMotion;//�����˶���Ϊ 3Fh ����������У������˶����ο�p194�������켣���У�GetFrameģʽ=5

	CAN_OBJ m_Frame_TargetPosition;//Ŀ��λ�ã�GetFrameģʽ=6
	CAN_OBJ m_Frame_ProfileVelocity;//�����ٶȣ�GetFrameģʽ=7
	CAN_OBJ m_Frame_ProfileAcceleration;////���μ��ٶȣ�GetFrameģʽ=8
	CAN_OBJ m_Frame_ProfileDeceleration;//�����ٶȣ�GetFrameģʽ=9

	CAN_OBJ m_Frame_SetPositionZero;//�����ٶȣ�GetFrameģʽ=10

	BOOL SetData(BYTE mode, DOUBLE dpcAngle);//����Ϊdpc�Ƕȣ����ȵ�λ

private:	//����������������ָ��
	void UpdateTargetPosition(INT tp);
	void UpdateProfileVelocity(INT pv);
	void UpdateProfileAcceleration(INT pa);
	void UpdateProfileDeceleration(INT pd);

private:
	void StartCAN();	//����CAN
	void CANMode();//����CANģʽ
	void OperationMode();	//����ģʽ��λ�ù켣ģʽ
	void MotionProfileType();//����
	void EnableMotion();//����һ���µ��˶�ǰ����Enable
	void StartMotion();//�����˶���Ϊ 3Fh ����������У������˶����ο�p194�������켣����
	void SetPositionZero();

};

