#pragma once
//保存CAN指令集

#include "EcanVci.h"//can卡头文件

class CCANCmd
{
public:
	CCANCmd(void);
	~CCANCmd(void);
public:
	CAN_OBJ * GetFrame( BYTE mode);//获取帧，输入为模式，返回为帧
	CAN_OBJ * GetFrame( BYTE mode, DOUBLE v);//重载，主要是针对设置dpc位置、速度和加速度
	


private:
	CAN_OBJ m_Frame_StartCAN;	//启动CAN，GetFrame模式=0
	CAN_OBJ m_Frame_CANMode;//进入CAN模式，GetFrame模式=1
	CAN_OBJ m_Frame_OperationMode;	//操作模式，位置轨迹模式，GetFrame模式=2
	CAN_OBJ m_Frame_MotionProfileType;//梯形，GetFrame模式=3
	CAN_OBJ m_Frame_EnableMotion;//启动一个新的运动前，先Enable，每次新的运行都需要Enable一次，GetFrame模式=4
	CAN_OBJ m_Frame_StartMotion;//启动运动，为 3Fh 启动电机运行，绝对运动，参考p194，连续轨迹运行，GetFrame模式=5

	CAN_OBJ m_Frame_TargetPosition;//目标位置，GetFrame模式=6
	CAN_OBJ m_Frame_ProfileVelocity;//梯形速度，GetFrame模式=7
	CAN_OBJ m_Frame_ProfileAcceleration;////梯形加速度，GetFrame模式=8
	CAN_OBJ m_Frame_ProfileDeceleration;//减加速度，GetFrame模式=9

	CAN_OBJ m_Frame_SetPositionZero;//减加速度，GetFrame模式=10

	BOOL SetData(BYTE mode, DOUBLE dpcAngle);//输入为dpc角度，弧度单位

private:	//接收脉冲数，设置指令
	void UpdateTargetPosition(INT tp);
	void UpdateProfileVelocity(INT pv);
	void UpdateProfileAcceleration(INT pa);
	void UpdateProfileDeceleration(INT pd);

private:
	void StartCAN();	//启动CAN
	void CANMode();//进入CAN模式
	void OperationMode();	//操作模式，位置轨迹模式
	void MotionProfileType();//梯形
	void EnableMotion();//启动一个新的运动前，先Enable
	void StartMotion();//启动运动，为 3Fh 启动电机运行，绝对运动，参考p194，连续轨迹运行
	void SetPositionZero();

};

