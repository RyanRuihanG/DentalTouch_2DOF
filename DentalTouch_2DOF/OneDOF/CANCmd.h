#pragma once
//保存CAN指令集

#include "EcanVci.h"//can卡头文件

class CCANCmd
{
public:
	CCANCmd(void);
	~CCANCmd(void);
public:
	//新
	CAN_OBJ * GetFrame(BYTE addr, BYTE mode);//获取帧，输入为模式，返回为帧
	CAN_OBJ * GetFrame(BYTE addr, BYTE mode, DOUBLE v);//重载，主要是针对设置dpc位置、速度和加速度

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

