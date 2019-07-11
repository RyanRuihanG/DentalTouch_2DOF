#pragma once
class CFiveBarKinematics
{
public:
	CFiveBarKinematics(void);
	~CFiveBarKinematics(void);

public:
	//逆解
	bool InverseKinematics(double x, double y, double th[2]);
	//正解
	bool ForwardKinematics(double th1, double th2, double xy[2]);
	//雅克比
	bool JacobianTheta(double th1, double th2, double J[2][2]);
	bool JacobianXY(double x, double y, double J[2][2]);
	bool JacobianXYTheta(double x, double y, double th1, double th2, double J[2][2]);

	//获取笛卡尔坐标系的力
	bool GetForce(double x, double y, double th1, double th2, double Fsensor[2], double Fxy[2]);
};

