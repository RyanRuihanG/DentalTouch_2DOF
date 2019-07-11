#pragma once
class CDTKinematics
{
public:
	CDTKinematics(void);
	~CDTKinematics(void);


public:
	//逆解
	BOOL InverseKinematics(double x, double y, double z, double theta[5]);//反解，平均一次0.001ms，验证正确
	BOOL GetPose(double x, double y, double z, double pose[3][3]);//0.003ms
	//正解，多个重载
	BOOL ForwardKinematics(double th1, double th2, double th3, double xyz[3]);//正解平均0.01ms，验证正确
	BOOL ForwardKinematics(double th1, double th2, double th3, double xyzpos[4][4]);//正解返回齐次矩阵平均0.01ms，验证正确
	//雅克比
	BOOL Jacobian(double x, double y, double z, double J[3][3]);//由xyz得到雅克比，平均0.003ms
private:
	double NewtonRaphson(double x0, double tol,double q[9]);
	double Func(double t, double q[9]);
	double dFunc(double t, double q[9]);
};

