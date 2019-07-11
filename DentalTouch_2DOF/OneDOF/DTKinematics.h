#pragma once
class CDTKinematics
{
public:
	CDTKinematics(void);
	~CDTKinematics(void);


public:
	//���
	BOOL InverseKinematics(double x, double y, double z, double theta[5]);//���⣬ƽ��һ��0.001ms����֤��ȷ
	BOOL GetPose(double x, double y, double z, double pose[3][3]);//0.003ms
	//���⣬�������
	BOOL ForwardKinematics(double th1, double th2, double th3, double xyz[3]);//����ƽ��0.01ms����֤��ȷ
	BOOL ForwardKinematics(double th1, double th2, double th3, double xyzpos[4][4]);//���ⷵ����ξ���ƽ��0.01ms����֤��ȷ
	//�ſ˱�
	BOOL Jacobian(double x, double y, double z, double J[3][3]);//��xyz�õ��ſ˱ȣ�ƽ��0.003ms
private:
	double NewtonRaphson(double x0, double tol,double q[9]);
	double Func(double t, double q[9]);
	double dFunc(double t, double q[9]);
};

