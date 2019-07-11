#include "stdafx.h"
#include "math.h"
#include "DTKinematics.h"
#define a 50.0 
#define b 135.5 
#define c 45.0  
#define d 15.0
#define e 17.5 //根据模型修改
#define L1 70.0 
#define L2 164.0 
#define L3 70.0 
#define L4 164.0
#define L5 100.0
#define L6 135.5
#define L7 46.5
#define L8 (L7+L6)
#define pi 3.141592653589793

CDTKinematics::CDTKinematics(void)
{
}


CDTKinematics::~CDTKinematics(void)
{
}



BOOL CDTKinematics::InverseKinematics(double x, double y, double z, double theta[5])
{
	int i;
	double num, den;
	double th1, th2, th3, th4, th5;
	double s3, c3, s34, c34, s4, c4, s5, c5;
	double aa, bb, cc, dd, tt, xx, yy, kk;
	double delta;
	double xF, yF, zF,xC, yC, zC;

	for (i = 0; i < 5; i++) //初始化一组不在约束范围内的值b
	{
		theta[i] = 10.0;
	}

	/* 支链3逆运动学 */
	if ( fabs(y) >= L8) 
	{
		return FALSE;
	} 
	else 
	{
		num = asin( y / L8);
		th5 =  num;
	}
	s5 = sin(th5);
	c5 = cos(th5);

	aa = -2.0 * L5 * e;
	bb = 2.0 * L5 * L8 * c5;
	cc = (x+b) * (x+b) + y * y + (z-c) * (z-c) - L5 * L5 - L8 * L8 - e * e;
	delta = aa * aa + bb * bb -cc * cc;
	dd = ( aa + sqrt( delta ) ) / (bb + cc);
	if ( delta < 0.0 ) //th4不存在实根
		return FALSE;
	else
		th4 = 2.0 * atan( dd );
	s4 = sin(th4);
	c4 = cos(th4);

	num = (x+b) * (L5+L8*c5*c4) - (z-c)*L8*c5*s4 -e*(z-c)*c4- e*(x+b)*s4;
	den = (x+b)*L8*c5*s4 + (z-c)*(L5+L8*c5*c4) -e*(z-c)*s4+ e*(x+b)*c4;
	aa = num / den;
	tt = ( -1.0 + sqrt(1.0+aa*aa) ) / aa;
	th3 = 2.0 * atan( tt );

	//判断th3是否满足要求
	xx = sin(th3);
	yy = cos(th3);
	if ( ( x + b) > 0 )
	{
		kk = (z-c)/(x+b);
		if (yy <= kk*xx )
			th3 = th3 + pi;
	}
	else if ( ( x + b) == 0 && (z - c) > 0 )
	{
		if ( xx >= 0 )
			th3 = th3 + pi;
	}
	else if ( x+b < 0 )
	{
		kk = (z-c)/(x+b);
		if ( yy >= kk*xx )
			th3 = th3 + pi;
	}
	else if ( ( x+b ) == 0 && ( z-c ) < 0 )
	{
		if ( xx <= 0 )
			th3 = th3 + pi;
	}

	s3 = sin(th3);
	c3 = cos(th3);
	s34 =  sin(th3 + th4);
	c34 =  cos(th3 + th4);

	
	//支链2逆运动学
	xF = -b + L5*s3 + L6*s34*c5 - d*s34*s5;
	yF = L6*s5 + d*c5;
	zF = c + L5*c3 + L6*c34*c5 - d*c34*s5 ;

	aa = yF - a;
	bb = zF;
	cc = - ( L4*L4 - L3*L3 - xF*xF - (yF-a)*(yF-a) - zF*zF) / (2*L3);
	delta = aa*aa +bb*bb -cc*cc;
	dd = ( aa + sqrt( delta ) ) / (bb + cc);
	if ( delta < 0 ) //th2不存在实根
		return FALSE;
	else
		th2 = 2*atan( dd );

	//支链1逆运动学
	xC = - b + L5*s3  + L6*s34*c5 + d*s34*s5;
	yC = L6*s5 - d*c5;
	zC = c + L5*c3 + L6*c34*c5 + d*c34*s5;

	aa = yC + a;
	bb = -zC;
	cc = ( L2*L2 - L1*L1 - xC*xC- (yC+a)*(yC+a) - zC*zC) / (2*L1);
	delta = aa*aa +bb*bb -cc*cc;
	dd = ( aa - sqrt( delta ) ) / (bb + cc);
	if ( delta < 0 ) //th1不存在实根
		return FALSE;
	else
		th1 = 2*atan( dd );

	theta[0] = th1;
	theta[1] = th2;
	theta[2] = th3;
	theta[3] = th4;
	theta[4] = th5;

	return TRUE;
}

BOOL CDTKinematics::ForwardKinematics(double th1, double th2, double th3, double xyz[3])
{
	double th4, th5;
	double s1, c1, s2, c2, s3, c3, s34, c34, s4, c4, s5, c5;
	double n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15,n16,n17,n18,n19;
	double m1,m2,m3,m4,m5,m6,m7,m8,m9,m10,m11,m12,m13,m14,m15,m16,m17,m18,m19;
	double k1,k2,k3,k4,k5,k6,k7,r1,r2,r3,r4,r5,r6,r7;
	s1 = sin(th1);
	c1 = cos(th1);
	s2 = sin(th2);
	c2 = cos(th2);
	s3 = sin(th3);
	c3 = cos(th3);

	n1=-L2*L2+L1*L1 + L5*L5 + L6*L6 + a*a + b*b + c*c + d*d - 2.0*L1*c*c1 + 2.0*L5*c*c3 + 2.0*L1*a*s1 - 2.0*L5*b*s3 - 2.0*L1*L5*c1*c3;
    n2=2.0*L6*a;
    n3=-2.0*a*d;
    n4=2.0*L5*L6;
    n5=2.0*L1*L6*s1;
    n6=-2.0*L1*d*s1;
    n7=2.0*L5*d;
    n8=2.0*L6*c*c3;
    n9=-2.0*L6*b*c3;
    n10=-2.0*L6*b*s3;
    n11=-2.0*L6*c*s3;
    n12=2.0*c*d*c3;
    n13=-2.0*b*d*c3;
    n14=-2.0*b*d*s3;
    n15=-2.0*c*d*s3;
    n16=-2.0*L1*L6*c1*c3;
    n17=2.0*L1*L6*c1*s3;
    n18=-2.0*L1*d*c1*c3;
    n19=2.0*L1*d*c1*s3;

	m1=-L4*L4+L3*L3 + L5*L5 + L6*L6 + a*a + b*b + c*c + d*d - 2.0*L3*c*c2 + 2.0*L5*c*c3 + 2.0*L3*a*s2 - 2.0*L5*b*s3- 2.0*L3*L5*c2*c3;
    m2=-n2;
    m3=n3;
    m4=n4;
    m5=-2.0*L3*L6*s2;
    m6=-2.0*L3*d*s2;
    m7=-n7;
    m8=n8;
    m9= n9;
    m10=n10;
    m11= n11;
    m12= -n12;
    m13=-n13;
    m14=-n14;
    m15=-n15;
    m16=-2.0*L3*L6*c2*c3;
    m17=2.0*L3*L6*c2*s3;
    m18=2.0*L3*d*c2*c3;
    m19= -2.0*L3*d*c2*s3;

	k1=n1;
    k2=n2+n5;
    k3=n3+n6;
    k4=n4+n8+n10+n16;
    k5=n7+n12+n14+n18;
    k6=n9+n11+n17;
    k7=n13+n15+n19;

    r1=m1;
    r2=m2+m5;
    r3=m3+m6;
    r4=m4+m8+m10+m16;
    r5=m7+m12+m14+m18;
    r6=m9+m11+m17;
    r7=m13+m15+m19;    

	//求解theta33
    if ( th2 == th1)
        th5=0.0;
    else  
	{
        double wa1 = k1-k3;
        double wb1 = 2.0*k2;
        double wc1 = k1+k3;
        double wa2 = -k4;
        double wb2 = 2.0*k5;
        double wc2 = k4;
        double wa3 = -k6;
        double wb3 = 2.0*k7;
        double wc3 = k6;
        double va1 = r1-r3;
        double vb1 = 2.0*r2;
        double vc1 = r1+r3;
        double va2 = -r4;
        double vb2 = 2.0*r5;
        double vc2 = r4;
        double va3 = -r6;
        double vb3 = 2.0*r7;
        double vc3 = r6;
        
        double fa4 = va2*wa1 - va1*wa2;
        double fa3 = va2*wb1 - va1*wb2 - vb1*wa2 + vb2*wa1;
        double fa2 = va2*wc1 - va1*wc2 - vb1*wb2 + vb2*wb1 - vc1*wa2 + vc2*wa1;
        double fa1 = vb2*wc1 - vb1*wc2 - vc1*wb2 + vc2*wb1;
        double fa0 = vc2*wc1 - vc1*wc2;
        
        double fb4 = va1*wa3 - va3*wa1;
        double fb3 = va1*wb3 - va3*wb1 + vb1*wa3 - vb3*wa1;
        double fb2 = va1*wc3 - va3*wc1 + vb1*wb3 - vb3*wb1 + vc1*wa3 - vc3*wa1;
        double fb1 = vb1*wc3 - vb3*wc1 + vc1*wb3 - vc3*wb1;
        double fb0 = vc1*wc3 - vc3*wc1;
        
        double fc4 = va3*wa2 - va2*wa3;
        double fc3 = va3*wb2 - va2*wb3 - vb2*wa3 + vb3*wa2;
        double fc2 = va3*wc2 - va2*wc3 - vb2*wb3 + vb3*wb2 - vc2*wa3 + vc3*wa2;
        double fc1 = vb3*wc2 - vb2*wc3 - vc2*wb3 + vc3*wb2;
        double fc0 = vc3*wc2 - vc2*wc3;
        
		double q[9];
		q[8] = fa4*fa4 + fb4*fb4 - fc4*fc4;
		q[7] = 2.0*fa3*fa4 + 2.0*fb3*fb4 - 2.0*fc3*fc4;
		q[6] = fa3*fa3 + fb3*fb3 - fc3*fc3 + 2.0*fa2*fa4 + 2.0*fb2*fb4 - 2.0*fc2*fc4;
		q[5] = 2.0*fa1*fa4 + 2.0*fa2*fa3 + 2.0*fb1*fb4 + 2.0*fb2*fb3 - 2.0*fc1*fc4 - 2.0*fc2*fc3;
		q[4] = fa2*fa2 + fb2*fb2 - fc2*fc2 + 2.0*fa0*fa4 + 2.0*fa1*fa3 + 2.0*fb0*fb4 + 2.0*fb1*fb3 - 2.0*fc0*fc4 - 2.0*fc1*fc3;
		q[3] = 2.0*fa0*fa3 + 2.0*fa1*fa2 + 2.0*fb0*fb3 + 2.0*fb1*fb2 - 2.0*fc0*fc3 - 2.0*fc1*fc2;
		q[2] = fa1*fa1 + fb1*fb1 - fc1*fc1 + 2.0*fa0*fa2 + 2.0*fb0*fb2 - 2.0*fc0*fc2;
		q[1] = 2.0*fa0*fa1 + 2.0*fb0*fb1 - 2.0*fc0*fc1;
		q[0] = fa0*fa0 + fb0*fb0 - fc0*fc0;
        
        //8阶方程
        //double f=q8*power(t,8);+q7*t.^7+q6*t.^6+q5*t.^5+q4*t.^4+q3*t.^3+q2*t.^2+q1*t+q0;
        double t0;
        //定迭代初值
        double delta = fabs(th1-th2)/pi*180.0;//单位转换成°
        if (( 0.0 < delta) && ( delta <= 1.0))
            t0 = 0.00873;
        if ( 1.0 < delta && delta <= 2.0 )
            t0 = 0.01746;
        if ( 2.0 < delta && delta<=5.0)
            t0 = 0.04366;     
        if ( 5.0<delta && delta<=10.0)
            t0 = 0.08748;   
        if (10.0<delta && delta<=20.0)
            t0 = 0.13165;      
        if (20.0<delta && delta<=30.0)
            t0 = 0.17633;  
        if (30.0<delta && delta<=40.0)
            t0 = 0.22169;     
        if (40.0<delta && delta<=60.0)
            t0 = 0.26795;       
        if (60.0<delta && delta<=90.0)
            t0 = 0.31530;
        if (90.0<delta && delta<=100.0)
            t0 = 0.36397;
        
        if (th1>th2)
            t0 = -t0;

        //牛顿法求出根t
        double gen=NewtonRaphson(t0,1e-8,q);

        th5=2*atan(gen);
	}

	//以上求出了theta33,下面求theta32
    s5 = sin(th5);
    c5 = cos(th5);
    
    double w1=k1+k2*s5+k3*c5;
    double w2=k4*c5+k5*s5;
    double w3=k6*c5+k7*s5;
    double v1=r1+r2*s5+r3*c5;
    double v2=r4*c5+r5*s5;
    double v3=r6*c5+r7*s5;

    if ( th1 != th2)
	{
        double fenzi=v2*w1-w2*v1;
        double fenmu=v1*w3-v3*w1;
        double shang=fenzi/fenmu;
        if ( shang >= 0.0 )  
            th4=atan(shang);
        else
            th4=atan(shang)+pi;
	}
    else
	{
        double ww0=-w2/w3;
        double ww1=-w1/w3;
        double dt=(2.0*ww0*ww1)*(2.0*ww0*ww1) - 4.0*(ww0*ww0+1.0)*(ww1*ww1-1.0);
        double gen1=(-2.0*ww0*ww1+sqrt(dt))/(2.0*(ww0*ww0+1.0));
        th4=acos(gen1);
	}

    
    s34 = sin(th3+th4);
    c34 = cos(th3+th4);
    //Q点坐标
    double xQ=L5*s3+L8*s34*c5-b;
    double yQ=L8*s5;
    double zQ=L5*c3+L8*c34*c5+c;
    //P点坐标
    double Qnz = -c34;
    double Qoz = 0;
    double Qaz = s34;
    xyz[0] = xQ - e*Qnz;
    xyz[1] = yQ - e*Qoz;
    xyz[2] = zQ - e*Qaz;

	return TRUE;
}

BOOL CDTKinematics::ForwardKinematics(double th1, double th2, double th3, double xyzpos[4][4])//正解返回齐次矩阵
{
	double th4, th5;
	double s1, c1, s2, c2, s3, c3, s34, c34, s4, c4, s5, c5;
	double n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15,n16,n17,n18,n19;
	double m1,m2,m3,m4,m5,m6,m7,m8,m9,m10,m11,m12,m13,m14,m15,m16,m17,m18,m19;
	double k1,k2,k3,k4,k5,k6,k7,r1,r2,r3,r4,r5,r6,r7;
	s1 = sin(th1);
	c1 = cos(th1);
	s2 = sin(th2);
	c2 = cos(th2);
	s3 = sin(th3);
	c3 = cos(th3);

	n1=-L2*L2+L1*L1 + L5*L5 + L6*L6 + a*a + b*b + c*c + d*d - 2.0*L1*c*c1 + 2.0*L5*c*c3 + 2.0*L1*a*s1 - 2.0*L5*b*s3 - 2.0*L1*L5*c1*c3;
    n2=2.0*L6*a;
    n3=-2.0*a*d;
    n4=2.0*L5*L6;
    n5=2.0*L1*L6*s1;
    n6=-2.0*L1*d*s1;
    n7=2.0*L5*d;
    n8=2.0*L6*c*c3;
    n9=-2.0*L6*b*c3;
    n10=-2.0*L6*b*s3;
    n11=-2.0*L6*c*s3;
    n12=2.0*c*d*c3;
    n13=-2.0*b*d*c3;
    n14=-2.0*b*d*s3;
    n15=-2.0*c*d*s3;
    n16=-2.0*L1*L6*c1*c3;
    n17=2.0*L1*L6*c1*s3;
    n18=-2.0*L1*d*c1*c3;
    n19=2.0*L1*d*c1*s3;

	m1=-L4*L4+L3*L3 + L5*L5 + L6*L6 + a*a + b*b + c*c + d*d - 2.0*L3*c*c2 + 2.0*L5*c*c3 + 2.0*L3*a*s2 - 2.0*L5*b*s3- 2.0*L3*L5*c2*c3;
    m2=-n2;
    m3=n3;
    m4=n4;
    m5=-2.0*L3*L6*s2;
    m6=-2.0*L3*d*s2;
    m7=-n7;
    m8=n8;
    m9= n9;
    m10=n10;
    m11= n11;
    m12= -n12;
    m13=-n13;
    m14=-n14;
    m15=-n15;
    m16=-2.0*L3*L6*c2*c3;
    m17=2.0*L3*L6*c2*s3;
    m18=2.0*L3*d*c2*c3;
    m19= -2.0*L3*d*c2*s3;

	k1=n1;
    k2=n2+n5;
    k3=n3+n6;
    k4=n4+n8+n10+n16;
    k5=n7+n12+n14+n18;
    k6=n9+n11+n17;
    k7=n13+n15+n19;

    r1=m1;
    r2=m2+m5;
    r3=m3+m6;
    r4=m4+m8+m10+m16;
    r5=m7+m12+m14+m18;
    r6=m9+m11+m17;
    r7=m13+m15+m19;    

	//求解theta33
    if ( th2 == th1)
        th5=0.0;
    else  
	{
        double wa1 = k1-k3;
        double wb1 = 2.0*k2;
        double wc1 = k1+k3;
        double wa2 = -k4;
        double wb2 = 2.0*k5;
        double wc2 = k4;
        double wa3 = -k6;
        double wb3 = 2.0*k7;
        double wc3 = k6;
        double va1 = r1-r3;
        double vb1 = 2.0*r2;
        double vc1 = r1+r3;
        double va2 = -r4;
        double vb2 = 2.0*r5;
        double vc2 = r4;
        double va3 = -r6;
        double vb3 = 2.0*r7;
        double vc3 = r6;
        
        double fa4 = va2*wa1 - va1*wa2;
        double fa3 = va2*wb1 - va1*wb2 - vb1*wa2 + vb2*wa1;
        double fa2 = va2*wc1 - va1*wc2 - vb1*wb2 + vb2*wb1 - vc1*wa2 + vc2*wa1;
        double fa1 = vb2*wc1 - vb1*wc2 - vc1*wb2 + vc2*wb1;
        double fa0 = vc2*wc1 - vc1*wc2;
        
        double fb4 = va1*wa3 - va3*wa1;
        double fb3 = va1*wb3 - va3*wb1 + vb1*wa3 - vb3*wa1;
        double fb2 = va1*wc3 - va3*wc1 + vb1*wb3 - vb3*wb1 + vc1*wa3 - vc3*wa1;
        double fb1 = vb1*wc3 - vb3*wc1 + vc1*wb3 - vc3*wb1;
        double fb0 = vc1*wc3 - vc3*wc1;
        
        double fc4 = va3*wa2 - va2*wa3;
        double fc3 = va3*wb2 - va2*wb3 - vb2*wa3 + vb3*wa2;
        double fc2 = va3*wc2 - va2*wc3 - vb2*wb3 + vb3*wb2 - vc2*wa3 + vc3*wa2;
        double fc1 = vb3*wc2 - vb2*wc3 - vc2*wb3 + vc3*wb2;
        double fc0 = vc3*wc2 - vc2*wc3;
        
		double q[9];
		q[8] = fa4*fa4 + fb4*fb4 - fc4*fc4;
		q[7] = 2.0*fa3*fa4 + 2.0*fb3*fb4 - 2.0*fc3*fc4;
		q[6] = fa3*fa3 + fb3*fb3 - fc3*fc3 + 2.0*fa2*fa4 + 2.0*fb2*fb4 - 2.0*fc2*fc4;
		q[5] = 2.0*fa1*fa4 + 2.0*fa2*fa3 + 2.0*fb1*fb4 + 2.0*fb2*fb3 - 2.0*fc1*fc4 - 2.0*fc2*fc3;
		q[4] = fa2*fa2 + fb2*fb2 - fc2*fc2 + 2.0*fa0*fa4 + 2.0*fa1*fa3 + 2.0*fb0*fb4 + 2.0*fb1*fb3 - 2.0*fc0*fc4 - 2.0*fc1*fc3;
		q[3] = 2.0*fa0*fa3 + 2.0*fa1*fa2 + 2.0*fb0*fb3 + 2.0*fb1*fb2 - 2.0*fc0*fc3 - 2.0*fc1*fc2;
		q[2] = fa1*fa1 + fb1*fb1 - fc1*fc1 + 2.0*fa0*fa2 + 2.0*fb0*fb2 - 2.0*fc0*fc2;
		q[1] = 2.0*fa0*fa1 + 2.0*fb0*fb1 - 2.0*fc0*fc1;
		q[0] = fa0*fa0 + fb0*fb0 - fc0*fc0;
        
        //8阶方程
        //double f=q8*power(t,8);+q7*t.^7+q6*t.^6+q5*t.^5+q4*t.^4+q3*t.^3+q2*t.^2+q1*t+q0;
        double t0;
        //定迭代初值
        double delta = fabs(th1-th2)/pi*180.0;//单位转换成°
        if (( 0.0 < delta) && ( delta <= 1.0))
            t0 = 0.00873;
        if ( 1.0 < delta && delta <= 2.0 )
            t0 = 0.01746;
        if ( 2.0 < delta && delta<=5.0)
            t0 = 0.04366;     
        if ( 5.0<delta && delta<=10.0)
            t0 = 0.08748;   
        if (10.0<delta && delta<=20.0)
            t0 = 0.13165;      
        if (20.0<delta && delta<=30.0)
            t0 = 0.17633;  
        if (30.0<delta && delta<=40.0)
            t0 = 0.22169;     
        if (40.0<delta && delta<=60.0)
            t0 = 0.26795;       
        if (60.0<delta && delta<=90.0)
            t0 = 0.31530;
        if (90.0<delta && delta<=100.0)
            t0 = 0.36397;
        
        if (th1>th2)
            t0 = -t0;

        //牛顿法求出根t
        double gen=NewtonRaphson(t0,1e-8,q);

        th5=2*atan(gen);
	}

	//以上求出了theta33,下面求theta32
    s5 = sin(th5);
    c5 = cos(th5);
    
    double w1=k1+k2*s5+k3*c5;
    double w2=k4*c5+k5*s5;
    double w3=k6*c5+k7*s5;
    double v1=r1+r2*s5+r3*c5;
    double v2=r4*c5+r5*s5;
    double v3=r6*c5+r7*s5;

    if ( th1 != th2)
	{
        double fenzi=v2*w1-w2*v1;
        double fenmu=v1*w3-v3*w1;
        double shang=fenzi/fenmu;
        if ( shang >= 0.0 )  
            th4=atan(shang);
        else
            th4=atan(shang)+pi;
	}
    else
	{
        double ww0=-w2/w3;
        double ww1=-w1/w3;
        double dt=(2.0*ww0*ww1)*(2.0*ww0*ww1) - 4.0*(ww0*ww0+1.0)*(ww1*ww1-1.0);
        double gen1=(-2.0*ww0*ww1+sqrt(dt))/(2.0*(ww0*ww0+1.0));
        th4=acos(gen1);
	}

    
    s34 = sin(th3+th4);
    c34 = cos(th3+th4);
    //Q点坐标
    double xQ=L5*s3+L8*s34*c5-b;
    double yQ=L8*s5;
    double zQ=L5*c3+L8*c34*c5+c;
    //P点坐标
	//double Qnx = s34*c5;
 //   double Qox = s5;
 //   double Qax = c34*c5;

	//double Qny = -s34*s5;
 //   double Qoy = c5;
 //   double Qay = -c34*s5;

    //double Qnz = -c34;
    //double Qoz = 0;
    //double Qaz = s34;
	// xyz[0] = xQ - e*(-c34);
    //xyz[1] = yQ - e* 0.0;
    //xyz[2] = zQ - e* s34;

	xyzpos[0][0] = s34*c5;
	xyzpos[1][0] = s5;
	xyzpos[2][0] = c34*c5;
	xyzpos[3][0] = 0.0;

	xyzpos[0][1] = -s34*s5;
	xyzpos[1][1] = c5;
	xyzpos[2][1] = -c34*s5;
	xyzpos[3][1] = 0.0;

	xyzpos[0][2] = -c34;
	xyzpos[1][2] = 0;
	xyzpos[2][2] = s34;
	xyzpos[3][2] = 0.0;

	xyzpos[0][3] = xQ - e*(-c34);
	xyzpos[1][3] = yQ - e* 0.0;
	xyzpos[2][3] = zQ - e* s34;
	xyzpos[3][3] = 1.0;

	return TRUE;
}
BOOL  CDTKinematics::GetPose(double x, double y, double z, double pose[3][3])
{
	double theta[5];
	double s1, c1, s2, c2, s3, c3, s34, c34, s4, c4, s5, c5;
	InverseKinematics(x, y, z, theta);
	double th1 = theta[0],	th2 = theta[1],	th3 = theta[2],	th4 = theta[3],	th5 = theta[4];
	s1 = sin(th1);
	c1 = cos(th1);
	s2 = sin(th2);
	c2 = cos(th2);
	s3 = sin(th3);
	c3 = cos(th3);
	s4 = sin(th4);
	c4 = cos(th4);
	s5 = sin(th5);
	c5 = cos(th5);
	s34 = sin(th3+th4);
	c34 = cos(th3+th4);

	pose[0][0] = s34*c5;
	pose[1][0] = s5;
	pose[2][0] = c34*c5;

	pose[0][1] = -s34*s5;
	pose[1][1] = c5;
	pose[2][1] = -c34*s5;

	pose[0][2] = -c34;
	pose[1][2] = 0;
	pose[2][2] = s34;

	return TRUE;
}

double CDTKinematics::NewtonRaphson(double x0, double tol,double q[9])//迭代算法，函数的导数需要提前求好。x0初值,表示theta33,tol允许误差
{
	double x1=x0,wucha=0.1;//给定一个初始误差
	double f,df,gen;
	int time=0;
	while(wucha>tol)
	{
		f=Func(x1,q);//计算原函数值
		df=dFunc(x1,q);//计算一阶导数值
		gen=x1-f/df;
		wucha=fabs(gen-x1);
		x1=gen;
		time++;
	}
	return gen;
}

double CDTKinematics::Func(double t, double q[9])
{
	double q0 = q[0];
	double q1 = q[1];
	double q2 = q[2];
	double q3 = q[3];
	double q4 = q[4];
	double q5 = q[5];
	double q6 = q[6];
	double q7 = q[7];
	double q8 = q[8];
	return q8 * pow(t,8) + q7* pow(t,7) + q6 * pow(t,6) + q5*pow(t,5)+q4*pow(t,4)+q3*pow(t,3)+q2*pow(t,2)+q1*t+q0;
}
double CDTKinematics::dFunc(double t, double q[9])
{
	double q0 = q[0];
	double q1 = q[1];
	double q2 = q[2];
	double q3 = q[3];
	double q4 = q[4];
	double q5 = q[5];
	double q6 = q[6];
	double q7 = q[7];
	double q8 = q[8];
	return 8.0*q8 * pow(t,7) + 7.0*q7* pow(t,6) + 6.0*q6 * pow(t,5) + 5.0*q5*pow(t,4)+4.0*q4*pow(t,3)+3.0*q3*pow(t,2)+2.0*q2*pow(t,1)+q1;
}


BOOL CDTKinematics::Jacobian(double x, double y, double z, double JP[3][3])//由xyz得到雅克比
{
	int i0,j0,k0;
	double theta[5];
	InverseKinematics(x, y, z, theta);
	double th1 = theta[0],	th2 = theta[1],	th3 = theta[2],	th4 = theta[3],	th5 = theta[4];
	double s1, c1, s2, c2, s3, c3, s34, c34, s4, c4, s5, c5;
	s1 = sin(th1);
	c1 = cos(th1);
	s2 = sin(th2);
	c2 = cos(th2);
	s3 = sin(th3);
	c3 = cos(th3);
	s4 = sin(th4);
	c4 = cos(th4);
	s5 = sin(th5);
	c5 = cos(th5);
	s34 = sin(th3 + th4);
	c34 = cos(th3 + th4);
	double r11 = L5*c3 + c34*c5*L8 ;
	double r12 = c34*c5*L8 ;
	double r13 = -s34*s5*L8 ;
	double r21 = 0;
	double r22 = 0;
	double r23 = c5*L8;
	double r31 = - L5*s3 - s34*c5*L8;
	double r32 = -s34*c5*L8;
	double r33 = -c34*s5*L8;
	double r[3][3] = {{r11,r12,r13},{r21,r22,r23},{r31,r32,r33}};

	double exp1 = L5*c3;
	double exp2 = L6*c34*c5;
	double exp3 = d*c34*s5;
	double exp4 = L5*s3;
	double exp5 = L6*s34*c5;
	double exp6 = d*s34*s5;
	double exp7 = L1*c1;
	double exp8 = L6*c5;
	double exp9 = d*s5;
	double exp10 = L1*s1;
	double exp11 = L6*s5;
	double exp12 = d*c5;
	double exp13 = L6*c34*s5;
	double exp14 = d*c34*c5;
	double exp15 = L6*s34*s5;
	double exp16 = d*s34*c5;
	double exp17 = L3*c2;
	double exp18 = L3*s2;
	double exp19 = exp1 + exp2 + exp3;
	double exp20 = exp4 - b + exp5 + exp6;
	double exp21 = exp4 + exp5 + exp6;
	double exp22 = c - exp7 + exp1 + exp2 + exp3;
	double exp23 = exp2 + exp3;
	double exp24 = exp5 + exp6;
	double exp25 = exp8 + exp9;
	double exp26 = a + exp10 + exp11 - exp12;
	double exp27 = exp13 - exp14;
	double exp28 = exp15 - exp16;
	double exp29 = exp4 + exp5 - exp6;
	double exp30 = c - exp17 + exp1 + exp2 - exp3;
	double exp31 = exp1 + exp2 - exp3;
	double exp32 = b - exp4 - exp5 + exp6;
	double exp33 = exp2 - exp3;
	double exp34 = exp5 - exp6;
	double exp35 = exp15 + exp16;
	double exp36 = exp8 - exp9;
	double exp37 = a + exp18 - exp11 - exp12;
	double exp38 = exp13 + exp14;

	double f = 2.0*exp19*exp20 - 2.0*exp21*exp22;
	double g = 2.0*exp23*exp20 - 2.0*exp24*exp22; 
	double h = 2.0*exp25*exp26 - 2.0*exp27*exp22 - 2.0*exp28*exp20;
	double p = 2.0*exp7*exp26 + 2.0*exp10*exp22;
	double l = - 2.0*exp29*exp30 - 2.0*exp31*exp32;
	double m = - 2.0*exp33*exp32 - 2.0*exp34*exp30;
	double n = 2.0*exp35*exp32 - 2.0*exp36*exp37 - 2.0*exp38*exp30;
	double q = 2.0*exp17*exp37 + 2.0*exp18*exp30;

	double den = g*n-m*h;

	double AA[3][3] = {{-e*s34,-e*s34,0},{0,0,0},{-e*c34,-e*c34,0}};
	double BB[3][3] = {{0,0,1},{-n*p/den,h*q/den,(h*l-n*f)/den},{m*p/den,-g*q/den,(m*f-g*l)/den}};
	double AB[3][3] = {0.0};

	double J[3][3];
	for ( i0 = 0; i0 < 3; i0++)
	{
		J[i0][0]= -n*p/den*r[i0][1]+m*p/den*r[i0][2];
		J[i0][1]= h*q/den*r[i0][1]-g*q/den*r[i0][2] ;
		J[i0][2]= r[i0][0]+(h*l-n*f)/den*r[i0][1]+(m*f-g*l)/den*r[i0][2];
	}

	for(i0=0;i0<3;i0++)  
	{
		for(j0=0;j0<3;j0++)  
		{  
			for( k0=0; k0<3; k0++)  
			{  
				AB[i0][j0] += AA[i0][k0] * BB[k0][j0];  
			}
			JP[i0][j0] = AB[i0][j0] + J[i0][j0];
		}  
	}
	return TRUE;
}


