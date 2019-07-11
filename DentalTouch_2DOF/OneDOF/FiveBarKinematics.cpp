#include "stdafx.h"
#include "FiveBarKinematics.h"
#include "math.h"

#define PI 3.141592654
#define a	90.0f
#define b	110.0
#define c	(74.0/2.0)

CFiveBarKinematics::CFiveBarKinematics(void)
{
}


CFiveBarKinematics::~CFiveBarKinematics(void)
{
}

bool CFiveBarKinematics::InverseKinematics(double x, double y, double th[2])
{
	double t1, t2, th1_1, th1_2, xO1A, yO1A, xAQ,yAQ,th2_1,th2_2,xO2B,yO2B,xBQ,yBQ ;
    //求th1
    double v = x - c;
    double w = y;
    double u = ( v*v + y*y +a*a -b*b ) / 2.0 / a;
    
    double delta = w*w + v*v - u*u;
    
    if ( delta <= 0.0 )
	{
        th[0] = 1000.0;
        th[1] = 1000.0;
		return false;
	}
    
    if ( (u+v) == 0.0 )
        th[0] = PI;
    else
	{
        t1 = (w + sqrt(delta))/(u+v);
        t2 = (w - sqrt(delta))/(u+v);
        th1_1 = 2.0 * atan(t1);
        th1_2 = 2.0 * atan(t2);
        
        xO1A = a * cos(th1_1);
        yO1A = a * sin(th1_1);
        
        xAQ = x - ( a * cos(th1_1) +c );
        yAQ = y - a * sin(th1_1);
    
        if ( ( xO1A*yAQ - yO1A*xAQ ) > 0.0 )
            th[0] = th1_1;
        else
            th[0] = th1_2; 
	}
    //求th2
    v = x + c;
    w = y;
    u = ( v*v + y*y +a*a -b*b ) / 2.0 / a;
    
    delta = w*w + v*v - u*u;
    
    if ( delta <= 0.0 )
	{
        th[0] = 1000.0;
        th[1] = 1000.0;
		return false;
	}

    
    if ( (u+v) == 0.0 )
        th[1] = PI;
    else
	{
        t1 = (w + sqrt(delta))/(u+v);
        t2 = (w - sqrt(delta))/(u+v);
        th2_1 = 2.0 * atan(t1);
        th2_2 = 2.0 * atan(t2);
        
        xO2B = a * cos(th2_1);
        yO2B = a * sin(th2_1);
        
        xBQ = x - ( a * cos(th2_1) -c );
        yBQ = y - a * sin(th2_1);
    
        if ( ( xO2B*yBQ - yO2B*xBQ ) < 0.0)
            th[1] = th2_1;
        else
            th[1] = th2_2;   
	}

	if( th[1] < 0.0)
	{
		th[1] = th[1] + 2.0 * PI;
	}


    double c1 = cos(th[0]);
    double s1 = sin(th[0]); 
    double c2 = cos(th[1]);
    double s2 = sin(th[1]);
    
    double m1 = - a * c1 - c;
    double n1 = -a * s1;
    double m2 = - a * c2 + c;
    double n2 = -a * s2;
    
     //出去内陷情况
    double xA = - m1;
    double yA = - n1;
    double xB = - m2;
    double yB = - n2;
    
    xAQ = x - xA;
    yAQ = y - yA;
    xBQ = x - xB;
    yBQ = y - yB;
    
    if ( ( xAQ*yBQ - xBQ*yAQ  ) < 0.0)
		return true;
	else
	{
		th[0] = 1000.0;
		th[1] = 1000.0;
		return false;
	}
}
//正解
bool CFiveBarKinematics::ForwardKinematics(double th1, double th2, double xy[2])
{
	double x, y;
	double c1 = cos(th1);
    double s1 = sin(th1); 
    double c2 = cos(th2);
    double s2 = sin(th2);
    
    double m1 = - a * c1 - c;
    double n1 = -a * s1;
    double m2 = - a * c2 + c;
    double n2 = -a * s2;

	if (m1 == m2)    //有无数个解，不满足要求，返回较大的数值
	{
		x = 10000.0;
		y = 10000.0;
		return false;
	}

	double k = - ( n1 - n2 ) / ( m1 - m2 );
    double p = - ( m1*m1 - m2*m2 + n1*n1 - n2*n2)/( m1 - m2)/2.0;
    
    double e = k * k + 1;
    double f = k * ( p + m1 ) + n1;
    double g = ( p + m1) * ( p + m1) + n1 * n1 -b * b;
    
    double delta = f*f - e*g;

	delta = f*f - e*g;
    
    if ( delta <= 0.0 )
	{
        x = 10000.0;
        y = 10000.0;
        return false;  
	}
    
    double yQ1 = ( -f + sqrt( delta ) ) / e;
    double yQ2 = ( -f - sqrt( delta ) ) / e;
    
    double xQ1 = k*yQ1+p;
    double xQ2 = k*yQ2+p;

	//判断选哪个值
    double xA = - m1;
    double yA = - n1;
    double xB = - m2;
    double yB = - n2;
    
    double xAQ = xQ1 - xA;
    double yAQ = yQ1 - yA;
    double xBQ = xQ1 - xB;
    double yBQ = yQ1 - yB;

	if (( xAQ*yBQ - xBQ*yAQ  ) < 0.0)
	{
		x = xQ1;
		y = yQ1;
	}
	else
	{
	    x = xQ2;
		y = yQ2;
	}

	xy[0] = x;
	xy[1] = y;

	return true;
}
//雅克比
bool CFiveBarKinematics::JacobianTheta(double th1, double th2, double J[2][2])
{
	double XY[2];
	ForwardKinematics(th1,th2,XY);
    double x = XY[0];
    double y = XY[1];
    if(JacobianXYTheta( x, y,th1,th2,J ) == true)
		return true;
	else
		return false;
}
bool CFiveBarKinematics::JacobianXY(double x, double y, double J[2][2])
{
	double th[2];
	InverseKinematics( x, y, th);
    double th1 = th[0];
    double th2 = th[1];
	if(JacobianXYTheta( x, y,th1,th2,J ) == true)
		return true;
	else
		return false;
}
bool CFiveBarKinematics::JacobianXYTheta(double x, double y, double th1, double th2, double J[2][2])
{
	double c1 = cos(th1);
    double s1 = sin(th1); 
    double c2 = cos(th2);
    double s2 = sin(th2);
    
    double m1 = - a * c1 - c;
    double n1 = -a * s1;
    double m2 = - a * c2 + c;
    double n2 = -a * s2;

	double e = x+m1;
    double f = y+n1;
    double g = x+m2;
    double h = y+n2;

	double den = e*h-g*f;

	if( den == 0.0 )
	{
		J[0][0] = 0.0;
		J[0][1] = 0.0;
		J[1][0] = 0.0;
		J[1][1] = 0.0;
		return false;
	}

	J[0][0] = 1.0/den * h* ( -e*a*s1+f*a*c1 );
	J[0][1] = 1.0/den * (-f) * ( -g*a*s2+h*a*c2 );
	J[1][0] = 1.0/den * (-g)* ( -e*a*s1+f*a*c1 );
	J[1][1] = 1.0/den * (e) * ( -g*a*s2+h*a*c2 );

	return true;
}
bool CFiveBarKinematics::GetForce(double x, double y, double th1, double th2, double Fsensor[2], double Fxy[2])
{
	double c1 = cos(th1);
    double s1 = sin(th1); 
    double c2 = cos(th2);
    double s2 = sin(th2);
    
    double m1 = - a * c1 - c;
    double n1 = -a * s1;
    double m2 = - a * c2 + c;
    double n2 = -a * s2;

	//判断选哪个值
    double xA = - m1;
    double yA = - n1;
    double xAQ = x - xA;
    double yAQ = y - yA;

	double Fqx = - Fsensor[0];
	double Fqy = - Fsensor[1];

	double ca = xAQ/b;
	double sa = yAQ/b;

	printf("\n\nca = %f\tsa = %f\n",xAQ/b,yAQ/b);

	Fxy[0] = Fqx*ca - Fqy*sa;
	Fxy[1] = Fqx*sa + Fqy*ca;

	return true;
}




