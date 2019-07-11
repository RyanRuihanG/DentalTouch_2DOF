
// OneDOFDlg.h : 头文件
//

#pragma once

#include "SerialPort.h"//串口通信头文件
#include "mmsystem.h"  //head file，多媒体定时器头文件
#include "EcanVci.h"//can卡头文件
#include "CANCmd.h"	//CAN指令集，自己编写
#include "afxwin.h"
#include "afxcmn.h"
#include "FiveBarKinematics.h"
#include "bdaqctrl.h"			//研华数采头文件
using namespace Automation::BDaq;


#ifndef _PI
#define	_PI
	#define PI 3.141592654f//圆周率
#endif

#define POSITION_DIMENTION			2	//位置维度
#define NUMBER_JOINTS				2	//驱动关节数量
#define LINK_ENCODER_COUNTS			72000.0	//连杆编码器线数
#define INIT_X						110.99419336		//初始xy以及对应的关节角rad
#define INIT_Y						78.70562001
#define INIT_ANGLE_DPC1				(-20.00193979/180.0*PI)
#define INIT_ANGLE_DPC2				(65.00292158/180.0*PI)
#define INIT_ANGLE_LINK1			(-20.00193979/180.0*PI)
#define INIT_ANGLE_LINK2			(65.00292158/180.0*PI)

#define MAX_ANGLE_DPC1				(116.0/180.0*PI)		//最大最小关节限位
#define MIN_ANGLE_DPC1				(-21.0/180.0*PI)
#define MAX_ANGLE_DPC2				(201.0/180.0*PI)
#define MIN_ANGLE_DPC2				(64.0/180.0*PI)

#define MAX_X						100.0			//末端限制在正方形内，超过时则不更新物理约束角度
#define MIN_X						(-100.0)
#define MAX_Y						100.0
#define MIN_Y						(-100.0)

#define MAX_WALL_X					100.0			//虚拟墙定义为直线段，而不是直线
#define MIN_WALL_X					(-100.0)
//#define MAX_WALL_Y					100.0
//#define MIN_WALL_Y					(-100.0)

#define ALFA1						(1.5/180.0*PI)//关节1控制间隙
#define ALFA2						(1.5/180.0*PI)//
#define BETA1						(17.0375/180.0*PI)//关节2机械间隙
#define BETA2						(16.85/180.0*PI)

typedef struct tagDevConfParam
{
	int deviceNumber;
	long channel;
	SignalCountingType cntType;
	//CHAR* profilePath;
}DevConfParam, *PDevConfParam;




// COneDOFDlg 对话框
class COneDOFDlg : public CDialogEx
{
// 构造
public:
	COneDOFDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
	enum { IDD = IDD_ONEDOF_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


public:

	//多媒体定时器调用的触觉和视觉渲染程序
	void MMTimerHapticRendering(UINT nIDEvent);
	void MMTimerGraphicRendering(UINT nIDEvent);


public:
	//串口相关的变量和函数
	CSerialPort m_ComPortForce;	//力传感器串口类，只接收
	CSerialPort m_ComPortCAN;	//CAN，只发送
	CSerialPort m_ComPortDAQ;	//DAQ，只接收
	afx_msg LRESULT OnComRecvData(WPARAM str, LPARAM commInfo);//串口接收程序
	BOOL ProcessForceData(DWORD len);//处理力传感器数据
	DWORD m_ComPortNumForce,	//力传感器串口号
	m_ComPortNumDAQ,//DAQ串口号
	m_ComPortNumCAN;//CAN串口号

	//CAN通信相关
	UINT m_DeviceType;	//保存can设备句柄
	//static UINT ReceiveThread(void *param);	//CAN接收
	CCANCmd m_CANCmd;//CAN指令集的类
	CAN_OBJ m_CANCmdFrame[30];//从m_CANCmd的getframe函数获取的指令,最大一次发送10帧，由can卡接口定义

	//力传感器
	DOUBLE m_dAmpZero[6];
	DOUBLE m_dChnGain[6];
	DOUBLE m_dChnEx[6];//
	static CONST DOUBLE m_dDecouplingCoefficient[6][6];//解耦矩阵

	//运动学相关
	CFiveBarKinematics m_FiveBarKinematics;

	//********************控制变量（重要）*************************
	DOUBLE m_Position_HapticTool[POSITION_DIMENTION];	//末端位置
	DOUBLE m_Cmd_DPCAngle[NUMBER_JOINTS];				//命令物理约束关节角
	DOUBLE m_Current_DPCAngle[NUMBER_JOINTS];			//当前物理约束关节角
	DOUBLE m_Read_LinkAngle[NUMBER_JOINTS];				//读取的连杆角度
	DOUBLE m_Cartesian_Force[POSITION_DIMENTION];		//笛卡尔力
	
	DOUBLE m_Read_ForceSensor[6];						//力传感器的读数
	DOUBLE m_Read_ForceSensor0[6];		//初始笛卡尔力

	BOOL m_bOnlyFree;		//TRUE只跟随，否则力反馈
	BOOL m_bInitValue;		//初始化变量

	//初始化标志位，用来判断“开始按钮”程序运行状态
	BOOL m_bFlagInitForce;//力传感器初始化标志位
	BOOL m_bFlagInitDAQ;//力传感器初始化标志位
	BOOL m_bFlagInitDrive;//驱动器初始化标志位



	BOOL m_bFlagRunned;//表示运行过后点击了停止按钮，此时必须重启电源、重启程序
	BOOL m_bTimerRunning;//多媒体定时器运行标志位
	BOOL m_bFlagCanConnect;	//连接标志位
	BOOL m_bFlagForceConnect;	//连接标志位
	BOOL m_bFlagDAQConnect;	//连接标志位
	void DrawCoordinates();
	void DrawTetrahedron();
	void DrawRect();


	double m_JacobianMatrix[POSITION_DIMENTION][NUMBER_JOINTS];

public:
	int m_nXPos, m_nYPos; //position of mouse
	double m_fXAngle, m_fYAngle; //angle of eye
	double m_fDist; //distance from the eye
	HGLRC m_hRC;
	unsigned int m_nTexStone, m_nTexCloud, m_nTex1D;
	int m_nState;
	void upsidedown32(unsigned char *pImage, unsigned width, unsigned height);

	void DrawCuboid();
	double m_SphereX,m_SphereY;
	double m_y0, m_theta;

	void PaintVirtualEnvi();

	//圆形虚拟墙参数定义
	double m_circleX,m_circleY, m_circleR; 


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedBtnConnectForce();
	afx_msg void OnBnClickedBtnConfigureForce();
	afx_msg void OnBnClickedBtnOpenDaqPort();
	afx_msg void OnBnClickedBtnOpenCanPort();
	afx_msg void OnBnClickedBtnInitialDrive();
	afx_msg void OnBnClickedBtnStart();
	afx_msg void OnDestroy();
	CEdit m_Edit1;
	CProgressCtrl m_progress;

private:
	UdCounterCtrl*     m_udCounterCtrl;//研华数采控制文件
	DevConfParam       m_confParam; // the device's configure information from config form.
public:
	afx_msg void OnBnClickedFreeHaptic();
	afx_msg void OnBnClickedStop();
	afx_msg void OnBnClickedInitValue();
	int m_RadioBtn;
	afx_msg void OnBnClickedRadio1();
	CStatic m_pic;
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseHWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnBnClickedButton1();
};
