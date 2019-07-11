
// OneDOFDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "OneDOF.h"
#include "OneDOFDlg.h"
#include "afxdialogex.h"
#include "math.h"  

#ifdef _DEBUG
#define new DEBUG_NEW
#endif



//多媒体定时器相关变量
UINT g_uResolution = 1;//分辨率
CONST UINT g_uDelayHaptic = 2;//触觉周期
CONST UINT g_uDelayGraphic = 30;//视觉周期
MMRESULT g_TimerVR;	//视觉定时器
MMRESULT g_TimerHR;	//触觉定时器
//多媒体定时器回调函数, 触觉渲染，1ms
void CALLBACK HapticRendering(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2);
//多媒体定时器回调函数, 视觉渲染,30ms
void CALLBACK GraphicRendering(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2);

//DAQ串口相关的变量，处理方法为：接收到当前帧立刻进行扫描判断，如果当前帧无效，忽略继续下一帧处理，在
//触觉渲染程序中使用上一次数据
#define DAQ_BUFFER_SIZE 16384	//所有DAQ相关的缓存大小
BYTE g_DAQReadBuffer[DAQ_BUFFER_SIZE] = {0};	//初始读取串口的数组
volatile double	g_DAQPositionBuffer[DAQ_BUFFER_SIZE][6] = {0.0};//顺序循环保存编码器123和姿态RPY的数值，每一行表示一个数据点，单位为弧度
volatile DWORD	g_DAQPositionIndex = 0;//当前g_DAQPositionBuffer的索引
DOUBLE	g_ComputingTime = 0.0; //程序运行时间



//力传感器相关数据
#define FORCE_BUFFER_SIZE 16384	//相关的缓存大小
BYTE * g_ForceReadBuffer = new BYTE[FORCE_BUFFER_SIZE];	//初始读取力串口的数组
BYTE * g_ForceMyRxBuffer = new BYTE[FORCE_BUFFER_SIZE];	//循环保存读取的数据
DWORD	g_MyRxBufferCounter = 0;
DWORD	g_DataPtrOut = 0;//上一完整帧最后一个字节索引
volatile double	g_ForceBuffer[FORCE_BUFFER_SIZE][6] = {0.0};//顺序循环保存Fx/FY/FZ/MX/MY/MZ
volatile DWORD	g_ForceIndex = 0;//当前g_ForceBuffer的索引
const double COneDOFDlg::m_dDecouplingCoefficient[6][6]=
{
	{0.190660744, 	0.223685906, 	0.534145445, 	18.003595500 ,	0.078319299, 	-17.726435630}, 
	{0.107527388 ,	-20.122044610 ,	0.477155640 ,	10.616147420 ,	0.295934085 ,	10.000624860}, 
	{56.159414210 ,	-0.330263321 ,	55.926562190 ,	1.128619676 ,	54.707066010 ,	-0.994680395}, 
	{-0.032886763, 	-0.013842118 ,	-0.308634583 ,	0.003593230 ,	0.251016213 ,	0.004968525 },
	{0.346667582 ,	0.000148175 ,	-0.131789607 ,	-0.016990855 ,	-0.143606749 ,	0.017654130 },
	{0.000069847 ,	0.125876500 ,	0.003589640 ,	0.118673608 ,	-0.000442646 ,	0.086879145 },

};


//质量阻尼模型
const double T = g_uDelayHaptic/1000.0;//周期s
double vt = 0.0, vt_1 = 0.0;//虚拟速度m/s
double xt = 0.0, xt_1 =0.0;//虚拟位置m
double Fv = 0.0;//虚拟力

int aa;
static double pp = 0.0;
int fen = 0;


#ifndef _COUNTS_ENCODER
#define	_COUNTS_ENCODER
	#define COUNTS_ENCODER 72000.0f//编码器分辨率
#endif

#ifndef _PI
#define	_PI
	#define PI 3.141592654f//圆周率
#endif

#ifndef _CANMODE
#define _CANMODE
	#define MODE_STARTCAN 0
	#define MODE_CANMODE 1
	#define MODE_OPERATIONMODE 2
	#define MODE_PROFILETYPE 3
	#define MODE_ENABLEMOTION 4
	#define MODE_STARTMOTION 5
	#define MODE_POSITION 6
	#define MODE_VELOCITY 7
	#define MODE_ACCELERATION 8
	#define MODE_DECELERATION 9
	#define MODE_SETPOSITIONZERO 10
#endif



// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// COneDOFDlg 对话框



COneDOFDlg::COneDOFDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(COneDOFDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

	m_ComPortNumForce = 7;	//力传感器串口号
	m_ComPortNumDAQ = 4;//DAQ串口号

	m_bTimerRunning = FALSE;//多媒体定时没有运行

	m_bFlagInitDrive = FALSE;//驱动器初始化标志位
	m_bFlagInitForce = FALSE;//力传感器初始化标志位

	m_bFlagRunned = FALSE;//已经运行过？


	m_bFlagCanConnect= FALSE;	//连接标志位
	m_bFlagForceConnect= FALSE;	//连接标志位
	m_bFlagDAQConnect= FALSE;	//连接标志位


	DOUBLE temp1[6] = { 32716.000000, 32764.000000, 32761.000000,32800.000000,32683.000000,32672.000000};
	DOUBLE temp2[6] = { 123.080090,	123.213244,	123.044366,	123.158034,	123.054109,	123.115814};
	DOUBLE temp3[6] = { 2.515955,	2.515955,	2.515955,	2.515955,	2.515955,	2.515955};
	for( INT i = 0; i < 6; i++ )
	{
		m_dAmpZero[i] = temp1[i]; 
		m_dChnGain[i] = temp2[i];
		m_dChnEx[i] = temp3[i];
	}



}

void COneDOFDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_EDIT1, m_Edit1);
	DDX_Control(pDX, IDC_PROGRESS1, m_progress);
}

BEGIN_MESSAGE_MAP(COneDOFDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BTN_CONNECT_FORCE, &COneDOFDlg::OnBnClickedBtnConnectForce)
	ON_BN_CLICKED(IDC_BTN_CONFIGURE_FORCE, &COneDOFDlg::OnBnClickedBtnConfigureForce)
	ON_BN_CLICKED(IDC_BTN_OPEN_DAQ_PORT, &COneDOFDlg::OnBnClickedBtnOpenDaqPort)
	ON_BN_CLICKED(IDC_BTN_OPEN_CAN_PORT, &COneDOFDlg::OnBnClickedBtnOpenCanPort)
	ON_BN_CLICKED(IDC_BTN_INITIAL_DRIVE, &COneDOFDlg::OnBnClickedBtnInitialDrive)
	ON_BN_CLICKED(IDC_BTN_START, &COneDOFDlg::OnBnClickedBtnStart)
	ON_WM_DESTROY()

	ON_MESSAGE(WM_COMM_RXSTR, OnComRecvData)//串口cnComm.h接收数据回调函数

END_MESSAGE_MAP()


// COneDOFDlg 消息处理程序

BOOL COneDOFDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void COneDOFDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void COneDOFDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR COneDOFDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



/**以下为触觉视觉渲染多媒体定时器线程函数**************************************************/
//多媒体定时器回调函数多线程, 触觉渲染，1ms
void CALLBACK HapticRendering(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2)
{
	COneDOFDlg * obj = (COneDOFDlg*) dwUser;
	obj->MMTimerHapticRendering(wTimerID);

}

//多媒体定时器回调函数多线程, 视觉渲染,30ms
void CALLBACK GraphicRendering(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2)
{
	COneDOFDlg * obj = (COneDOFDlg*) dwUser;
	obj->MMTimerGraphicRendering(wTimerID);
}

//触觉渲染多线程函数，1ms，由多媒体定时器调用
void COneDOFDlg::MMTimerHapticRendering(UINT nIDEvent)
{
// do what you want to do, but quickly

	DWORD temp= g_ForceIndex;
	DWORD temp1 = temp-1 ,temp2=temp-2, temp3=temp-3;

	if(  0 == temp)
	{
		temp1 = FORCE_BUFFER_SIZE-1;
		temp2 = FORCE_BUFFER_SIZE-2;
		temp3 = FORCE_BUFFER_SIZE-3;
	}

	if( 1 == temp )
		{
		temp1 = 0;
		temp2 = FORCE_BUFFER_SIZE-1;
		temp3 = FORCE_BUFFER_SIZE-2;
	}
	if( 2 == temp)
	{
		temp1 = 1;
		temp2 = 0;
		temp3 = FORCE_BUFFER_SIZE-1;
	}
	double dforce =   ( g_ForceBuffer[temp][2] + g_ForceBuffer[temp1][2] + g_ForceBuffer[temp2][2]+g_ForceBuffer[temp3][2]  - 4.0*m_Force0[2])/4.0;//人手作用力

	//两种模型公用变量
	static double dAngle = 0.0;//关节角度
	double v_angle = 0.0;
	const double RR = 180.0;//连杆长度R为180mm

	////x=kf模型
	//double kk = 1.0;
	//dAngle += dforce/kk/RR;//力除以刚度等于位移mm，dforce/kk/R弧度
	//if ( dAngle >= (30.0/180.0*PI) ) dAngle=30.0/180.0*PI;
	//if ( dAngle <= (-30.0/180.0*PI) ) dAngle=-30.0/180.0*PI;//正负10度以内
	//v_angle = 90.0/180.0*PI;

	//质量模型
	double m = 0.0;//虚拟质量kg
	static BOOL flag = FALSE;
	double k2 = 30.1*1000.0;
	double bb = 00.0;//阻尼
	if(flag)
	{
		m = 0.8;
		bb = 20.0;
	}
	else
	{
		m = 0.2;
		bb = 0.2;
	}
	m = 0.5;
	bb = 10.0;

	double FF = dforce + Fv - bb * vt_1;
	double a = FF/m;//加速度m/s2
	vt = vt_1 + a * T;//虚拟速度m/s
	//速度滤波
	vt = 0.9 * vt +  ( 1 - 0.9 ) * vt_1;

	vt_1 = vt;


	
	xt = xt_1 + vt * T;//目标位置m
	xt_1 = xt;

	dAngle = xt/(RR/1000.0);//目标角度，rad
	v_angle = vt/(RR/1000.0);//转角速度rad/s

	if ( dAngle >= (30.0/180.0*PI) ) dAngle=30.0/180.0*PI;
	if ( dAngle <= (-30.0/180.0*PI) ) dAngle=-30.0/180.0*PI;//正负10度以内
	xt_1 = dAngle * (RR/1000.0);

	if ( v_angle >= (360.0/180.0*PI) )  v_angle=360.0/180.0*PI;
	if ( v_angle <= (-360.0/180.0*PI) )  v_angle=-360.0/180.0*PI;
	vt_1 = v_angle * (RR/1000.0);

	Fv = 0.0;
	flag = FALSE;
	if( xt > 0.040)
	{
		Fv = - k2 * fabs( xt - 0.040);
		flag = TRUE;
	}
	if( xt < -0.040)
	{
		Fv =  k2 * fabs( xt + 0.040);
		flag = TRUE;
	}





	//发送目标位置速度
	//m_CANCmdFrame[0] = * m_CANCmd.GetFrame(MODE_POSITION, dAngle);
	//m_CANCmdFrame[1] = * m_CANCmd.GetFrame(MODE_VELOCITY, fabs(v_angle));//(360.0/180.0*PI)
	//m_CANCmdFrame[2] = * m_CANCmd.GetFrame(MODE_ENABLEMOTION);//使能
	//m_CANCmdFrame[3] = * m_CANCmd.GetFrame(MODE_STARTMOTION);//开始运行
	//Transmit(m_DeviceType,0,0,m_CANCmdFrame,4);//注意是4帧一起发送，发送一次，1-5帧的范围大概都是0.5ms

	CString str;
	str.Format(_T("角度：%.5f\t,速度：%.5f\t,dforce：%.5f\t"), dAngle/PI*180.0,v_angle/PI*180.0,dforce);
    //str.Format(_T("x：%.5f\tv：%.5f\ta:%.5f\tdforce：%.5f\tFv:%.5f"), xt,vt,a,dforce,Fv);
	GetDlgItem(IDC_EDIT1)->SetWindowText(str);


	m_progress.SetPos((int)fabs(dforce/10.0*100.0));
	

	fen++;
	if(fen==50)
	{
		fen=0;
		str.Format( _T("g_ComputingTime=%.5f\n,g_ForceIndex=%d"),	g_ComputingTime,g_ForceIndex );
		GetDlgItem(IDC_EDIT2)->SetWindowText(str);
		
	}
}



//视觉渲染,30ms，由多媒体定时器调用
void COneDOFDlg::MMTimerGraphicRendering(UINT nIDEvent)
{
// do what you want to do, but quickly

	DWORD temp;
	CString str;
	temp = g_DAQPositionIndex;
	str.Format(_T("%.2f,\n  %.2f,\n  %.2f,\n  %.2f,\n  %.2f,\n  %.2f,\n g_DAQPositionIndex:%d,\n   g_ComputingTime:%f,\n  接收帧数：%d,    pp:%f"),
					g_DAQPositionBuffer[g_DAQPositionIndex][0],
					g_DAQPositionBuffer[g_DAQPositionIndex][1],
					g_DAQPositionBuffer[g_DAQPositionIndex][2],
					g_DAQPositionBuffer[g_DAQPositionIndex][3],
					g_DAQPositionBuffer[g_DAQPositionIndex][4],
					g_DAQPositionBuffer[g_DAQPositionIndex][5],
					g_DAQPositionIndex,
					g_ComputingTime,
					aa,
					pp);

	temp= g_ForceIndex;
	//str.Format(_T("%.2f,\n  %.2f,\n  %.2f,\n  %.2f,\n  %.2f,\n  %.2f,\n g_DAQPositionIndex:%2d,\n   g_ComputingTime:%.2f,\n  接收帧数：%2d,    pp:%.2f"),
	//			g_ForceBuffer[temp][0]-m_Force0[0],
	//			g_ForceBuffer[temp][1]-m_Force0[1],
	//			g_ForceBuffer[temp][2]-m_Force0[2],
	//			g_ForceBuffer[temp][3]-m_Force0[3],
	//			g_ForceBuffer[temp][4]-m_Force0[4],
	//			g_ForceBuffer[temp][5]-m_Force0[5],
	//		g_ForceIndex,
	//		g_ComputingTime,
	//		aa,
	//		pp);

	str.Format( _T("%.2f\n"),g_ForceBuffer[temp][2]-m_Force0[2] );
	//g_ForceIndex
	GetDlgItem(IDC_EDIT2)->SetWindowText(str);


}

BOOL COneDOFDlg::ProcessForceData(DWORD len)//处理力传感器数据
{

	//***************处理数据******************
	//Head fram	  PackageLength	 DataNo	       Data	       ChkSum
	//0xAA,0x55	     HB,LB         2B   (ChNum*N*DNpCH) B	 1B		//(ChNum*N*DNpCH)为6*2*1=12B，所以共有19字节
	//BYTE g_ForceReadBuffer[FORCE_BUFFER_SIZE] = {0};	//初始读取力串口的数组
	//volatile double	g_ForceBuffer[FORCE_BUFFER_SIZE][6] = {0.0};//顺序循环保存Fx/FY/FZ/MX/MY/MZ
	//volatile DWORD	g_ForceIndex = 0;//当前g_ForceBuffer的索引

	if(g_ForceMyRxBuffer == NULL) {return FALSE;}
	if(g_DataPtrOut == 0xFFFFFFFF)	{return FALSE;}

	DWORD HeaderIndex =0x00;//指向0x55的索引，所有索引都从0开始
	BOOL DataHeaderFlag = FALSE;//头的标志位
	DWORD PackageLength= 0xFFFFFFFF; 
	DWORD HighIndex,LowIndex;
	DWORD RxLengthTemp = 0x00;
	DWORD RxCounterCurrent = g_MyRxBufferCounter;
	DWORD i = 0x00;

	while (1)
	{
		//--------------------------------------------------------------------------------	
		//数据长度，从g_DataPtrOut算起
		HeaderIndex = g_DataPtrOut;
		if(RxCounterCurrent >= HeaderIndex)
		{
			RxLengthTemp = RxCounterCurrent - HeaderIndex;
		}
		else
		{
			RxLengthTemp = FORCE_BUFFER_SIZE - HeaderIndex + RxCounterCurrent;
		}
		if(RxLengthTemp <= 9)
		{
			break;//退出while
		}
		//--------------------------------------------------------------------------------
		//寻找帧头
		for(i = 0x00;i < RxLengthTemp; i++)
		{	
			if(HeaderIndex == FORCE_BUFFER_SIZE )
			{
				HeaderIndex = 0x00;
			}
			if(HeaderIndex == FORCE_BUFFER_SIZE-1)
			{
				if((g_ForceMyRxBuffer[HeaderIndex] == 0xAA) && (g_ForceMyRxBuffer[0] == 0x55))
				{
					DataHeaderFlag = TRUE;
					break;
				}
			}
			else if((g_ForceMyRxBuffer[HeaderIndex] == 0xAA) && (g_ForceMyRxBuffer[HeaderIndex+1] == 0x55))
			{
				DataHeaderFlag = TRUE;
				break;
			}
			HeaderIndex++;	
		}
		if(DataHeaderFlag != TRUE)
		{
			break;
		}
		DataHeaderFlag = FALSE;
		//-------------------------------------------------------------------------------
		HighIndex = HeaderIndex+2;
		LowIndex = HeaderIndex+3;
		if(HighIndex >= FORCE_BUFFER_SIZE)
		{
			HighIndex = HighIndex - FORCE_BUFFER_SIZE;
		}
		if(LowIndex >= FORCE_BUFFER_SIZE)
		{
			LowIndex = LowIndex - FORCE_BUFFER_SIZE;
		}
		PackageLength = g_ForceMyRxBuffer[HighIndex]*256 + g_ForceMyRxBuffer[LowIndex];

		if(RxCounterCurrent >= HeaderIndex)
		{
			RxLengthTemp = RxCounterCurrent - HeaderIndex;
		}else
		{
			RxLengthTemp = FORCE_BUFFER_SIZE - HeaderIndex + RxCounterCurrent;
		}	
		if(RxLengthTemp < PackageLength + 4)
		{
			break;
		}
		//Now, HeaderIndex point to‘0xAA’
		//Head fram	  PackageLength	 DataNo	       Data	       ChkSum
		//0xAA,0x55	     HB,LB         2B   (ChNum*N*DNpCH) B	 1B
		//--------------------------------------------------------------------------------
		//Save data	in order
		DWORD MoveIndex = HeaderIndex+6;
		if(MoveIndex >=  FORCE_BUFFER_SIZE)
		{
			MoveIndex = MoveIndex -  FORCE_BUFFER_SIZE;
		}		
		BYTE CheckSum = 0x00;
		for(i = 0x00; i <  PackageLength-3; i++)
		{			
			CheckSum += g_ForceMyRxBuffer[MoveIndex];	
			MoveIndex++;
			if(MoveIndex == FORCE_BUFFER_SIZE)
			{
				MoveIndex = 0x00;
			}
		}
		if(CheckSum != g_ForceMyRxBuffer[MoveIndex])
		{
			g_DataPtrOut = MoveIndex;//指向checksum
			break;
		}
		//---------------------------------------------------------------------------------
		BYTE DataTemp[12] = {0};//临时保存数据，6通道，一个点
		MoveIndex = HeaderIndex+6;
		if(MoveIndex >= FORCE_BUFFER_SIZE)
		{
			MoveIndex = MoveIndex - FORCE_BUFFER_SIZE;
		}
		for(i = 0x00; i < PackageLength-3; i++)
		{			
			DataTemp[i] = g_ForceMyRxBuffer[MoveIndex++];				
			if(MoveIndex == FORCE_BUFFER_SIZE)
			{
				MoveIndex = 0x00;
			}
		}	
		g_DataPtrOut = MoveIndex;
		//---------------------------------------------------------------------------------
		//计算ADcounts
		INT ADCounts[6] = {0};//保存读取的AD
		for( i =0; i < 6; i++ )
		{
			ADCounts[i] = DataTemp[2*i]<<8 |  DataTemp[2*i+1];
		}
		//计算每个通道
		DOUBLE	ChValue[6]={0.0};
		for ( i = 0; i < 6; i++)
		{
			ChValue[i] = 1000.0 * ( ADCounts[i] - m_dAmpZero[i]) / (double)65535.0 * (double)5  / m_dChnGain[i] / m_dChnEx[i];  
		}
		//利用解耦矩阵计算力值
		DOUBLE ForceTemp[6] = {0.0};
		for( i = 0x00; i < 6; i++)
		{
			ForceTemp[i] =   ChValue[0]*m_dDecouplingCoefficient[i][0] + ChValue[1]*m_dDecouplingCoefficient[i][1]
									+ ChValue[2]*m_dDecouplingCoefficient[i][2] + ChValue[3]*m_dDecouplingCoefficient[i][3]
									+ ChValue[4]*m_dDecouplingCoefficient[i][4] + ChValue[5]*m_dDecouplingCoefficient[i][5];
		}
		//保存数据，g_ForceIndex始终指向最新一组数据
		DWORD ForceNextIndex = g_ForceIndex + 1;//指向下一个存储位置，注意g_ForceIndex定义
		if( ForceNextIndex >= FORCE_BUFFER_SIZE)
			ForceNextIndex = ForceNextIndex - FORCE_BUFFER_SIZE;//注意循环保存的索引

		for( i = 0; i <6; i++)
		{
			g_ForceBuffer[ForceNextIndex][i] = ForceTemp[i];//保存数据
		
		}
		g_ForceIndex = ForceNextIndex;//保存有效后，指向下一个有效数据，其他线程通过g_ForceIndex访问数据
		
		//CString str;
		//str.Format( _T("g_ComputingTime=%.5f\n,g_ForceIndex=%d"),	g_ComputingTime, ForceNextIndex );
		//GetDlgItem(IDC_EDIT2)->SetWindowText(str);
	}
	return TRUE;
}



/**以下为力传感器和DAQ串口接收程序******************************************/
LRESULT  COneDOFDlg::OnComRecvData(WPARAM str, LPARAM commInfo)
{
	struct serialPortInfo
	{
		UINT portNr;//串口号
		DWORD bytesRead;//读取的字节数
	}*pCommInfo;
	pCommInfo = (serialPortInfo*)commInfo;

	if (pCommInfo->portNr == m_ComPortNumForce)//接收力传感器信号
	{
		////*******用于计算两个数据之间的时间间隔*******
		//static int aaa = 0;
		//static LARGE_INTEGER nFreq;
		//static LARGE_INTEGER nBeginTime;
		//static LARGE_INTEGER nEndTime;
		//static double time;

		//if(0 == aaa)
		//{
		//	QueryPerformanceFrequency(&nFreq);
		//	QueryPerformanceCounter(&nBeginTime);//测试程序运行时间
		//}

		//aaa++;

		//if(2 == aaa)
		//{
		//	QueryPerformanceCounter(&nEndTime);
		//	time=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
		//	g_ComputingTime = time * 1000.0;//运行时间为0.01ms
		//	aaa=0;
		//}
		////********************************************
		

		////*******************测试运行时间**************
		//LARGE_INTEGER nFreq;
		//LARGE_INTEGER nBeginTime;
		//LARGE_INTEGER nEndTime;
		//double time;
		//QueryPerformanceFrequency(&nFreq);
		//QueryPerformanceCounter(&nBeginTime);//测试程序运行时间
		////*******************************************

		DWORD len = pCommInfo->bytesRead;
		DWORD i = 0x00;
		//读取原始数据
		memcpy(g_ForceReadBuffer,(BYTE*)str,len);

		for( i =0; i < len; i++)
		{
			if(g_MyRxBufferCounter >= FORCE_BUFFER_SIZE)
			{
				g_MyRxBufferCounter = 0;
			}
			g_ForceMyRxBuffer[g_MyRxBufferCounter++] =  g_ForceReadBuffer[i];//保存原始数据，g_MyRxBufferCounter最后指向数据的下一个位
		}

		ProcessForceData(len);//处理力传感器数据
	}


	if(pCommInfo->portNr == m_ComPortNumDAQ)//接收DAQ信号
	{
	
		////*******************测试运行时间**************
		//LARGE_INTEGER nFreq;
		//LARGE_INTEGER nBeginTime;
		//LARGE_INTEGER nEndTime;
		//double time;
		//QueryPerformanceFrequency(&nFreq);
		//QueryPerformanceCounter(&nBeginTime);//测试程序运行时间
		////*******************************************

		DWORD len = pCommInfo->bytesRead;
		DWORD HeaderIndex =0x00;//指向0x55的索引，所有索引都从0开始
		DWORD i = 0x00;

		memcpy(g_DAQReadBuffer,(BYTE*)str,len);

		aa = len;

		//姿态值、码盘值一帧（共22B,耗时0.3ms,带宽满足） 0x55 0xaa Lenth(1B) Ecd1(4B) Ecd2(4B) Ecd3(4B) R(2B) P(2B) Y(2B) CheckSum(1B)
		//字节数校验
		if( len <= 21 )//如果总长小于22字节返回
			return -1;
		//AfxMessageBox(_T("len <= 21"));
		//搜索头部
		for( i=0; i<(len-1); i++)
		{
			if( g_DAQReadBuffer[i] == 0x55 && g_DAQReadBuffer[i+1] == 0xAA )
			{
				HeaderIndex = i;//此时i为头
				break;
			}
			if( i == (len-2) )
				return -1;//找不到头
		}
		//AfxMessageBox(_T("搜索头部"));
		//从HeaderIndex到索引len-1（即数据最有一个字节）必须为>22个字节
		if( (len-HeaderIndex) <= 21 )
			return -1;
		//AfxMessageBox(_T("len-HeaderIndex) <= 21 "));
		//长度Length(1B)校验

		////CheckSum(1B)校验
		//DWORD CheckSum =0;
		//for( i = (HeaderIndex+3); i < (HeaderIndex+21); i++)
		//{
		//	CheckSum += g_DAQReadBuffer[i];
		//}
		//if( CheckSum != g_DAQReadBuffer[HeaderIndex+21])//校验和不对
		//	return -1;
		//保存数据
		DWORD32 TempCountsEncoder = 0;//保存编码器整数
		DWORD TempRPY = 0;//保存RPY缓存
		DOUBLE PositionTemp[6] = {0.0};//按顺序保存ABC编码器和RPY角度

		TempCountsEncoder = (g_DAQReadBuffer[3+HeaderIndex]<<24) | (g_DAQReadBuffer[4+HeaderIndex]<<16)
							| (g_DAQReadBuffer[5+HeaderIndex]<<8) | g_DAQReadBuffer[6+HeaderIndex];
		PositionTemp[0] = TempCountsEncoder / COUNTS_ENCODER * 2* PI;//A弧度

		TempCountsEncoder = (g_DAQReadBuffer[7+HeaderIndex]<<24) | (g_DAQReadBuffer[8+HeaderIndex]<<16)
							| (g_DAQReadBuffer[9+HeaderIndex]<<8) | g_DAQReadBuffer[10+HeaderIndex];
		PositionTemp[1] = TempCountsEncoder / COUNTS_ENCODER * 2* PI;//B弧度

		TempCountsEncoder = (g_DAQReadBuffer[11+HeaderIndex]<<24) | (g_DAQReadBuffer[12+HeaderIndex]<<16)
							| (g_DAQReadBuffer[13+HeaderIndex]<<8) | g_DAQReadBuffer[14+HeaderIndex];
		PositionTemp[2] = TempCountsEncoder / COUNTS_ENCODER * 2* PI;//C弧度

		TempRPY = (g_DAQReadBuffer[15+HeaderIndex]<<8) | g_DAQReadBuffer[16+HeaderIndex];
		PositionTemp[3] = TempRPY / 32768.0f * PI;//R弧度

		TempRPY = (g_DAQReadBuffer[17+HeaderIndex]<<8) | g_DAQReadBuffer[18+HeaderIndex];
		PositionTemp[4] = TempRPY / 32768.0f * PI;//R弧度

		TempRPY = (g_DAQReadBuffer[19+HeaderIndex]<<8) | g_DAQReadBuffer[20+HeaderIndex];
		PositionTemp[5] = TempRPY / 32768.0f * PI;//R弧度

		//g_DAQPositionIndex始终指向最新一组数据
		DWORD NextIndex = g_DAQPositionIndex + 1;//指向下一个存储位置，注意g_DAQPositionBuffer定义
		if( NextIndex >= DAQ_BUFFER_SIZE)
			NextIndex = NextIndex - DAQ_BUFFER_SIZE;//注意循环保存的索引

		for( i = 0; i <6; i++)
		{
			g_DAQPositionBuffer[NextIndex][i] = PositionTemp[i];//保存数据
		
		}
		g_DAQPositionIndex = NextIndex;//保存有效后，指向下一个有效数据，其他线程通过g_DAQPositionIndex访问数据

		//AfxMessageBox(_T("校验和不对 "));
		////数据处理和保存
		//BYTE aa[18] = {0};
		//for( i=0; i< 18; i++)
		//{
		//	aa[i] = g_DAQReadBuffer[i+HeaderIndex+3];
		//}

		//m_ComPortForce.Write(aa,18);



	////*******************获取运行时间**************
	//QueryPerformanceCounter(&nEndTime);
	//time=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
	//g_ComputingTime = time * 1000.0;
	////*******************************************


	}
	return 0;
}


/***以下为消息响应函数************************************************************/
//连接力传感器
void COneDOFDlg::OnBnClickedBtnConnectForce()
{
	// TODO: 在此添加控件通知处理程序代码
	CString str;
	GetDlgItem(IDC_BTN_CONNECT_FORCE)->GetWindowText(str);

	if( m_bFlagForceConnect == FALSE )
	{
		if(!m_ComPortForce.InitPort(*this,m_ComPortNumForce,1000000))	//如果打开串口失败
		{	
			AfxMessageBox(_T("连接力传感器失败！"));
			return;
		}
		else//打开成功
		{
			m_bFlagForceConnect = TRUE;
			m_ComPortForce.StartMonitoring();
			GetDlgItem(IDC_BTN_CONNECT_FORCE)->SetWindowText(_T("断开力传感器"));
			AfxMessageBox(_T("连接力传感器成功！"));
			Sleep(50);//使力传感器保存一定数量的初始值

		}
	}
	else
	{
		//结束触觉和视觉渲染
		timeKillEvent(g_TimerVR);
		timeKillEvent(g_TimerHR);
		timeEndPeriod (g_uResolution);

		CString CmdStr = _T("AT+GSD=STOP\r\n");
		LPTSTR pszTest=CmdStr.GetBuffer(CmdStr.GetLength());
		USES_CONVERSION;
		CHAR * pChar = W2A(pszTest);
		INT len = strlen(pChar);
		m_ComPortForce.WriteToPort(pChar,len);
		Sleep(50);

		m_ComPortForce.ClosePort(); //关闭串口
		m_bFlagForceConnect = FALSE;
		GetDlgItem(IDC_BTN_CONNECT_FORCE)->SetWindowText(_T("连接力传感器"));
		GetDlgItem(IDC_BTN_START)->SetWindowText(_T("开始"));
	}

}

//配置力传感器
void COneDOFDlg::OnBnClickedBtnConfigureForce()
{
	// TODO: 在此添加控件通知处理程序代码
	CString	CmdStr;
	LPTSTR  pszTest;
	CHAR *pChar;
	INT len;

	if(m_bFlagForceConnect == FALSE)
	{
		AfxMessageBox(_T("请打开力传感器串口！"));
		return;
	}
	//得到实时数据步骤：
	//第一步：设置矩阵AT+DCPM=。。。（提前已设置）
	//Set the decoupling matrix unit by command DCPCU，（提前已设置）
	//Send command AT+SGDM=(A01,A02,A03,A04,A05,A06);C;1;(WMA:1)\r\n to set the mode to receive data.
	//AT+SMPR=1000\r\n	//配置采样频率
	//AT+GSD\r\n循环接收


	CmdStr = _T("AT+SMPR=1000\r\n");	//配置采样频率
	pszTest=CmdStr.GetBuffer(CmdStr.GetLength());
	USES_CONVERSION;
	pChar = W2A(pszTest);
	len = strlen(pChar);
	m_ComPortForce.WriteToPort(pChar,len);

	Sleep (50);

	CmdStr = _T("AT+SGDM=(A01,A02,A03,A04,A05,A06);C;1;(WMA:1)\r\n");  //Set mode of receiving data.
	pszTest=CmdStr.GetBuffer(CmdStr.GetLength());
	pChar = W2A(pszTest);
	len = strlen(pChar);
	//m_ComPortForce.Write(pChar,len);
	m_ComPortForce.WriteToPort(pChar,len);
	Sleep (50);

	CmdStr = _T("AT+GSD\r\n");  //Set mode of receiving data.
	pszTest=CmdStr.GetBuffer(CmdStr.GetLength());
	pChar = W2A(pszTest);
	len = strlen(pChar);
	m_ComPortForce.WriteToPort(pChar,len);

	Sleep (50);


	CmdStr.ReleaseBuffer();

	AfxMessageBox(_T("配置成功！"));

	GetDlgItem(IDC_BTN_CONFIGURE_FORCE)->EnableWindow(FALSE);

	m_bFlagInitForce = TRUE;

	//m_CANCmdFrame[0] = * m_CANCmd.GetFrame(MODE_SETPOSITIONZERO);//!!!不要删除，设置电机当前位置为0，调试正确
	//Transmit(m_DeviceType,0,0,m_CANCmdFrame,1);

}

//打开DAQ串口
void COneDOFDlg::OnBnClickedBtnOpenDaqPort()
{
	//// TODO: 在此添加控件通知处理程序代码
	//CString str;
	//GetDlgItem(IDC_BTN_OPEN_DAQ_PORT)->GetWindowText(str);
	//if( m_bFlagDAQConnect == FALSE ) //( str == _T("打开DAQ串口") )
	//{
	//	if(!m_ComPortDAQ.InitPort(*this,m_ComPortNumDAQ,1000000))	//如果打开串口失败
	//	{	
	//		AfxMessageBox(_T("DAQ串口不存在或被占用！"));
	//		return;
	//	}
	//	else//打开成功
	//	{

	//		m_bFlagDAQConnect = TRUE;
	//		m_ComPortDAQ.StartMonitoring();
	//		GetDlgItem(IDC_BTN_OPEN_DAQ_PORT)->SetWindowText(_T("关闭DAQ串口"));

	//		Sleep(50);//使DAQ保存一定数量的初始值
	//	}
	//}
	//else// if ( m_ComPortDAQ.IsOpen() )//( str == _T("关闭DAQ串口") )//关闭串口
	//{
	//	//结束触觉和视觉渲染
	//	timeKillEvent(g_TimerVR);
	//	timeKillEvent(g_TimerHR);
	//	timeEndPeriod (g_uResolution);

	//	m_ComPortDAQ.ClosePort(); //关闭串口
	//	m_bFlagDAQConnect = FALSE;
	//	GetDlgItem(IDC_BTN_OPEN_DAQ_PORT)->SetWindowText(_T("打开DAQ串口"));
	//	GetDlgItem(IDC_BTN_START)->SetWindowText(_T("开始"));
	//}



	//m_CANCmdFrame[0] = * m_CANCmd.GetFrame(MODE_SETPOSITIONZERO);//!!!不要删除，设置电机当前位置为0，调试正确
	//Transmit(m_DeviceType,0,0,m_CANCmdFrame,1);

	//m_CANCmdFrame[0] = * m_CANCmd.GetFrame(MODE_POSITION, 100000.0/180.0*PI);
	//m_CANCmdFrame[1] = * m_CANCmd.GetFrame(MODE_VELOCITY, 180.0/180.0*PI);
	//m_CANCmdFrame[2] = * m_CANCmd.GetFrame(MODE_ENABLEMOTION);//使能
	//m_CANCmdFrame[3] = * m_CANCmd.GetFrame(MODE_STARTMOTION);//开始运行
	//Transmit(m_DeviceType,0,0,m_CANCmdFrame,4);//注意是4帧一起发送，发送一次，1-5帧的范围大概都是0.5ms


}

//打开CAN串口
void COneDOFDlg::OnBnClickedBtnOpenCanPort()
{
	// TODO: 在此添加控件通知处理程序代码
	if( m_bFlagCanConnect == TRUE)//要关闭CAN
	{
		//结束触觉和视觉渲染
		timeKillEvent(g_TimerVR);
		timeKillEvent(g_TimerHR);
		timeEndPeriod (g_uResolution);

		m_bFlagCanConnect = FALSE;
		GetDlgItem(IDC_BTN_OPEN_CAN_PORT)->SetWindowText(_T("打开CAN串口"));
		GetDlgItem(IDC_BTN_START)->SetWindowText(_T("开始"));
		CloseDevice(m_DeviceType,0);

		return;
	}

	INIT_CONFIG init_config;//CAN初始化配置结构体
	init_config.AccCode = 0;
    init_config.AccMask =0xffffff;
    init_config.Filter = 0;
	init_config.Timing0 = 0;
    init_config.Timing1 =0x14;//波特率为10000kHZ
	init_config.Mode = 0;

	if( OpenDevice(m_DeviceType,0,0) != STATUS_OK )
	{
		MessageBox(_T("打开CAN串口失败！"),_T("警告"),MB_OK|MB_ICONQUESTION);
		return;
	}

	//MessageBox(_T("打开CAN串口成功！"),_T("警告"),MB_OK|MB_ICONQUESTION);

	if( InitCAN(m_DeviceType,0,0,&init_config) != STATUS_OK )
	{
		MessageBox(_T("初始化CAN串口失败！"),_T("警告"),MB_OK|MB_ICONQUESTION);
		CloseDevice(m_DeviceType,0);
		return;
	}
	//MessageBox(_T("初始化CAN串口成功！"),_T("警告"),MB_OK|MB_ICONQUESTION);

	//AfxBeginThread(ReceiveThread,this);//如果需要接收数据，启动线程

	if( StartCAN(m_DeviceType,0,0) == 1 )
	{
		MessageBox(_T("启动CAN成功！"),_T("警告"),MB_OK|MB_ICONQUESTION);
		m_bFlagCanConnect = TRUE;		//标志位
		GetDlgItem(IDC_BTN_OPEN_CAN_PORT)->SetWindowText(_T("断开CAN串口"));
	}
	else
	{
		MessageBox(_T("启动CAN失败！"),_T("警告"),MB_OK|MB_ICONQUESTION);	
		CloseDevice(m_DeviceType,0);
	}

	Sleep(50);

}

//初始化驱动器
void COneDOFDlg::OnBnClickedBtnInitialDrive()
{
	//TODO: 在此添加控件通知处理程序代码
	//if( m_bFlagCanConnect == FALSE)
	//{
	//	MessageBox(_T("请先打开CAN串口！"),_T("警告"),MB_OK|MB_ICONQUESTION);
	//	return;
	//}

	//m_CANCmdFrame[0] = * m_CANCmd.GetFrame(MODE_STARTCAN);//获取启动帧，第一步启动can
	//m_CANCmdFrame[1] = * m_CANCmd.GetFrame(MODE_CANMODE);//第二步，can模式
	//m_CANCmdFrame[2] = * m_CANCmd.GetFrame(MODE_OPERATIONMODE);//第三步，位置模式
	//m_CANCmdFrame[3] = * m_CANCmd.GetFrame(MODE_PROFILETYPE);//第四步，梯形模式
	//m_CANCmdFrame[4] = * m_CANCmd.GetFrame(MODE_ENABLEMOTION);//第五步，使能

	//if( 5 == Transmit(m_DeviceType,0,0,m_CANCmdFrame,5) )
	//{
	//	m_CANCmdFrame[0] = * m_CANCmd.GetFrame(MODE_SETPOSITIONZERO);//!!!不要删除，设置电机当前位置为0，调试正确
	//	Transmit(m_DeviceType,0,0,m_CANCmdFrame,1);
	//	MessageBox(_T("初始化驱动器完成！"),_T("警告"),MB_OK|MB_ICONQUESTION);
	//	GetDlgItem(IDC_BTN_INITIAL_DRIVE)->EnableWindow(FALSE);
	//	m_bFlagInitDrive = TRUE;
	//}
	//else
	//	MessageBox(_T("初始化驱动器失败！"),_T("警告"),MB_OK|MB_ICONQUESTION);

	if( m_bFlagCanConnect == FALSE)
	{
		MessageBox(_T("请先打开CAN串口！"),_T("警告"),MB_OK|MB_ICONQUESTION);
		return;
	}

	m_CANCmdFrame[0] = * m_CANCmd.GetFrame(0, MODE_STARTCAN);//获取启动帧，第一步启动can
	Transmit(m_DeviceType,0,0,m_CANCmdFrame,1);

	m_CANCmdFrame[1] = * m_CANCmd.GetFrame(1, MODE_CANMODE);//第二步，can模式
	m_CANCmdFrame[2] = * m_CANCmd.GetFrame(2, MODE_CANMODE);//第二步，can模式
	m_CANCmdFrame[3] = * m_CANCmd.GetFrame(3, MODE_CANMODE);//第二步，can模式

	m_CANCmdFrame[4] = * m_CANCmd.GetFrame(1, MODE_OPERATIONMODE);//第三步，位置模式
	m_CANCmdFrame[5] = * m_CANCmd.GetFrame(2, MODE_OPERATIONMODE);//第三步，位置模式
	m_CANCmdFrame[6] = * m_CANCmd.GetFrame(3, MODE_OPERATIONMODE);//第三步，位置模式

	m_CANCmdFrame[7] = * m_CANCmd.GetFrame(1, MODE_PROFILETYPE);//第四步，梯形模式
	m_CANCmdFrame[8] = * m_CANCmd.GetFrame(2, MODE_PROFILETYPE);//第四步，梯形模式
	m_CANCmdFrame[9] = * m_CANCmd.GetFrame(3, MODE_PROFILETYPE);//第四步，梯形模式

	m_CANCmdFrame[10] = * m_CANCmd.GetFrame(1, MODE_ENABLEMOTION);//第五步，使能
	m_CANCmdFrame[11] = * m_CANCmd.GetFrame(2, MODE_ENABLEMOTION);//第五步，使能
	m_CANCmdFrame[12] = * m_CANCmd.GetFrame(3, MODE_ENABLEMOTION);//第五步，使能

	if( 13 == Transmit(m_DeviceType,0,0,m_CANCmdFrame,13) )
	{
		m_CANCmdFrame[0] = * m_CANCmd.GetFrame(1, MODE_SETPOSITIONZERO);//!!!不要删除，设置电机当前位置为0，调试正确
		m_CANCmdFrame[1] = * m_CANCmd.GetFrame(2, MODE_SETPOSITIONZERO);//!!!不要删除，设置电机当前位置为0，调试正确
		m_CANCmdFrame[2] = * m_CANCmd.GetFrame(3, MODE_SETPOSITIONZERO);//!!!不要删除，设置电机当前位置为0，调试正确

		Transmit(m_DeviceType,0,0,m_CANCmdFrame,3);
		MessageBox(_T("初始化驱动器完成！"),_T("警告"),MB_OK|MB_ICONQUESTION);
		GetDlgItem(IDC_BTN_INITIAL_DRIVE)->EnableWindow(FALSE);
		m_bFlagInitDrive = TRUE;
	
	
		m_CANCmdFrame[0] = * m_CANCmd.GetFrame(1, MODE_POSITION, 1000);
		m_CANCmdFrame[1] = * m_CANCmd.GetFrame(2, MODE_POSITION, 1000);
		m_CANCmdFrame[2] = * m_CANCmd.GetFrame(3, MODE_POSITION, 1000);
		m_CANCmdFrame[3] = * m_CANCmd.GetFrame(1, MODE_VELOCITY, (60.0/180.0*PI));//(360.0/180.0*PI)
		m_CANCmdFrame[4] = * m_CANCmd.GetFrame(2, MODE_VELOCITY, (60.0/180.0*PI));//(360.0/180.0*PI)
		m_CANCmdFrame[5] = * m_CANCmd.GetFrame(3, MODE_VELOCITY, (60.0/180.0*PI));//(360.0/180.0*PI)
		m_CANCmdFrame[6] = * m_CANCmd.GetFrame(1, MODE_ENABLEMOTION);//使能
		m_CANCmdFrame[7] = * m_CANCmd.GetFrame(2, MODE_ENABLEMOTION);//使能
		m_CANCmdFrame[8] = * m_CANCmd.GetFrame(3, MODE_ENABLEMOTION);//使能
		m_CANCmdFrame[9] = * m_CANCmd.GetFrame(1, MODE_STARTMOTION);//开始运行
		m_CANCmdFrame[10] = * m_CANCmd.GetFrame(2, MODE_STARTMOTION);//开始运行
		m_CANCmdFrame[11] = * m_CANCmd.GetFrame(3, MODE_STARTMOTION);//开始运行
		Transmit(m_DeviceType,0,0,m_CANCmdFrame,12);//注意是4帧一起发送，发送一次，1-5帧的范围大概都是0.5ms
	
	
	}
	else
		MessageBox(_T("初始化驱动器失败！"),_T("警告"),MB_OK|MB_ICONQUESTION);





}

//开始程序，即打开视觉和触觉多媒体定时器
void COneDOFDlg::OnBnClickedBtnStart()
{
	// TODO: 在此添加控件通知处理程序代码
	
	//if( m_bFlagRunned == TRUE )	//如果已经运行过一次
	//{
	//	MessageBox(_T("必须重启电源、重启程序！"),_T("警告"),MB_OK|MB_ICONQUESTION);
	//	return;
	//} 

	////判断端口是否都打开
	//if( !m_ComPortForce.IsOpen() )
	//{
	//	MessageBox(_T("力传感器串口没有打开！"),_T("警告"),MB_OK|MB_ICONQUESTION);
	//	return;
	//} 
	//if( !m_ComPortDAQ.IsOpen() )
	//{
	//	MessageBox(_T("DAQ串口没有打开！"),_T("警告"),MB_OK|MB_ICONQUESTION);
	//	return;
	//}
	//if( !m_CanConnectFlag )
	//{
	//	MessageBox(_T("CAN串口没有打开！"),_T("警告"),MB_OK|MB_ICONQUESTION);
	//	return;
	//}
	//
	//if( ! m_bFlagInitDrive )
	//{
	//	MessageBox(_T("驱动器没有初始化！"),_T("警告"),MB_OK|MB_ICONQUESTION);
	//	return;
	//}

	//if( ! m_bFlagInitForce )
	//{
	//	MessageBox(_T("力传感器没有配置！"),_T("警告"),MB_OK|MB_ICONQUESTION);
	//	return;
	//}




	//初始化程序变量，如连杆初始位置、驱动器初始位置、力传感初始值，包括软件和硬件，
	//主要防止重启程序不重新供电造成的错误。
	//程序运行有两种情况，1，程序运行过程中若重启，必须重启硬件电源，即每次运行程序（包括点击停止再开始）前都要重启电源
	//2、以当前点击按钮时的状态为初始状态，力传感器有一个初始力F0，连杆有一个初始角度E0，电机应该也有一个位置M0
	//（但是无法获得，所以最好重启电源或者有一条指令设置电机位置为零），之后所有的计算都是基于这个计算

	//初始化代码

	INT temp= g_ForceIndex;				//以当前的力为0点
	m_Force0[0] = g_ForceBuffer[temp][0];
	m_Force0[1] = g_ForceBuffer[temp][1];
	m_Force0[2] = g_ForceBuffer[temp][2];
	m_Force0[3] = g_ForceBuffer[temp][3];
	m_Force0[4] = g_ForceBuffer[temp][4];
	m_Force0[5] = g_ForceBuffer[temp][5];




	//开始触觉渲染
	CString str;
	GetDlgItem(IDC_BTN_START)->GetWindowText(str);
	if ( m_bTimerRunning == FALSE) //( str == _T("开始") )
	{
		//提示用户，确保程序运行前重启硬件
		static WORD confirm = 0;
		confirm++;
		if ( confirm == 1 )
		{
			MessageBox(_T("确保硬件已重启！！"),_T("警告"),MB_OK|MB_ICONQUESTION);
			return;
		}
		confirm = 0;

		//启动多媒体定时器
		TIMECAPS tc;
		if( timeGetDevCaps(&tc, sizeof(TIMECAPS)) == TIMERR_NOERROR )//获取本机的最大分辨率
		{
			g_uResolution = min(max(tc.wPeriodMin, 0), tc.wPeriodMax);//为最大值
			timeBeginPeriod(g_uResolution);//设置分辨率
		}
		g_TimerHR = timeSetEvent(
								g_uDelayHaptic,
								g_uResolution,
								HapticRendering,
								(DWORD)this,
								TIME_PERIODIC);//先启动GR时点击文本框会卡死，加上带有返回值的can发送指令，会卡死

	/*	g_TimerVR = timeSetEvent(
								g_uDelayGraphic,
								g_uResolution,
								GraphicRendering,
								(DWORD)this,
								TIME_PERIODIC);*/
		
		GetDlgItem(IDC_BTN_START)->SetWindowText(_T("停止"));
		 m_bTimerRunning = TRUE;
	}
	else if ( m_bTimerRunning == TRUE )//(str == _T("停止"))
	{
		timeKillEvent(g_TimerVR);
		timeKillEvent(g_TimerHR);
		timeEndPeriod (g_uResolution);

		m_bTimerRunning = FALSE;

		GetDlgItem(IDC_BTN_START)->SetWindowText(_T("开始"));

		m_bFlagInitDrive = FALSE;
		m_bFlagInitForce = FALSE;

		m_bFlagRunned = TRUE;
	}


}

void COneDOFDlg::OnDestroy()
{
	CDialogEx::OnDestroy();
	// TODO: 在此处添加消息处理程序代码
	//多媒体定时器清理
	timeKillEvent(g_TimerVR);
	timeKillEvent(g_TimerHR);
	timeEndPeriod (g_uResolution);

	//串口清理
	if(m_ComPortDAQ.IsOpen()) m_ComPortDAQ.ClosePort(); //关闭串口
	if(m_ComPortCAN.IsOpen()) m_ComPortCAN.ClosePort(); //关闭串口
	if(m_ComPortForce.IsOpen()) m_ComPortForce.ClosePort(); //关闭串口


}

