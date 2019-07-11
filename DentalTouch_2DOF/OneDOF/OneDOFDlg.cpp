
// OneDOFDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "OneDOF.h"
#include "OneDOFDlg.h"
#include "afxdialogex.h"
#include "math.h"  
#include "FiveBarKinematics.h"
#include <gl\gl.h>
#include <gl\glu.h>
#include <gl\glut.h>  

#include "lodepng.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

using namespace std;

CStdioFile myfile;
typedef struct MyStruct
{
	double x;
	double y;
	double f;
}Friction;
typedef struct MyStruct2
{
	double deta;
	double f;
}Stiffness;

int frictionflag = 0;//0表示没开始，1表示开始记录，2表示停止记录

Friction myfriction;
vector<Friction> myvec;

Stiffness mystiffness;
vector<Stiffness> myvecstiffness;

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

//用于力反馈模式，上一次在约束空间
static BOOL bFirst_in_Wall = FALSE;

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
	#define MODE_STARTCAN			0
	#define MODE_CANMODE			1
	#define MODE_OPERATIONMODE		2
	#define MODE_PROFILETYPE		3
	#define MODE_ENABLEMOTION		4
	#define MODE_STARTMOTION		5
	#define MODE_POSITION			6
	#define MODE_VELOCITY			7
	#define MODE_ACCELERATION		8
	#define MODE_DECELERATION		9
	#define	MODE_SETPOSITIONZERO	10 
	#define MODE_ENABLEPDO			11
	#define	MODE_ENABLEALLAXIS		12
	#define MODE_EXECUTEALLMOVING	13
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
	, m_RadioBtn(0)
{
	INT i,j;
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

	m_ComPortNumForce = 8;	//力传感器串口号
	m_ComPortNumDAQ = 4;//DAQ串口号

	m_bTimerRunning = FALSE;//多媒体定时没有运行

	m_bFlagInitDrive = FALSE;//驱动器初始化标志位
	m_bFlagInitForce = FALSE;//力传感器初始化标志位
	m_bFlagInitDAQ = FALSE;//力传感器初始化标志位

	m_bFlagRunned = FALSE;//已经运行过？


	m_bFlagCanConnect= FALSE;	//连接标志位
	m_bFlagForceConnect= FALSE;	//连接标志位
	m_bFlagDAQConnect= FALSE;	//连接标志位


	DOUBLE temp1[6] = { 32716.000000, 32764.000000, 32761.000000,32800.000000,32683.000000,32672.000000};
	DOUBLE temp2[6] = { 123.080090,	123.213244,	123.044366,	123.158034,	123.054109,	123.115814};
	DOUBLE temp3[6] = { 2.515955,	2.515955,	2.515955,	2.515955,	2.515955,	2.515955};
	
	//力传感器参数初始化
	for( i = 0; i < 6; i++ )
	{
		m_dAmpZero[i] = temp1[i]; 
		m_dChnGain[i] = temp2[i];
		m_dChnEx[i] = temp3[i];
		m_Read_ForceSensor0[i] = 0.0;
	}

	//控制变量
	for( i = 0; i < POSITION_DIMENTION; i++ )
	{
		m_Position_HapticTool[i] = 0.0;
		m_Cartesian_Force[i] = 0.0;
	}
	for( i = 0; i < NUMBER_JOINTS; i++ )
	{
		m_Cmd_DPCAngle[i] = 0.0;
		m_Read_LinkAngle[i] = 0.0;
	}
	for( i = 0; i < POSITION_DIMENTION; i++ )
	{
		for( j = 0; j < NUMBER_JOINTS; j++ )
		{
			//m_Pose_HapticTool[i][j] = 0.0;
			m_JacobianMatrix[i][j] = 0.0;
		}
	}
	m_bOnlyFree	= TRUE;		//只跟随
	m_bInitValue = FALSE;		//初始化变量

	m_udCounterCtrl = NULL;
	m_udCounterCtrl = UdCounterCtrl::Create();
	//memset(&m_confParam, 0, sizeof(m_confParam));

	m_confParam.deviceNumber = 0;
	m_confParam.channel = 0;
	m_confParam.cntType =((	SignalCountingType)AbPhaseX2);
	m_confParam.deviceNumber = 0;

	m_fXAngle = 0.0;
	m_fYAngle = 0.0;
	m_fDist = 800.0f; 
	m_nState=0;
	m_SphereX = 0.0;
	m_SphereY = 0.0;
	m_y0 = 115.0 - 40.0;
	m_theta = 0.0;

	m_circleX = 0.0;
	m_circleY = 35.0;
	m_circleR = 80.0; 

	//m_circleX = 0.0;
	//m_circleY = 175.0;
	//m_circleR = 80.0; 

	myfile.Open(_T("data.txt"), CFile::modeCreate | CFile::modeWrite | CFile::typeText);

}

void COneDOFDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_EDIT1, m_Edit1);
	DDX_Control(pDX, IDC_PROGRESS1, m_progress);
	DDX_Radio(pDX, IDC_RADIO1, m_RadioBtn);
	DDX_Control(pDX, IDC_PIC, m_pic);
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

	ON_BN_CLICKED(IDC_RADIO1, &COneDOFDlg::OnBnClickedRadio1)
	ON_BN_CLICKED(IDC_RADIO2, &COneDOFDlg::OnBnClickedRadio1)
	ON_WM_CREATE()
	ON_WM_ERASEBKGND()
	ON_WM_KEYDOWN()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_MOUSEHWHEEL()
	ON_WM_MOUSEMOVE()
	ON_WM_SIZE()
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON1, &COneDOFDlg::OnBnClickedButton1)
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
	// initialize OpenGL
   //CClientDC dc(this);
   CClientDC dc(GetDlgItem(IDC_PIC));
	PIXELFORMATDESCRIPTOR pfd = 
	{
		sizeof(PIXELFORMATDESCRIPTOR),	// size of this pfd
		1,								// version number
		PFD_DRAW_TO_WINDOW |			// support window
		PFD_SUPPORT_OPENGL |			// support OpenGL
		PFD_DOUBLEBUFFER,				// double buffered
		PFD_TYPE_RGBA,					// RGBA type
		24,								// 24-bit color depth
		0, 0, 0, 0, 0, 0,				// color bits ignored
		0,								// no alpha buffer
		0,								// shift bit ignored
		0,								// no accumulation buffer
		0, 0, 0, 0,						// accum bits ignored
		32,								// 32-bit z-buffer
		0,								// no stencil buffer
		0,								// no auxiliary buffer
		PFD_MAIN_PLANE,					// main layer
		0,								// reserved
		0, 0, 0							// layer masks ignored
	};
	int pixelformat = ChoosePixelFormat(dc.GetSafeHdc(), &pfd);		//选择像素格式
	if (pixelformat == 0 )
	{
		MessageBox(_T("ChoosePixelFormat failed"));
		return false;
	}

	DescribePixelFormat(dc.GetSafeHdc(), pixelformat, sizeof(PIXELFORMATDESCRIPTOR), &pfd); //返回真正的像素格式
	if (!SetPixelFormat(dc.GetSafeHdc(), pixelformat, &pfd))//设置像素格式
	{
		MessageBox(_T("SetPixelFormat failed"));
		return false;
	}

	m_hRC = wglCreateContext(dc.GetSafeHdc());	//创建绘图上下文


	wglMakeCurrent(dc.GetSafeHdc(), m_hRC);
	int i;
	unsigned error;
	unsigned width, height;
	unsigned char *pStoneImg, *pCloudImg;
	unsigned char img1D[16][3];
	error = lodepng_decode32_file(&pStoneImg, &width, &height, "stone.png");
	if(error)
	{
		MessageBox(_T("failed"));
		return false;
	}
	upsidedown32(pStoneImg, width, height); //Upside down the image
	glGenTextures(1, &m_nTexStone);
	glBindTexture(GL_TEXTURE_2D, m_nTexStone);
	glTexImage2D(GL_TEXTURE_2D, 0, 4, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pStoneImg);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	free(pStoneImg);

	//make cloud texture
	error = lodepng_decode32_file(&pCloudImg, &width, &height, "cloud.png");
	if(error)
	{
		MessageBox(_T("failed"));
		return false;
	}
	upsidedown32(pCloudImg, width, height); //Upside down the image
	glGenTextures(1, &m_nTexCloud);
	glBindTexture(GL_TEXTURE_2D, m_nTexCloud);
	glTexImage2D(GL_TEXTURE_2D, 0, 4, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pCloudImg);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	free(pCloudImg);

	//make 1D texture
	for(i=0; i<16; i++)
		if(i%4)
			img1D[i][0] = img1D[i][1] = img1D[i][2] = 255;
		else
			img1D[i][0] = img1D[i][1] = img1D[i][2] = 0;
	glGenTextures(1, &m_nTex1D);
	glBindTexture(GL_TEXTURE_1D, m_nTex1D);
	glTexImage1D(GL_TEXTURE_1D, 0, 3, 16, 0, GL_RGB, GL_UNSIGNED_BYTE, img1D);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);

	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	
    wglMakeCurrent(dc.GetSafeHdc(), NULL);

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void COneDOFDlg::upsidedown32(unsigned char *pImage, unsigned width, unsigned height)
{
	unsigned i;
	unsigned char *pLine;

	pLine = (unsigned char *)malloc(4*width);
	if(pLine == NULL)
	{
		printf("No memory left!");
		exit(0);
	}
	for(i=0; i<height/2; i++)
	{
		memcpy(pLine, &pImage[i*4*width], 4*width);
		memcpy(&pImage[i*4*width], &pImage[(height-1-i)*4*width], 4*width);
		memcpy(&pImage[(height-1-i)*4*width], pLine, 4*width);
	}
	free(pLine);
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


	static int aa = 0;
	if( aa == 0)
	{
		//GLfloat lit_position[] = {0.0f, 0.0f, 1.0f, 0.0f};
		//GLfloat mat_yellow[] = {100.0f, 100.0f, 0.0f, 100.0f};
		//GLfloat mat_cyan[] = {0.0f, 100.0f, 100.0f, 100.0f};
		////CClientDC dc(this);
		//CPaintDC dc(GetDlgItem(IDC_PIC));
		//wglMakeCurrent(dc.GetSafeHdc(), m_hRC);
		//GLint rect[4];
		//glGetIntegerv(GL_VIEWPORT, rect);

		//glEnable(GL_DEPTH_TEST);
		//glClearColor(0.0, 0.0, 0.0, 0.0);
		//glClearDepth(1.0);
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
		////********************************************
		////glMatrixMode(GL_PROJECTION);
		////glLoadIdentity();
		////if(rect[3] < 1) rect[3]=1;
		////gluPerspective(30.0, 1.0*rect[2]/rect[3], 0.1, 20.0);

		////glMatrixMode(GL_MODELVIEW);
		////glLoadIdentity();

		////glTranslated(0.0, 0.0, -m_fDist);
		////glRotated(m_fXAngle, 1.0, 0.0, 0.0);
		////glRotated(m_fYAngle, 0.0, 1.0, 0.0);
		////DrawCoordinates();
		////glColor3d(0.0, 0.0, 1.0);
		////gluQuadricDrawStyle(pQuadric, GLU_LINE);
		////gluCylinder(pQuadric, 0.5, 0.0,1.0, 50, 5);

		////*******************************************
		//glMatrixMode(GL_PROJECTION);
		//glLoadIdentity();
		////if(rect[3] < 1) rect[3]=1;
		////gluPerspective(30.0, 1.0*rect[2]/rect[3], 0.1, 8000.0);//单位为mm，长度为实际长度
		////if( rect[2] > rect[3] )
		//	glOrtho(-50.0, 350.0, -400.0*((double)rect[3]/(double)rect[2])/2.0, 400.0*((double)rect[3]/(double)rect[2])/2.00,0.01,1100.0);
		////glOrtho(200.0, rect[2], 0, rect[3]);


		//glMatrixMode(GL_MODELVIEW);
		//glLoadIdentity();
		//glTranslated(0.0, 0.0, -500.0);//-m_fDist
		//glRotated(m_fXAngle, 1.0, 0.0, 0.0);
		//glRotated(m_fYAngle, 0.0, 0.0, 1.0);//所有的变化都是基于视点坐标系，这三句话是将所有物体整体移动和旋转。该方法的好处是，可以较为简单的改变整体视角
		//glRotated(-90.0, 0.0, 0.0, 1.0);//注意坐标变换顺序，先进行这一句，在进行上面的指令，以此类推，都是相对动坐标系
		////gluLookAt(0.0, 0.0, m_fDist,  
		////			0.0, 0.0, 0.0,  
		////			0.0, 1.0, 0.0);  //不容易改变整体视角


		//glDisable(GL_LIGHTING);
		//DrawCoordinates();
		//glEnable(GL_LIGHTING);
		//glEnable(GL_LIGHT0);

		////glPushMatrix(); //显示直角四面体
		////
		////	glTranslatef(10.0, 0.0, 0.0);
		////	glScaled(50.5, 50.5,50.5);

		////	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_yellow);
		////	DrawTetrahedron();
		////glPopMatrix();

		////GLUquadricObj *pQuadric = gluNewQuadric();
		////glPushMatrix(); //显示圆锥体
		////	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,1);
		////	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_cyan);
		////	glMaterialfv(GL_BACK, GL_DIFFUSE, mat_yellow);
		////	glTranslated(-1.0, 0.0, 0.0); //对应变换阵T6
		////	glRotated(-90.0, 1.0, 0.0, 0.0); //对应变换阵T7
		////	glScaled(50.5, 50.5,50.5);
		////	gluQuadricDrawStyle(pQuadric, GLU_FILL);
		////	gluCylinder(pQuadric, 0.5, 0.0,1.0, 50, 5); //显示圆锥体
		////	//显示圆锥体
		////glPopMatrix();

		////glColor3d(0.0, 0.0, 1.0);
		////gluQuadricDrawStyle(pQuadric, GLU_LINE);
		////gluCylinder(pQuadric, 50.0, 0.0,100.0, 50, 5);
		////gluDeleteQuadric(pQuadric);
		//double aa = 10.0;
		//glPushMatrix(); 
		//	glTranslatef(0.0, m_y0, 0.0);
		//	glRotated(m_theta, 0.0, 0.0, 1.0);
		//	glTranslatef(-100.0, -aa, 0.0);
		//	glScaled(200.0, aa, 10.0);
		//	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_yellow);
		//	DrawCuboid();
		//glPopMatrix();
	
		//glPushMatrix(); 
		//	glTranslatef(m_SphereX, m_SphereY, 2.5);
		//	glutSolidSphere(5.0,16,16);
		//glPopMatrix();
	 // 

		//SwapBuffers(wglGetCurrentDC());
		//wglMakeCurrent(dc.GetSafeHdc(), NULL);
		PaintVirtualEnvi();
		aa = 2;
	}


}
void COneDOFDlg::DrawCuboid()
{

	double pnt[8][3] = {{0.0,0.0,0.0}, {1.0,0.0,0.0}, {1.0,1.0,0.0}, {0.0,1.0,0.0},{0.0,0.0,1.0}, {1.0,0.0,1.0}, {1.0,1.0,1.0}, {0.0,1.0,1.0}};
	int face[6][4] = {{0,1,5,4}, {1,2,6,5}, {2,3,7,6}, {0,4,7,3},{4,5,6,7}, {0,3,2,1}};
	glDisable(GL_TEXTURE_1D);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, m_nTexStone);
	glBegin(GL_POLYGON); 
		glTexCoord2d(0.0, 0.0); glVertex3dv( pnt[face[0][0]] );
		glTexCoord2d(1.0, 0.0); glVertex3dv( pnt[face[0][1]] );
		glTexCoord2d(1.0, 1.0); glVertex3dv( pnt[face[0][2]] );
		glTexCoord2d(0.0, 1.0); glVertex3dv( pnt[face[0][3]] );
	glEnd();

	glBegin(GL_POLYGON); 
		glTexCoord2d(0.0, 0.0); glVertex3dv( pnt[face[1][0]] );
		glTexCoord2d(1.0, 0.0); glVertex3dv( pnt[face[1][1]] );
		glTexCoord2d(1.0, 1.0); glVertex3dv( pnt[face[1][2]] );
		glTexCoord2d(0.0, 1.0); glVertex3dv( pnt[face[1][3]] );
	glEnd();
	
	glBegin(GL_POLYGON); 
		glTexCoord2d(0.0, 0.0); glVertex3dv( pnt[face[2][0]] );
		glTexCoord2d(1.0, 0.0); glVertex3dv( pnt[face[2][1]] );
		glTexCoord2d(1.0, 1.0); glVertex3dv( pnt[face[2][2]] );
		glTexCoord2d(0.0, 1.0); glVertex3dv( pnt[face[2][3]] );
	glEnd();

	glBegin(GL_POLYGON); 
		glTexCoord2d(0.0, 0.0); glVertex3dv( pnt[face[3][0]] );
		glTexCoord2d(1.0, 0.0); glVertex3dv( pnt[face[3][1]] );
		glTexCoord2d(1.0, 1.0); glVertex3dv( pnt[face[3][2]] );
		glTexCoord2d(0.0, 1.0); glVertex3dv( pnt[face[3][3]] );
	glEnd();
	
	glBegin(GL_POLYGON); 
		glTexCoord2d(0.0, 0.0); glVertex3dv( pnt[face[4][0]] );
		glTexCoord2d(1.0, 0.0); glVertex3dv( pnt[face[4][1]] );
		glTexCoord2d(1.0, 1.0); glVertex3dv( pnt[face[4][2]] );
		glTexCoord2d(0.0, 1.0); glVertex3dv( pnt[face[4][3]] );
	glEnd();

	glBegin(GL_POLYGON); 
		glTexCoord2d(0.0, 0.0); glVertex3dv( pnt[face[5][0]] );
		glTexCoord2d(1.0, 0.0); glVertex3dv( pnt[face[5][1]] );
		glTexCoord2d(1.0, 1.0); glVertex3dv( pnt[face[5][2]] );
		glTexCoord2d(0.0, 1.0); glVertex3dv( pnt[face[5][3]] );
	glEnd();
	
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_TEXTURE_1D);
}
void COneDOFDlg::DrawRect()
{
	glColor3f(1.0f, 0.0f, 0.0f); //画红色的x轴
	glBegin(GL_LINE_LOOP);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(1.0f, 0.0f, 0.0f);
		glVertex3f(1.0f, 1.0f, 0.0f);
		glVertex3f(0.0f, 1.0f, 0.0f);
	glEnd();
}


void COneDOFDlg::DrawTetrahedron()
{
	double pnt[4][3] = {{0.0,0.0,0.0}, {1.0,0.0,0.0}, {0.0,1.0,0.0}, {0.0,0.0,1.0}};
	int tetra[4][3] = {{0,2,1}, {0,3,2}, {0,1,3}, {1,2,3}};//面的点的方向，法线方向为贴图方向

	glDisable(GL_TEXTURE_2D);
	glEnable(GL_TEXTURE_1D);
	glBindTexture(GL_TEXTURE_1D, m_nTex1D);
	glBegin(GL_POLYGON); //X-Y
		glTexCoord1d(0.0); glVertex3dv(pnt[tetra[0][0]]);
		glTexCoord1d(1.0); glVertex3dv(pnt[tetra[0][1]]);
		glTexCoord1d(0.0); glVertex3dv(pnt[tetra[0][2]]);
	glEnd();

	glDisable(GL_TEXTURE_2D);
	glEnable(GL_TEXTURE_1D);
	glBindTexture(GL_TEXTURE_1D, m_nTex1D);
	glBegin(GL_POLYGON); //Y-Z
		glTexCoord1d(0.0); glVertex3dv(pnt[tetra[1][0]]);
		glTexCoord1d(0.0); glVertex3dv(pnt[tetra[1][1]]);
		glTexCoord1d(1.0); glVertex3dv(pnt[tetra[1][2]]);
	glEnd();

	glDisable(GL_TEXTURE_1D);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, m_nTexStone);
	glBegin(GL_POLYGON); //Z-X
		glTexCoord2d(0.0, 0.0); glVertex3dv(pnt[tetra[2][0]]);
		glTexCoord2d(1.0, 0.0); glVertex3dv(pnt[tetra[2][1]]);
		glTexCoord2d(0.0, 1.0); glVertex3dv(pnt[tetra[2][2]]);
	glEnd();

	glDisable(GL_TEXTURE_1D);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, m_nTexCloud);
	glBegin(GL_POLYGON); //slope
		glTexCoord2d(1.0, 0.0); glVertex3dv(pnt[tetra[3][0]]);
		glTexCoord2d(0.5, 1.0); glVertex3dv(pnt[tetra[3][1]]);
		glTexCoord2d(0.0, 0.0); glVertex3dv(pnt[tetra[3][2]]);
	glEnd();

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_TEXTURE_1D);
}
void COneDOFDlg::DrawCoordinates()
{
	glColor3f(1.0f, 0.0f, 0.0f); //画红色的x轴
	glBegin(GL_LINES);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(20.0f, 0.0f, 0.0f);
	glEnd();
	glColor3f(0.0f, 10.0f, 0.0f); //画绿色的y轴
	glBegin(GL_LINES);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(0.0f, 20.0f, 0.0f);
	glEnd();
	glColor3f(0.0f, 0.0f, 1.0f); //画蓝色的z轴
	glBegin(GL_LINES);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(0.0f, 0.0f, 20.0f);
	glEnd();
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
	static double prex,prey;
	static int firstinwall = 0;

	//获取连杆角度
	m_udCounterCtrl->setEnabled(TRUE);
	int value[2];
	m_udCounterCtrl->Read(2, value);
	m_Read_LinkAngle[0] = INIT_ANGLE_LINK1 + value[0] / LINK_ENCODER_COUNTS * 2.0 * PI;
	m_Read_LinkAngle[1] = INIT_ANGLE_LINK2 + value[1] / LINK_ENCODER_COUNTS * 2.0 * PI;

	//正解，得出末端位置
	m_FiveBarKinematics.ForwardKinematics(m_Read_LinkAngle[0],m_Read_LinkAngle[1],m_Position_HapticTool);
	
	////嵌入深度记录
	//CString str;
	//if(1==frictionflag)
	//{
	//	//读取力数值并转换到笛卡尔空间
	//	myfriction.x = m_Position_HapticTool[0];
	//	myfriction.y = m_Position_HapticTool[1];
	//	myfriction.f = 0.0;
	//	myvec.push_back(myfriction);
	//}
	//if(2==frictionflag)
	//{
	//	for(vector<Friction>::iterator it = myvec.begin();it!=myvec.end();it++)
	//	{

	//		str.Format(_T("%f\t%f\t\n"), (*it).x, (*it).y);
	//		myfile.WriteString(str);
	//	}
	//	frictionflag = 0;
	//	myvec.clear();
	//}


	////阻尼实验
	//CString str;
	//if(1==frictionflag)
	//{
	//	//读取力数值并转换到笛卡尔空间
	//	myfriction.x = m_Position_HapticTool[0];
	//	myfriction.y = m_Position_HapticTool[1];
	//	myfriction.f = 0.0;
	//	myvec.push_back(myfriction);
	//}
	//if(2==frictionflag)
	//{
	//	for(vector<Friction>::iterator it = myvec.begin();it!=myvec.end();it++)
	//	{

	//		str.Format(_T("%f\t%f\t\n"), (*it).x, (*it).y);
	//		myfile.WriteString(str);
	//	}
	//	frictionflag = 0;
	//	myvec.clear();
	//}

	
	if( TRUE == m_bOnlyFree )//只跟随
	{
		firstinwall = 0; 

		//关节1
		if( m_Read_LinkAngle[0] < ( m_Current_DPCAngle[0] + ALFA1 )  )
		{
			m_Cmd_DPCAngle[0] = m_Read_LinkAngle[0] - ALFA1;
		}
		else if( m_Read_LinkAngle[0] > ( m_Current_DPCAngle[0] + BETA1 - ALFA1 ) )
		{
			m_Cmd_DPCAngle[0] = m_Read_LinkAngle[0] + ALFA1 - BETA1;
		}
		else
		{
		}
		//关节2
		if( m_Read_LinkAngle[1] <= ( m_Current_DPCAngle[1] + ALFA2 )  )
		{
			m_Cmd_DPCAngle[1] = m_Read_LinkAngle[1] - ALFA2;
		}
		else if( m_Read_LinkAngle[1] >= ( m_Current_DPCAngle[1] + BETA2 - ALFA2 ) )
		{
			m_Cmd_DPCAngle[1] = m_Read_LinkAngle[1] + ALFA2 - BETA2;
		}
		else
		{
		}

		bFirst_in_Wall = FALSE;

		////反向摩擦力实验
		//if(1==frictionflag)
		//{
		//	//读取力数值并转换到笛卡尔空间
		//	INT temp = g_ForceIndex;
		//	m_Read_ForceSensor[0] = g_ForceBuffer[temp][0];
		//	m_Read_ForceSensor[1] = g_ForceBuffer[temp][1];
		//	double Fsensor[] = { m_Read_ForceSensor[0]-m_Read_ForceSensor0[0], m_Read_ForceSensor[1]-m_Read_ForceSensor0[1] };

		//	//计算笛卡尔力
		//	m_FiveBarKinematics.GetForce(m_Position_HapticTool[0], m_Position_HapticTool[1], m_Read_LinkAngle[0], m_Read_LinkAngle[1], Fsensor, m_Cartesian_Force);

		//	myfriction.x = m_Position_HapticTool[0];
		//	myfriction.y = m_Position_HapticTool[1];
		//	myfriction.f = sqrt( m_Cartesian_Force[0]*m_Cartesian_Force[0]+ m_Cartesian_Force[1]*m_Cartesian_Force[1] );
		//	myvec.push_back(myfriction);
		//
		//}
		//if(2==frictionflag)
		//{
		//	OnBnClickedBtnStart();
		//	for(vector<Friction>::iterator it = myvec.begin();it!=myvec.end();it++)
		//	{
		//		CString str;
		//		str.Format(_T("%f\t%f\t%f\n"), (*it).x, (*it).y,(*it).f);
		//		myfile.WriteString(str);
		//	}
		//	frictionflag = 0;
		//}


	}
	else//增加力反馈
	{

		//**************虚拟墙为斜线******************
		double kk = 2000.0;//刚度系数，越大，阻力越大

		double th = m_theta / 180.0 * PI;//直线斜率为x度
		double k_line = tan(th);  //直线斜率
		double x0 = 0.0, y0 = m_y0;//直线过该点
		double t[2] = {cos(th), sin(th)};//直线的切线单位向量
		double n[2] = {cos(th + 90.0 / 180.0 * PI), sin(th + 90.0 / 180.0 * PI)};//法线单位向量
		double y_cal = k_line * ( m_Position_HapticTool[0] - x0 ) + y0; 
		if( (m_Position_HapticTool[1] <= y_cal)  )  //&& ( m_Position_HapticTool[0] > MIN_WALL_X && m_Position_HapticTool[0] < MAX_WALL_X )
		{
			if (firstinwall == 0)//第一次进入wall
			{
				firstinwall = 1;
				prex = m_Position_HapticTool[0];
				prey = m_Position_HapticTool[1];
			}

			//**************虚拟墙为斜线******************
			//读取力数值并转换到笛卡尔空间
			INT temp = g_ForceIndex;
			m_Read_ForceSensor[0] = g_ForceBuffer[temp][0];
			m_Read_ForceSensor[1] = g_ForceBuffer[temp][1];
			double Fsensor[] = { m_Read_ForceSensor[0]-m_Read_ForceSensor0[0], m_Read_ForceSensor[1]-m_Read_ForceSensor0[1] };

			//计算笛卡尔力
			m_FiveBarKinematics.GetForce(m_Position_HapticTool[0], m_Position_HapticTool[1], m_Read_LinkAngle[0], m_Read_LinkAngle[1], Fsensor, m_Cartesian_Force);
			
			//X=F/K模型
			double Ft[2] = {  (m_Cartesian_Force[0]*t[0]+m_Cartesian_Force[1]*t[1])*t[0]  , (m_Cartesian_Force[0]*t[0]+m_Cartesian_Force[1]*t[1])*t[1]  };//作用力沿t方向分力

			double detaD[2] = { Ft[0]/kk, Ft[1]/kk  };//位移

			double targetX =  prex + detaD[0];
			double targetY =  prey + detaD[1];
			m_FiveBarKinematics.InverseKinematics(targetX, targetY, m_Cmd_DPCAngle);
			m_Cmd_DPCAngle[0] = m_Cmd_DPCAngle[0];
			m_Cmd_DPCAngle[1] = m_Cmd_DPCAngle[1] - BETA2;
			prex = targetX;
			prey = targetY;


			//刚度实验
			CString str;
			if(1==frictionflag)
			{
				//读取力数值并转换到笛卡尔空间
				myfriction.x = m_Position_HapticTool[0];
				myfriction.y = m_Position_HapticTool[1];
				myfriction.f = m_Cartesian_Force[1];
				myvec.push_back(myfriction);
			}
			if(2==frictionflag)
			{
				for(vector<Friction>::iterator it = myvec.begin();it!=myvec.end();it++)
				{

					str.Format(_T("%f\t%f\t%f\n"), (*it).x, (*it).y, (*it).f);
					myfile.WriteString(str);
				}
				frictionflag = 0;
				myvec.clear();
			}

		}


		/*
		//**************外圆弧******************
		double th;//直线斜率为x度
		if( (m_Position_HapticTool[0]-m_circleX) == 0.0 )
			th = 90.0 / 180.0 * PI;
		if( (m_Position_HapticTool[0]-m_circleX) > 0.0 )
			th = atan((m_Position_HapticTool[1]-m_circleY)/(m_Position_HapticTool[0]-m_circleX));
		if( (m_Position_HapticTool[0]-m_circleX) < 0.0 )
			th = atan((m_Position_HapticTool[1]-m_circleY)/(m_Position_HapticTool[0]-m_circleX)) + PI;

		double kk = 15.0;//刚度系数
		double t[2] = {cos(th), sin(th)};
		double n[2] = {cos(th + 90.0 / 180.0 * PI), sin(th + 90.0 / 180.0 * PI)};//法线单位向量
		double temp = ( m_Position_HapticTool[0] - m_circleX) * ( m_Position_HapticTool[0] - m_circleX) + ( m_Position_HapticTool[1] - m_circleY) * ( m_Position_HapticTool[1] - m_circleY);

		if( temp <= m_circleR*m_circleR  )
		{
			if (firstinwall == 0)//第一次进入wall
			{
				firstinwall = 1;
				prex = m_Position_HapticTool[0];
				prey = m_Position_HapticTool[1];
			}
			//**************外圆弧******************
			//读取力数值并转换到笛卡尔空间
			INT temp = g_ForceIndex;
			m_Read_ForceSensor[0] = g_ForceBuffer[temp][0];
			m_Read_ForceSensor[1] = g_ForceBuffer[temp][1];
			double Fsensor[] = { m_Read_ForceSensor[0]-m_Read_ForceSensor0[0], m_Read_ForceSensor[1]-m_Read_ForceSensor0[1] };

			//计算笛卡尔力
			m_FiveBarKinematics.GetForce(m_Position_HapticTool[0], m_Position_HapticTool[1], m_Read_LinkAngle[0], m_Read_LinkAngle[1], Fsensor, m_Cartesian_Force);
			
			//X=F/K模型
			double Ft[2] = {  (m_Cartesian_Force[0]*n[0]+m_Cartesian_Force[1]*n[1])*n[0]  , (m_Cartesian_Force[0]*n[0]+m_Cartesian_Force[1]*n[1])*n[1]  };

			double detaD[2] = { Ft[0]/kk, Ft[1]/kk  };//位移
						
			double RR = sqrt( (prex-m_circleX)  *  (prex-m_circleX)  +  (prey-m_circleY)  *  (prey-m_circleY));

			double theta;
			if( (prex-m_circleX) == 0.0 )
				theta = 90.0 / 180.0 * PI;
			if( (prex-m_circleX) > 0.0 )
				theta = atan((prey-m_circleY)/(prex-m_circleX));
			if( (prex-m_circleX) < 0.0 )
				theta = atan((prey-m_circleY)/(prex-m_circleX)) + PI;

			double theta_next;
			double dd = sqrt( detaD[0]*detaD[0] + detaD[1]*detaD[1]);
			if( detaD[0] >=0.0 )
				theta_next = theta - dd / RR;
			else
				theta_next = theta + dd / RR;
			double targetX =  RR * cos(theta_next) + m_circleX;
			double targetY =  RR * sin(theta_next) + m_circleY;

			m_FiveBarKinematics.InverseKinematics(targetX, targetY, m_Cmd_DPCAngle);
			m_Cmd_DPCAngle[0] = m_Cmd_DPCAngle[0];
			m_Cmd_DPCAngle[1] = m_Cmd_DPCAngle[1] - BETA2;
			prex = targetX;
			prey = targetY;
		}
		*/

		/*
		//**************凹圆弧******************
		double kk = 10.0;//刚度系数

		double temp = ( m_Position_HapticTool[0] - m_circleX) * ( m_Position_HapticTool[0] - m_circleX) + ( m_Position_HapticTool[1] - m_circleY) * ( m_Position_HapticTool[1] - m_circleY);

		if( temp >= m_circleR*m_circleR && ( m_Position_HapticTool[1]< (m_circleY-m_circleR/4.0) ) )
		{
			if (firstinwall == 0)//第一次进入wall
			{
				firstinwall = 1;
				prex = m_Position_HapticTool[0];
				prey = m_Position_HapticTool[1];
			}
			double th;//直线斜率为x度
			if( (m_Position_HapticTool[0]-m_circleX) == 0.0 )
				th = 90.0 / 180.0 * PI;
			if( (m_Position_HapticTool[0]-m_circleX) > 0.0 )
				th = atan((m_Position_HapticTool[1]-m_circleY)/(m_Position_HapticTool[0]-m_circleX));
			if( (m_Position_HapticTool[0]-m_circleX) < 0.0 )
				th = atan((m_Position_HapticTool[1]-m_circleY)/(m_Position_HapticTool[0]-m_circleX)) + PI;
			double t[2] = {cos(th), sin(th)};
			double n[2] = {cos(th + 90.0 / 180.0 * PI), sin(th + 90.0 / 180.0 * PI)};//法线单位向量


			//**************外圆弧******************
			//读取力数值并转换到笛卡尔空间
			INT temp = g_ForceIndex;
			m_Read_ForceSensor[0] = g_ForceBuffer[temp][0];
			m_Read_ForceSensor[1] = g_ForceBuffer[temp][1];
			double Fsensor[] = { m_Read_ForceSensor[0]-m_Read_ForceSensor0[0], m_Read_ForceSensor[1]-m_Read_ForceSensor0[1] };

			//计算笛卡尔力
			m_FiveBarKinematics.GetForce(m_Position_HapticTool[0], m_Position_HapticTool[1], m_Read_LinkAngle[0], m_Read_LinkAngle[1], Fsensor, m_Cartesian_Force);
			
			//X=F/K模型
			double Ft[2] = {  (m_Cartesian_Force[0]*n[0]+m_Cartesian_Force[1]*n[1])*n[0]  , (m_Cartesian_Force[0]*n[0]+m_Cartesian_Force[1]*n[1])*n[1]  };

			double detaD[2] = { Ft[0]/kk, Ft[1]/kk  };//位移
						
			double RR = sqrt( (prex-m_circleX)  *  (prex-m_circleX)  +  (prey-m_circleY)  *  (prey-m_circleY));

			double theta;
			if( (prex-m_circleX) == 0.0 )
				theta = -90.0 / 180.0 * PI;
			if( (prex-m_circleX) > 0.0 )
				theta = atan((prey-m_circleY)/(prex-m_circleX));
			if( (prex-m_circleX) < 0.0 )
				theta = atan((prey-m_circleY)/(prex-m_circleX)) - PI;

			double theta_next;
			double dd = sqrt( detaD[0]*detaD[0] + detaD[1]*detaD[1]);
			if( detaD[0] >=0.0 )
				theta_next = theta + dd / RR;
			else
				theta_next = theta - dd / RR;
			double targetX =  RR * cos(theta_next) + m_circleX;
			double targetY =  RR * sin(theta_next) + m_circleY;

			m_FiveBarKinematics.InverseKinematics(targetX, targetY, m_Cmd_DPCAngle);
			m_Cmd_DPCAngle[0] = m_Cmd_DPCAngle[0];
			m_Cmd_DPCAngle[1] = m_Cmd_DPCAngle[1] - BETA2;
			prex = targetX;
			prey = targetY;
		}
		*/

		//自由空间
		else      
		{
			firstinwall = 0; 

			m_Cmd_DPCAngle[0] = m_Read_LinkAngle[0] - ALFA1;	//前提是已知上一次DPC的哪一侧碰撞连杆
			m_Cmd_DPCAngle[1] = m_Read_LinkAngle[1] + ALFA2 - BETA2;
			bFirst_in_Wall = FALSE;
		}

	}

	//安全判断
	if( m_Cmd_DPCAngle[0] >= MAX_ANGLE_DPC1 )
		m_Cmd_DPCAngle[0] = MAX_ANGLE_DPC1;
	if( m_Cmd_DPCAngle[0] <= MIN_ANGLE_DPC1 )
		m_Cmd_DPCAngle[0] = MIN_ANGLE_DPC1;

	if( m_Cmd_DPCAngle[1] >= MAX_ANGLE_DPC2 )
		m_Cmd_DPCAngle[1] = MAX_ANGLE_DPC2;
	if( m_Cmd_DPCAngle[1] <= MIN_ANGLE_DPC2 )
		m_Cmd_DPCAngle[1] = MIN_ANGLE_DPC2;

	m_Current_DPCAngle[0] = m_Cmd_DPCAngle[0];
	m_Current_DPCAngle[1] = m_Cmd_DPCAngle[1];

	//发送电机位置
	m_CANCmdFrame[0] = * m_CANCmd.GetFrame(1, MODE_POSITION, m_Cmd_DPCAngle[0]-INIT_ANGLE_DPC1);
	m_CANCmdFrame[1] = * m_CANCmd.GetFrame(2, MODE_POSITION, -(m_Cmd_DPCAngle[1]-INIT_ANGLE_DPC2));//电机调试转向相反
	//m_CANCmdFrame[2] = * m_CANCmd.GetFrame(1, MODE_VELOCITY, 2.0*360.0/360.0*PI);
	//m_CANCmdFrame[3] = * m_CANCmd.GetFrame(2, MODE_VELOCITY, 2.0*360.0/360.0*PI);
	m_CANCmdFrame[2] = * m_CANCmd.GetFrame(0, MODE_ENABLEALLAXIS);
	m_CANCmdFrame[3] = * m_CANCmd.GetFrame(0, MODE_EXECUTEALLMOVING);
	Transmit(m_DeviceType, 0, 0, m_CANCmdFrame, 4);

	CString m_counterValue;
	m_counterValue.Format(_T("\t%f\t"),m_Position_HapticTool[0]);  //m_Cmd_DPCAngle[0]/PI*180.0
	GetDlgItem(IDC_EDIT1)->SetWindowText(m_counterValue.GetBuffer());

	m_counterValue.Format(_T("\t%f\t"),m_Position_HapticTool[1]);  //m_Cmd_DPCAngle[1]/PI*180.0
	//m_counterValue.Format(_T("\t%f\t"),m_Position_HapticTool[1]); 
	GetDlgItem(IDC_EDIT2)->SetWindowText(m_counterValue.GetBuffer());

	

}

void COneDOFDlg::PaintVirtualEnvi()
{

	m_SphereX = m_Position_HapticTool[0];
	m_SphereY = m_Position_HapticTool[1];


	GLfloat lit_position[] = {0.0f, 0.0f, 1.0f, 0.0f};
	GLfloat mat_yellow[] = {100.0f, 100.0f, 0.0f, 100.0f};
	GLfloat mat_cyan[] = {0.0f, 100.0f, 100.0f, 100.0f};
	GLfloat mat_red[] = {255.0f, 0.0f, 00.0f, 0.0f};
	//CClientDC dc(this);
	CPaintDC dc(GetDlgItem(IDC_PIC));
    wglMakeCurrent(dc.GetSafeHdc(), m_hRC);
	GLint rect[4];
	glGetIntegerv(GL_VIEWPORT, rect);

	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0, 0.0, 0.0, 0.0);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//if(rect[3] < 1) rect[3]=1;
	//gluPerspective(30.0, 1.0*rect[2]/rect[3], 0.1, 8000.0);//单位为mm，长度为实际长度
	//if( rect[2] > rect[3] )
		glOrtho(-50.0, 350.0, -400.0*((double)rect[3]/(double)rect[2])/2.0, 400.0*((double)rect[3]/(double)rect[2])/2.00,0.01,1100.0);
	//glOrtho(200.0, rect[2], 0, rect[3]);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslated(0.0, 0.0, -500.0);//-m_fDist
	glRotated(m_fXAngle, 1.0, 0.0, 0.0);
	glRotated(m_fYAngle, 0.0, 0.0, 1.0);//所有的变化都是基于视点坐标系，这三句话是将所有物体整体移动和旋转。该方法的好处是，可以较为简单的改变整体视角
	glRotated(-90.0, 0.0, 0.0, 1.0);//注意坐标变换顺序，先进行这一句，在进行上面的指令，以此类推，都是相对动坐标系

	//gluLookAt(0.0, 0.0, m_fDist,  
	//			0.0, 0.0, 0.0,  
	//			0.0, 1.0, 0.0);  //不容易改变整体视角

	glDisable(GL_LIGHTING);
	DrawCoordinates();
	//工作空间
	glPushMatrix(); 
		glTranslatef(0.0, 115.0, 0.0);
		glTranslatef(-50.0, -50.0, 0.0);
		glScaled(100.0, 100.0, 0);
		DrawRect();
	glPopMatrix();
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_yellow);

	double aa = 30.0;

	glPushMatrix(); //虚拟墙
		glTranslatef(0.0, m_y0, 0.0);
		glRotated(m_theta, 0.0, 0.0, 1.0);
		glTranslatef(-100.0, -aa, 0.0);
		glScaled(200.0, aa, 10.0);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_yellow);
		DrawCuboid();
	glPopMatrix();

	
	//glPushMatrix(); //圆形
	//	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_cyan);
	//	glTranslatef(m_circleX, m_circleY, 2.5);
	//	//glutSolidSphere(m_circleR,160,160);
	//	glutWireSphere(m_circleR,32,32);
	//glPopMatrix();
	

	glPushMatrix(); 
		glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_red);
		glTranslatef(m_SphereX, m_SphereY, 250);
		glutSolidSphere(2.0,8,8);
	glPopMatrix();

    SwapBuffers(wglGetCurrentDC());
    wglMakeCurrent(dc.GetSafeHdc(), NULL);




}

//视觉渲染,30ms，由多媒体定时器调用
void COneDOFDlg::MMTimerGraphicRendering(UINT nIDEvent)
{
// do what you want to do, but quickly
	PaintVirtualEnvi();
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
			//GetDlgItem(IDC_BTN_CONNECT_FORCE)->SetWindowText(_T("断开力传感器"));
			GetDlgItem(IDC_BTN_CONNECT_FORCE)->EnableWindow(FALSE);
			//AfxMessageBox(_T("连接力传感器成功！"));
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

	//AfxMessageBox(_T("配置成功！"));

	GetDlgItem(IDC_BTN_CONFIGURE_FORCE)->EnableWindow(FALSE);

	m_bFlagInitForce = TRUE;

	//m_CANCmdFrame[0] = * m_CANCmd.GetFrame(MODE_SETPOSITIONZERO);//!!!不要删除，设置电机当前位置为0，调试正确
	//Transmit(m_DeviceType,0,0,m_CANCmdFrame,1);

}

//打开DAQ串口
void COneDOFDlg::OnBnClickedBtnOpenDaqPort()
{
	//// TODO: 在此添加控件通知处理程序代码

	//读取一次三个轴编码器读数，耗时小于0.02ms

	CString str;
	int i = 0;
	GetDlgItem(IDC_BTN_OPEN_DAQ_PORT)->GetWindowText(str);
	if( ( str == _T("打开DAQ串口") ) ) 
	{	
		UdCounterCtrl* udCounterCtrl = UdCounterCtrl::Create();
		Array<DeviceTreeNode>* sptedDevices = udCounterCtrl->getSupportedDevices();
		if (sptedDevices->getCount() == 0)
		{
			AfxMessageBox(_T("无可用设备！"));
			return;
		}
		else
		{
			ErrorCode	errorCode;
			DeviceTreeNode const & node = sptedDevices->getItem(0);	
			DeviceInformation devInfo( node.Description );
			errorCode = m_udCounterCtrl->setSelectedDevice(devInfo);
		
			// set counter channel
			errorCode = m_udCounterCtrl->setChannelCount(3);//1，扫描的通道数目
			errorCode = m_udCounterCtrl->setChannelStart(m_confParam.channel);//起始通道

			for (i = m_udCounterCtrl->getChannelStart(); i < m_udCounterCtrl->getChannelStart() + m_udCounterCtrl->getChannelCount(); i++ )
			{
				errorCode = m_udCounterCtrl->getChannels()->getItem(1).setCountingType(m_confParam.cntType);
			}
			// Set reset times for counter value
			m_udCounterCtrl->setResetTimesByIndex(0);//-1 means that reset counter value infinite times,0 means that can not reset counter value
			//set reset value
			for (i = m_udCounterCtrl->getChannelStart(); i < m_udCounterCtrl->getChannelStart() + m_udCounterCtrl->getChannelCount(); i++ )
			{
				errorCode = m_udCounterCtrl->getChannels()->getItem(i).setInitialValue(0x00);
			}
			//Start UpDown Counter function.
			errorCode = m_udCounterCtrl->setEnabled(true);

			sptedDevices->Dispose();
			udCounterCtrl->Dispose();
			//GetDlgItem(IDC_BTN_OPEN_DAQ_PORT)->SetWindowText(_T("关闭DAQ串口"));
			m_bFlagInitDAQ = TRUE;
			GetDlgItem(IDC_BTN_OPEN_DAQ_PORT)->EnableWindow(FALSE);
		}
	}
	else
	{
		m_udCounterCtrl->setEnabled(FALSE);
		GetDlgItem(IDC_BTN_OPEN_DAQ_PORT)->SetWindowText(_T("打开DAQ串口"));
		return;
	}

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
		//MessageBox(_T("启动CAN成功！"),_T("警告"),MB_OK|MB_ICONQUESTION);
		m_bFlagCanConnect = TRUE;		//标志位
		GetDlgItem(IDC_BTN_OPEN_CAN_PORT)->SetWindowText(_T("断开CAN串口"));
		GetDlgItem(IDC_BTN_OPEN_CAN_PORT)->EnableWindow(FALSE);
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
	if( m_bFlagCanConnect == FALSE)
	{
		MessageBox(_T("请先打开CAN串口！"),_T("警告"),MB_OK|MB_ICONQUESTION);
		return;
	}

	m_CANCmdFrame[0] = * m_CANCmd.GetFrame(0, MODE_STARTCAN);//获取启动帧，第一步启动can
	//Transmit(m_DeviceType,0,0,m_CANCmdFrame,1);

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

	m_CANCmdFrame[13] = * m_CANCmd.GetFrame(1,MODE_ENABLEPDO);//使能PDO
	m_CANCmdFrame[14] = * m_CANCmd.GetFrame(2,MODE_ENABLEPDO);//使能PDO
	m_CANCmdFrame[15] = * m_CANCmd.GetFrame(3,MODE_ENABLEPDO);//使能PDO

	if( 16 == Transmit(m_DeviceType,0,0,m_CANCmdFrame,16) )
	{
		m_CANCmdFrame[0] = * m_CANCmd.GetFrame(1, MODE_SETPOSITIONZERO);//!!!不要删除，设置电机当前位置为0，调试正确
		m_CANCmdFrame[1] = * m_CANCmd.GetFrame(2, MODE_SETPOSITIONZERO);//!!!不要删除，设置电机当前位置为0，调试正确
		m_CANCmdFrame[2] = * m_CANCmd.GetFrame(3, MODE_SETPOSITIONZERO);//!!!不要删除，设置电机当前位置为0，调试正确

		Transmit(m_DeviceType,0,0,m_CANCmdFrame,3);
		//MessageBox(_T("初始化驱动器完成！"),_T("警告"),MB_OK|MB_ICONQUESTION);
		GetDlgItem(IDC_BTN_INITIAL_DRIVE)->EnableWindow(FALSE);
		m_bFlagInitDrive = TRUE;
	}
	else
		MessageBox(_T("初始化驱动器失败！"),_T("警告"),MB_OK|MB_ICONQUESTION);

}


//开始程序，即打开视觉和触觉多媒体定时器
void COneDOFDlg::OnBnClickedBtnStart()
{
	
	//初始化程序变量，如连杆初始位置、驱动器初始位置、力传感初始值，包括软件和硬件，
	//主要防止重启程序不重新供电造成的错误。
	//程序运行有两种情况，1，程序运行过程中若重启，必须重启硬件电源，即每次运行程序（包括点击停止再开始）前都要重启电源
	//2、以当前点击按钮时的状态为初始状态，力传感器有一个初始力F0，连杆有一个初始角度E0，电机应该也有一个位置M0
	//（但是无法获得，所以最好重启电源或者有一条指令设置电机位置为零），之后所有的计算都是基于这个计算
	//if( ! m_bFlagInitForce  )
	//{
	//	MessageBox(_T("没有初始化力传感器！"),_T("警告"),MB_OK|MB_ICONQUESTION);
	//	return;
	//}
	//if( ! m_bFlagInitDAQ  )
	//{
	//	MessageBox(_T("没有初始化DAQ！"),_T("警告"),MB_OK|MB_ICONQUESTION);
	//	return;
	//}
	//if( ! m_bFlagInitDrive  )
	//{
	//	MessageBox(_T("没有初始化驱动器！"),_T("警告"),MB_OK|MB_ICONQUESTION);
	//	return;
	//}


	

//********************初始化控制参量*********************************
	//末端初始位置
	m_Position_HapticTool[0] = INIT_X;//x
	m_Position_HapticTool[1] = INIT_Y;//y

	//获取编码器读数
	m_udCounterCtrl->setEnabled(TRUE);
	int value[2];
	m_udCounterCtrl->Read(2, value);
	m_Read_LinkAngle[0] = INIT_ANGLE_LINK1 + value[0]/LINK_ENCODER_COUNTS*2.0*PI;
	m_Read_LinkAngle[1] = INIT_ANGLE_LINK2 + value[1]/LINK_ENCODER_COUNTS*2.0*PI;
	m_Current_DPCAngle[0] = m_Read_LinkAngle[0];
	m_Current_DPCAngle[1] = m_Read_LinkAngle[1];

	////初始笛卡尔力
	//INT temp= g_ForceIndex;
	//m_Read_ForceSensor[0] = g_ForceBuffer[temp][0];
	//m_Read_ForceSensor[1] = g_ForceBuffer[temp][1];
	//double Fsensor[] = { m_Read_ForceSensor[0], m_Read_ForceSensor[1] };
	//m_FiveBarKinematics.GetForce(INIT_X,INIT_Y,INIT_ANGLE_LINK1,INIT_ANGLE_LINK2,Fsensor,m_Cartesian_Force0);

	//初始笛卡尔力
	INT temp= g_ForceIndex;
	m_Read_ForceSensor0[0] = g_ForceBuffer[temp][0];
	m_Read_ForceSensor0[1] = g_ForceBuffer[temp][1];

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
		//SetTimer(1,20,NULL);
		g_TimerVR = timeSetEvent(
								g_uDelayGraphic,
								g_uResolution,
								GraphicRendering,
								(DWORD)this,
								TIME_PERIODIC);
		
		GetDlgItem(IDC_BTN_START)->SetWindowText(_T("停止"));
		m_bTimerRunning = TRUE;
	}
	else if ( m_bTimerRunning == TRUE )
	{
		timeKillEvent(g_TimerVR);
		timeKillEvent(g_TimerHR);
		timeEndPeriod (g_uResolution);

		m_bTimerRunning = FALSE;
		GetDlgItem(IDC_BTN_START)->EnableWindow(FALSE);
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

	m_udCounterCtrl->Dispose();

	wglDeleteContext(m_hRC);
	if(glIsTexture(m_nTexStone)) glDeleteTextures(1, &m_nTexStone);
	if(glIsTexture(m_nTexCloud)) glDeleteTextures(1, &m_nTexCloud);
	if(glIsTexture(m_nTex1D)) glDeleteTextures(1, &m_nTex1D);

	myfile.Close();

}



void COneDOFDlg::OnBnClickedRadio1()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	switch(m_RadioBtn)
	{
	case 0:
		//MessageBox(_T("自由"),_T("警告"),MB_OK|MB_ICONQUESTION);
		m_bOnlyFree = TRUE;
		break;
	case 1:
		//MessageBox(_T("力反馈"),_T("警告"),MB_OK|MB_ICONQUESTION);
		m_bOnlyFree = FALSE;
		break;
	}
}

	////*******************测试运行时间**************
	//LARGE_INTEGER nFreq;
	//LARGE_INTEGER nBeginTime;
	//LARGE_INTEGER nEndTime;
	//double time;
	//QueryPerformanceFrequency(&nFreq);
	//QueryPerformanceCounter(&nBeginTime);//测试程序运行时间
	////*******************************************

	////*******************获取运行时间**************
	//QueryPerformanceCounter(&nEndTime);
	//time=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
	//g_ComputingTime = time * 1000.0;
////*******************************************



int COneDOFDlg::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDialogEx::OnCreate(lpCreateStruct) == -1)
		return -1;


		return 0;
}


BOOL COneDOFDlg::OnEraseBkgnd(CDC* pDC)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值

	//return CDialogEx::OnEraseBkgnd(pDC);
	return true;

}


void COneDOFDlg::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	if(nChar == '0')
		m_nState = 0;
	else if(nChar == '1')
		m_nState = 1;
	MessageBox(_T("ChoosePixelFormat failed"));
	CDialogEx::OnKeyDown(nChar, nRepCnt, nFlags);
}


void COneDOFDlg::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
			SetCapture();	//抓住鼠标，即使鼠标移动到窗口之外，仍然有效
	m_nXPos=point.x;
	m_nYPos=point.y;
	CDialogEx::OnLButtonDown(nFlags, point);
}


void COneDOFDlg::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	if(GetCapture() == this) ReleaseCapture();//释放鼠标

	CDialogEx::OnLButtonUp(nFlags, point);
}


void COneDOFDlg::OnMouseHWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// 此功能要求 Windows Vista 或更高版本。
	// _WIN32_WINNT 符号必须 >= 0x0600。
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	m_fDist += 0.01f;
	Invalidate();

	MessageBox(_T("ChoosePixelFormat failed"));
	CDialogEx::OnMouseHWheel(nFlags, zDelta, pt);
}


void COneDOFDlg::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	if(MK_LBUTTON!=nFlags || GetCapture() != this) return;

	GLint dx,dy; //offset of mouse;

	dx = point.x-m_nXPos;
	dy = point.y-m_nYPos;
	m_nXPos = point.x;
	m_nYPos = point.y;

	if(m_nState == 0)
	{
		m_fYAngle += dx*0.5f;
		m_fXAngle += dy*0.5f;
	}
	else if(m_nState == 1)
		m_fDist += (dx+dy)*0.01f;

	Invalidate();	//激发OnDraw更新绘图



	CDialogEx::OnMouseMove(nFlags, point);
}


void COneDOFDlg::OnSize(UINT nType, int cx, int cy)
{
	CDialogEx::OnSize(nType, cx, cy);

	// TODO: 在此处添加消息处理程序代码
	CClientDC dc(GetDlgItem(IDC_PIC));
    wglMakeCurrent(dc.GetSafeHdc(), m_hRC);//当前的绘图上下文，每次都需要调用

    glViewport(0, 0, cx, cy);//视口的左下角点，长和宽

    wglMakeCurrent(dc.GetSafeHdc(), NULL);//NULL放弃绘图的上下文，交给别人使用

	Invalidate();	//激发OnDraw更新绘图
}


void COneDOFDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	//
	m_SphereX = m_Position_HapticTool[0];
	m_SphereY = m_Position_HapticTool[1];
	Invalidate();	//激发OnDraw更新绘图
	CDialogEx::OnTimer(nIDEvent);
}


void COneDOFDlg::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码
	CString str;
	GetDlgItem(IDC_BUTTON1)->GetWindowText(str);
	if( ( str == _T("开始记录") ) ) 
	{	
		frictionflag = 1;
		GetDlgItem(IDC_BUTTON1)->SetWindowText(_T("停止记录"));
	}
	else
	{
		frictionflag = 2;
		GetDlgItem(IDC_BUTTON1)->SetWindowText(_T("开始记录"));
	}

}
