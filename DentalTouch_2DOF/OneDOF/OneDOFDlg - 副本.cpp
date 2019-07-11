
// OneDOFDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "OneDOF.h"
#include "OneDOFDlg.h"
#include "afxdialogex.h"
#include "math.h"  

#ifdef _DEBUG
#define new DEBUG_NEW
#endif



//��ý�嶨ʱ����ر���
UINT g_uResolution = 1;//�ֱ���
CONST UINT g_uDelayHaptic = 2;//��������
CONST UINT g_uDelayGraphic = 30;//�Ӿ�����
MMRESULT g_TimerVR;	//�Ӿ���ʱ��
MMRESULT g_TimerHR;	//������ʱ��
//��ý�嶨ʱ���ص�����, ������Ⱦ��1ms
void CALLBACK HapticRendering(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2);
//��ý�嶨ʱ���ص�����, �Ӿ���Ⱦ,30ms
void CALLBACK GraphicRendering(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2);

//DAQ������صı�����������Ϊ�����յ���ǰ֡���̽���ɨ���жϣ������ǰ֡��Ч�����Լ�����һ֡������
//������Ⱦ������ʹ����һ������
#define DAQ_BUFFER_SIZE 16384	//����DAQ��صĻ����С
BYTE g_DAQReadBuffer[DAQ_BUFFER_SIZE] = {0};	//��ʼ��ȡ���ڵ�����
volatile double	g_DAQPositionBuffer[DAQ_BUFFER_SIZE][6] = {0.0};//˳��ѭ�����������123����̬RPY����ֵ��ÿһ�б�ʾһ�����ݵ㣬��λΪ����
volatile DWORD	g_DAQPositionIndex = 0;//��ǰg_DAQPositionBuffer������
DOUBLE	g_ComputingTime = 0.0; //��������ʱ��



//���������������
#define FORCE_BUFFER_SIZE 16384	//��صĻ����С
BYTE * g_ForceReadBuffer = new BYTE[FORCE_BUFFER_SIZE];	//��ʼ��ȡ�����ڵ�����
BYTE * g_ForceMyRxBuffer = new BYTE[FORCE_BUFFER_SIZE];	//ѭ�������ȡ������
DWORD	g_MyRxBufferCounter = 0;
DWORD	g_DataPtrOut = 0;//��һ����֡���һ���ֽ�����
volatile double	g_ForceBuffer[FORCE_BUFFER_SIZE][6] = {0.0};//˳��ѭ������Fx/FY/FZ/MX/MY/MZ
volatile DWORD	g_ForceIndex = 0;//��ǰg_ForceBuffer������
const double COneDOFDlg::m_dDecouplingCoefficient[6][6]=
{
	{0.190660744, 	0.223685906, 	0.534145445, 	18.003595500 ,	0.078319299, 	-17.726435630}, 
	{0.107527388 ,	-20.122044610 ,	0.477155640 ,	10.616147420 ,	0.295934085 ,	10.000624860}, 
	{56.159414210 ,	-0.330263321 ,	55.926562190 ,	1.128619676 ,	54.707066010 ,	-0.994680395}, 
	{-0.032886763, 	-0.013842118 ,	-0.308634583 ,	0.003593230 ,	0.251016213 ,	0.004968525 },
	{0.346667582 ,	0.000148175 ,	-0.131789607 ,	-0.016990855 ,	-0.143606749 ,	0.017654130 },
	{0.000069847 ,	0.125876500 ,	0.003589640 ,	0.118673608 ,	-0.000442646 ,	0.086879145 },

};


//��������ģ��
const double T = g_uDelayHaptic/1000.0;//����s
double vt = 0.0, vt_1 = 0.0;//�����ٶ�m/s
double xt = 0.0, xt_1 =0.0;//����λ��m
double Fv = 0.0;//������

int aa;
static double pp = 0.0;
int fen = 0;


#ifndef _COUNTS_ENCODER
#define	_COUNTS_ENCODER
	#define COUNTS_ENCODER 72000.0f//�������ֱ���
#endif

#ifndef _PI
#define	_PI
	#define PI 3.141592654f//Բ����
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



// ����Ӧ�ó��򡰹��ڡ��˵���� CAboutDlg �Ի���

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// �Ի�������
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

// ʵ��
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


// COneDOFDlg �Ի���



COneDOFDlg::COneDOFDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(COneDOFDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

	m_ComPortNumForce = 7;	//�����������ں�
	m_ComPortNumDAQ = 4;//DAQ���ں�

	m_bTimerRunning = FALSE;//��ý�嶨ʱû������

	m_bFlagInitDrive = FALSE;//��������ʼ����־λ
	m_bFlagInitForce = FALSE;//����������ʼ����־λ

	m_bFlagRunned = FALSE;//�Ѿ����й���


	m_bFlagCanConnect= FALSE;	//���ӱ�־λ
	m_bFlagForceConnect= FALSE;	//���ӱ�־λ
	m_bFlagDAQConnect= FALSE;	//���ӱ�־λ


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

	ON_MESSAGE(WM_COMM_RXSTR, OnComRecvData)//����cnComm.h�������ݻص�����

END_MESSAGE_MAP()


// COneDOFDlg ��Ϣ�������

BOOL COneDOFDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// ��������...���˵�����ӵ�ϵͳ�˵��С�

	// IDM_ABOUTBOX ������ϵͳ���Χ�ڡ�
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

	// ���ô˶Ի����ͼ�ꡣ��Ӧ�ó��������ڲ��ǶԻ���ʱ����ܽ��Զ�
	//  ִ�д˲���
	SetIcon(m_hIcon, TRUE);			// ���ô�ͼ��
	SetIcon(m_hIcon, FALSE);		// ����Сͼ��

	// TODO: �ڴ���Ӷ���ĳ�ʼ������

	return TRUE;  // ���ǽ��������õ��ؼ������򷵻� TRUE
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

// �����Ի��������С����ť������Ҫ����Ĵ���
//  �����Ƹ�ͼ�ꡣ����ʹ���ĵ�/��ͼģ�͵� MFC Ӧ�ó���
//  �⽫�ɿ���Զ���ɡ�

void COneDOFDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // ���ڻ��Ƶ��豸������

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// ʹͼ���ڹ����������о���
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// ����ͼ��
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//���û��϶���С������ʱϵͳ���ô˺���ȡ�ù��
//��ʾ��
HCURSOR COneDOFDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



/**����Ϊ�����Ӿ���Ⱦ��ý�嶨ʱ���̺߳���**************************************************/
//��ý�嶨ʱ���ص��������߳�, ������Ⱦ��1ms
void CALLBACK HapticRendering(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2)
{
	COneDOFDlg * obj = (COneDOFDlg*) dwUser;
	obj->MMTimerHapticRendering(wTimerID);

}

//��ý�嶨ʱ���ص��������߳�, �Ӿ���Ⱦ,30ms
void CALLBACK GraphicRendering(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2)
{
	COneDOFDlg * obj = (COneDOFDlg*) dwUser;
	obj->MMTimerGraphicRendering(wTimerID);
}

//������Ⱦ���̺߳�����1ms���ɶ�ý�嶨ʱ������
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
	double dforce =   ( g_ForceBuffer[temp][2] + g_ForceBuffer[temp1][2] + g_ForceBuffer[temp2][2]+g_ForceBuffer[temp3][2]  - 4.0*m_Force0[2])/4.0;//����������

	//����ģ�͹��ñ���
	static double dAngle = 0.0;//�ؽڽǶ�
	double v_angle = 0.0;
	const double RR = 180.0;//���˳���RΪ180mm

	////x=kfģ��
	//double kk = 1.0;
	//dAngle += dforce/kk/RR;//�����Ընȵ���λ��mm��dforce/kk/R����
	//if ( dAngle >= (30.0/180.0*PI) ) dAngle=30.0/180.0*PI;
	//if ( dAngle <= (-30.0/180.0*PI) ) dAngle=-30.0/180.0*PI;//����10������
	//v_angle = 90.0/180.0*PI;

	//����ģ��
	double m = 0.0;//��������kg
	static BOOL flag = FALSE;
	double k2 = 30.1*1000.0;
	double bb = 00.0;//����
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
	double a = FF/m;//���ٶ�m/s2
	vt = vt_1 + a * T;//�����ٶ�m/s
	//�ٶ��˲�
	vt = 0.9 * vt +  ( 1 - 0.9 ) * vt_1;

	vt_1 = vt;


	
	xt = xt_1 + vt * T;//Ŀ��λ��m
	xt_1 = xt;

	dAngle = xt/(RR/1000.0);//Ŀ��Ƕȣ�rad
	v_angle = vt/(RR/1000.0);//ת���ٶ�rad/s

	if ( dAngle >= (30.0/180.0*PI) ) dAngle=30.0/180.0*PI;
	if ( dAngle <= (-30.0/180.0*PI) ) dAngle=-30.0/180.0*PI;//����10������
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





	//����Ŀ��λ���ٶ�
	//m_CANCmdFrame[0] = * m_CANCmd.GetFrame(MODE_POSITION, dAngle);
	//m_CANCmdFrame[1] = * m_CANCmd.GetFrame(MODE_VELOCITY, fabs(v_angle));//(360.0/180.0*PI)
	//m_CANCmdFrame[2] = * m_CANCmd.GetFrame(MODE_ENABLEMOTION);//ʹ��
	//m_CANCmdFrame[3] = * m_CANCmd.GetFrame(MODE_STARTMOTION);//��ʼ����
	//Transmit(m_DeviceType,0,0,m_CANCmdFrame,4);//ע����4֡һ���ͣ�����һ�Σ�1-5֡�ķ�Χ��Ŷ���0.5ms

	CString str;
	str.Format(_T("�Ƕȣ�%.5f\t,�ٶȣ�%.5f\t,dforce��%.5f\t"), dAngle/PI*180.0,v_angle/PI*180.0,dforce);
    //str.Format(_T("x��%.5f\tv��%.5f\ta:%.5f\tdforce��%.5f\tFv:%.5f"), xt,vt,a,dforce,Fv);
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



//�Ӿ���Ⱦ,30ms���ɶ�ý�嶨ʱ������
void COneDOFDlg::MMTimerGraphicRendering(UINT nIDEvent)
{
// do what you want to do, but quickly

	DWORD temp;
	CString str;
	temp = g_DAQPositionIndex;
	str.Format(_T("%.2f,\n  %.2f,\n  %.2f,\n  %.2f,\n  %.2f,\n  %.2f,\n g_DAQPositionIndex:%d,\n   g_ComputingTime:%f,\n  ����֡����%d,    pp:%f"),
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
	//str.Format(_T("%.2f,\n  %.2f,\n  %.2f,\n  %.2f,\n  %.2f,\n  %.2f,\n g_DAQPositionIndex:%2d,\n   g_ComputingTime:%.2f,\n  ����֡����%2d,    pp:%.2f"),
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

BOOL COneDOFDlg::ProcessForceData(DWORD len)//����������������
{

	//***************��������******************
	//Head fram	  PackageLength	 DataNo	       Data	       ChkSum
	//0xAA,0x55	     HB,LB         2B   (ChNum*N*DNpCH) B	 1B		//(ChNum*N*DNpCH)Ϊ6*2*1=12B�����Թ���19�ֽ�
	//BYTE g_ForceReadBuffer[FORCE_BUFFER_SIZE] = {0};	//��ʼ��ȡ�����ڵ�����
	//volatile double	g_ForceBuffer[FORCE_BUFFER_SIZE][6] = {0.0};//˳��ѭ������Fx/FY/FZ/MX/MY/MZ
	//volatile DWORD	g_ForceIndex = 0;//��ǰg_ForceBuffer������

	if(g_ForceMyRxBuffer == NULL) {return FALSE;}
	if(g_DataPtrOut == 0xFFFFFFFF)	{return FALSE;}

	DWORD HeaderIndex =0x00;//ָ��0x55��������������������0��ʼ
	BOOL DataHeaderFlag = FALSE;//ͷ�ı�־λ
	DWORD PackageLength= 0xFFFFFFFF; 
	DWORD HighIndex,LowIndex;
	DWORD RxLengthTemp = 0x00;
	DWORD RxCounterCurrent = g_MyRxBufferCounter;
	DWORD i = 0x00;

	while (1)
	{
		//--------------------------------------------------------------------------------	
		//���ݳ��ȣ���g_DataPtrOut����
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
			break;//�˳�while
		}
		//--------------------------------------------------------------------------------
		//Ѱ��֡ͷ
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
		//Now, HeaderIndex point to��0xAA��
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
			g_DataPtrOut = MoveIndex;//ָ��checksum
			break;
		}
		//---------------------------------------------------------------------------------
		BYTE DataTemp[12] = {0};//��ʱ�������ݣ�6ͨ����һ����
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
		//����ADcounts
		INT ADCounts[6] = {0};//�����ȡ��AD
		for( i =0; i < 6; i++ )
		{
			ADCounts[i] = DataTemp[2*i]<<8 |  DataTemp[2*i+1];
		}
		//����ÿ��ͨ��
		DOUBLE	ChValue[6]={0.0};
		for ( i = 0; i < 6; i++)
		{
			ChValue[i] = 1000.0 * ( ADCounts[i] - m_dAmpZero[i]) / (double)65535.0 * (double)5  / m_dChnGain[i] / m_dChnEx[i];  
		}
		//���ý�����������ֵ
		DOUBLE ForceTemp[6] = {0.0};
		for( i = 0x00; i < 6; i++)
		{
			ForceTemp[i] =   ChValue[0]*m_dDecouplingCoefficient[i][0] + ChValue[1]*m_dDecouplingCoefficient[i][1]
									+ ChValue[2]*m_dDecouplingCoefficient[i][2] + ChValue[3]*m_dDecouplingCoefficient[i][3]
									+ ChValue[4]*m_dDecouplingCoefficient[i][4] + ChValue[5]*m_dDecouplingCoefficient[i][5];
		}
		//�������ݣ�g_ForceIndexʼ��ָ������һ������
		DWORD ForceNextIndex = g_ForceIndex + 1;//ָ����һ���洢λ�ã�ע��g_ForceIndex����
		if( ForceNextIndex >= FORCE_BUFFER_SIZE)
			ForceNextIndex = ForceNextIndex - FORCE_BUFFER_SIZE;//ע��ѭ�����������

		for( i = 0; i <6; i++)
		{
			g_ForceBuffer[ForceNextIndex][i] = ForceTemp[i];//��������
		
		}
		g_ForceIndex = ForceNextIndex;//������Ч��ָ����һ����Ч���ݣ������߳�ͨ��g_ForceIndex��������
		
		//CString str;
		//str.Format( _T("g_ComputingTime=%.5f\n,g_ForceIndex=%d"),	g_ComputingTime, ForceNextIndex );
		//GetDlgItem(IDC_EDIT2)->SetWindowText(str);
	}
	return TRUE;
}



/**����Ϊ����������DAQ���ڽ��ճ���******************************************/
LRESULT  COneDOFDlg::OnComRecvData(WPARAM str, LPARAM commInfo)
{
	struct serialPortInfo
	{
		UINT portNr;//���ں�
		DWORD bytesRead;//��ȡ���ֽ���
	}*pCommInfo;
	pCommInfo = (serialPortInfo*)commInfo;

	if (pCommInfo->portNr == m_ComPortNumForce)//�������������ź�
	{
		////*******���ڼ�����������֮���ʱ����*******
		//static int aaa = 0;
		//static LARGE_INTEGER nFreq;
		//static LARGE_INTEGER nBeginTime;
		//static LARGE_INTEGER nEndTime;
		//static double time;

		//if(0 == aaa)
		//{
		//	QueryPerformanceFrequency(&nFreq);
		//	QueryPerformanceCounter(&nBeginTime);//���Գ�������ʱ��
		//}

		//aaa++;

		//if(2 == aaa)
		//{
		//	QueryPerformanceCounter(&nEndTime);
		//	time=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
		//	g_ComputingTime = time * 1000.0;//����ʱ��Ϊ0.01ms
		//	aaa=0;
		//}
		////********************************************
		

		////*******************��������ʱ��**************
		//LARGE_INTEGER nFreq;
		//LARGE_INTEGER nBeginTime;
		//LARGE_INTEGER nEndTime;
		//double time;
		//QueryPerformanceFrequency(&nFreq);
		//QueryPerformanceCounter(&nBeginTime);//���Գ�������ʱ��
		////*******************************************

		DWORD len = pCommInfo->bytesRead;
		DWORD i = 0x00;
		//��ȡԭʼ����
		memcpy(g_ForceReadBuffer,(BYTE*)str,len);

		for( i =0; i < len; i++)
		{
			if(g_MyRxBufferCounter >= FORCE_BUFFER_SIZE)
			{
				g_MyRxBufferCounter = 0;
			}
			g_ForceMyRxBuffer[g_MyRxBufferCounter++] =  g_ForceReadBuffer[i];//����ԭʼ���ݣ�g_MyRxBufferCounter���ָ�����ݵ���һ��λ
		}

		ProcessForceData(len);//����������������
	}


	if(pCommInfo->portNr == m_ComPortNumDAQ)//����DAQ�ź�
	{
	
		////*******************��������ʱ��**************
		//LARGE_INTEGER nFreq;
		//LARGE_INTEGER nBeginTime;
		//LARGE_INTEGER nEndTime;
		//double time;
		//QueryPerformanceFrequency(&nFreq);
		//QueryPerformanceCounter(&nBeginTime);//���Գ�������ʱ��
		////*******************************************

		DWORD len = pCommInfo->bytesRead;
		DWORD HeaderIndex =0x00;//ָ��0x55��������������������0��ʼ
		DWORD i = 0x00;

		memcpy(g_DAQReadBuffer,(BYTE*)str,len);

		aa = len;

		//��ֵ̬������ֵһ֡����22B,��ʱ0.3ms,�������㣩 0x55 0xaa Lenth(1B) Ecd1(4B) Ecd2(4B) Ecd3(4B) R(2B) P(2B) Y(2B) CheckSum(1B)
		//�ֽ���У��
		if( len <= 21 )//����ܳ�С��22�ֽڷ���
			return -1;
		//AfxMessageBox(_T("len <= 21"));
		//����ͷ��
		for( i=0; i<(len-1); i++)
		{
			if( g_DAQReadBuffer[i] == 0x55 && g_DAQReadBuffer[i+1] == 0xAA )
			{
				HeaderIndex = i;//��ʱiΪͷ
				break;
			}
			if( i == (len-2) )
				return -1;//�Ҳ���ͷ
		}
		//AfxMessageBox(_T("����ͷ��"));
		//��HeaderIndex������len-1������������һ���ֽڣ�����Ϊ>22���ֽ�
		if( (len-HeaderIndex) <= 21 )
			return -1;
		//AfxMessageBox(_T("len-HeaderIndex) <= 21 "));
		//����Length(1B)У��

		////CheckSum(1B)У��
		//DWORD CheckSum =0;
		//for( i = (HeaderIndex+3); i < (HeaderIndex+21); i++)
		//{
		//	CheckSum += g_DAQReadBuffer[i];
		//}
		//if( CheckSum != g_DAQReadBuffer[HeaderIndex+21])//У��Ͳ���
		//	return -1;
		//��������
		DWORD32 TempCountsEncoder = 0;//�������������
		DWORD TempRPY = 0;//����RPY����
		DOUBLE PositionTemp[6] = {0.0};//��˳�򱣴�ABC��������RPY�Ƕ�

		TempCountsEncoder = (g_DAQReadBuffer[3+HeaderIndex]<<24) | (g_DAQReadBuffer[4+HeaderIndex]<<16)
							| (g_DAQReadBuffer[5+HeaderIndex]<<8) | g_DAQReadBuffer[6+HeaderIndex];
		PositionTemp[0] = TempCountsEncoder / COUNTS_ENCODER * 2* PI;//A����

		TempCountsEncoder = (g_DAQReadBuffer[7+HeaderIndex]<<24) | (g_DAQReadBuffer[8+HeaderIndex]<<16)
							| (g_DAQReadBuffer[9+HeaderIndex]<<8) | g_DAQReadBuffer[10+HeaderIndex];
		PositionTemp[1] = TempCountsEncoder / COUNTS_ENCODER * 2* PI;//B����

		TempCountsEncoder = (g_DAQReadBuffer[11+HeaderIndex]<<24) | (g_DAQReadBuffer[12+HeaderIndex]<<16)
							| (g_DAQReadBuffer[13+HeaderIndex]<<8) | g_DAQReadBuffer[14+HeaderIndex];
		PositionTemp[2] = TempCountsEncoder / COUNTS_ENCODER * 2* PI;//C����

		TempRPY = (g_DAQReadBuffer[15+HeaderIndex]<<8) | g_DAQReadBuffer[16+HeaderIndex];
		PositionTemp[3] = TempRPY / 32768.0f * PI;//R����

		TempRPY = (g_DAQReadBuffer[17+HeaderIndex]<<8) | g_DAQReadBuffer[18+HeaderIndex];
		PositionTemp[4] = TempRPY / 32768.0f * PI;//R����

		TempRPY = (g_DAQReadBuffer[19+HeaderIndex]<<8) | g_DAQReadBuffer[20+HeaderIndex];
		PositionTemp[5] = TempRPY / 32768.0f * PI;//R����

		//g_DAQPositionIndexʼ��ָ������һ������
		DWORD NextIndex = g_DAQPositionIndex + 1;//ָ����һ���洢λ�ã�ע��g_DAQPositionBuffer����
		if( NextIndex >= DAQ_BUFFER_SIZE)
			NextIndex = NextIndex - DAQ_BUFFER_SIZE;//ע��ѭ�����������

		for( i = 0; i <6; i++)
		{
			g_DAQPositionBuffer[NextIndex][i] = PositionTemp[i];//��������
		
		}
		g_DAQPositionIndex = NextIndex;//������Ч��ָ����һ����Ч���ݣ������߳�ͨ��g_DAQPositionIndex��������

		//AfxMessageBox(_T("У��Ͳ��� "));
		////���ݴ���ͱ���
		//BYTE aa[18] = {0};
		//for( i=0; i< 18; i++)
		//{
		//	aa[i] = g_DAQReadBuffer[i+HeaderIndex+3];
		//}

		//m_ComPortForce.Write(aa,18);



	////*******************��ȡ����ʱ��**************
	//QueryPerformanceCounter(&nEndTime);
	//time=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
	//g_ComputingTime = time * 1000.0;
	////*******************************************


	}
	return 0;
}


/***����Ϊ��Ϣ��Ӧ����************************************************************/
//������������
void COneDOFDlg::OnBnClickedBtnConnectForce()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString str;
	GetDlgItem(IDC_BTN_CONNECT_FORCE)->GetWindowText(str);

	if( m_bFlagForceConnect == FALSE )
	{
		if(!m_ComPortForce.InitPort(*this,m_ComPortNumForce,1000000))	//����򿪴���ʧ��
		{	
			AfxMessageBox(_T("������������ʧ�ܣ�"));
			return;
		}
		else//�򿪳ɹ�
		{
			m_bFlagForceConnect = TRUE;
			m_ComPortForce.StartMonitoring();
			GetDlgItem(IDC_BTN_CONNECT_FORCE)->SetWindowText(_T("�Ͽ���������"));
			AfxMessageBox(_T("�������������ɹ���"));
			Sleep(50);//ʹ������������һ�������ĳ�ʼֵ

		}
	}
	else
	{
		//�����������Ӿ���Ⱦ
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

		m_ComPortForce.ClosePort(); //�رմ���
		m_bFlagForceConnect = FALSE;
		GetDlgItem(IDC_BTN_CONNECT_FORCE)->SetWindowText(_T("������������"));
		GetDlgItem(IDC_BTN_START)->SetWindowText(_T("��ʼ"));
	}

}

//������������
void COneDOFDlg::OnBnClickedBtnConfigureForce()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString	CmdStr;
	LPTSTR  pszTest;
	CHAR *pChar;
	INT len;

	if(m_bFlagForceConnect == FALSE)
	{
		AfxMessageBox(_T("��������������ڣ�"));
		return;
	}
	//�õ�ʵʱ���ݲ��裺
	//��һ�������þ���AT+DCPM=����������ǰ�����ã�
	//Set the decoupling matrix unit by command DCPCU������ǰ�����ã�
	//Send command AT+SGDM=(A01,A02,A03,A04,A05,A06);C;1;(WMA:1)\r\n to set the mode to receive data.
	//AT+SMPR=1000\r\n	//���ò���Ƶ��
	//AT+GSD\r\nѭ������


	CmdStr = _T("AT+SMPR=1000\r\n");	//���ò���Ƶ��
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

	AfxMessageBox(_T("���óɹ���"));

	GetDlgItem(IDC_BTN_CONFIGURE_FORCE)->EnableWindow(FALSE);

	m_bFlagInitForce = TRUE;

	//m_CANCmdFrame[0] = * m_CANCmd.GetFrame(MODE_SETPOSITIONZERO);//!!!��Ҫɾ�������õ����ǰλ��Ϊ0��������ȷ
	//Transmit(m_DeviceType,0,0,m_CANCmdFrame,1);

}

//��DAQ����
void COneDOFDlg::OnBnClickedBtnOpenDaqPort()
{
	//// TODO: �ڴ���ӿؼ�֪ͨ����������
	//CString str;
	//GetDlgItem(IDC_BTN_OPEN_DAQ_PORT)->GetWindowText(str);
	//if( m_bFlagDAQConnect == FALSE ) //( str == _T("��DAQ����") )
	//{
	//	if(!m_ComPortDAQ.InitPort(*this,m_ComPortNumDAQ,1000000))	//����򿪴���ʧ��
	//	{	
	//		AfxMessageBox(_T("DAQ���ڲ����ڻ�ռ�ã�"));
	//		return;
	//	}
	//	else//�򿪳ɹ�
	//	{

	//		m_bFlagDAQConnect = TRUE;
	//		m_ComPortDAQ.StartMonitoring();
	//		GetDlgItem(IDC_BTN_OPEN_DAQ_PORT)->SetWindowText(_T("�ر�DAQ����"));

	//		Sleep(50);//ʹDAQ����һ�������ĳ�ʼֵ
	//	}
	//}
	//else// if ( m_ComPortDAQ.IsOpen() )//( str == _T("�ر�DAQ����") )//�رմ���
	//{
	//	//�����������Ӿ���Ⱦ
	//	timeKillEvent(g_TimerVR);
	//	timeKillEvent(g_TimerHR);
	//	timeEndPeriod (g_uResolution);

	//	m_ComPortDAQ.ClosePort(); //�رմ���
	//	m_bFlagDAQConnect = FALSE;
	//	GetDlgItem(IDC_BTN_OPEN_DAQ_PORT)->SetWindowText(_T("��DAQ����"));
	//	GetDlgItem(IDC_BTN_START)->SetWindowText(_T("��ʼ"));
	//}



	//m_CANCmdFrame[0] = * m_CANCmd.GetFrame(MODE_SETPOSITIONZERO);//!!!��Ҫɾ�������õ����ǰλ��Ϊ0��������ȷ
	//Transmit(m_DeviceType,0,0,m_CANCmdFrame,1);

	//m_CANCmdFrame[0] = * m_CANCmd.GetFrame(MODE_POSITION, 100000.0/180.0*PI);
	//m_CANCmdFrame[1] = * m_CANCmd.GetFrame(MODE_VELOCITY, 180.0/180.0*PI);
	//m_CANCmdFrame[2] = * m_CANCmd.GetFrame(MODE_ENABLEMOTION);//ʹ��
	//m_CANCmdFrame[3] = * m_CANCmd.GetFrame(MODE_STARTMOTION);//��ʼ����
	//Transmit(m_DeviceType,0,0,m_CANCmdFrame,4);//ע����4֡һ���ͣ�����һ�Σ�1-5֡�ķ�Χ��Ŷ���0.5ms


}

//��CAN����
void COneDOFDlg::OnBnClickedBtnOpenCanPort()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	if( m_bFlagCanConnect == TRUE)//Ҫ�ر�CAN
	{
		//�����������Ӿ���Ⱦ
		timeKillEvent(g_TimerVR);
		timeKillEvent(g_TimerHR);
		timeEndPeriod (g_uResolution);

		m_bFlagCanConnect = FALSE;
		GetDlgItem(IDC_BTN_OPEN_CAN_PORT)->SetWindowText(_T("��CAN����"));
		GetDlgItem(IDC_BTN_START)->SetWindowText(_T("��ʼ"));
		CloseDevice(m_DeviceType,0);

		return;
	}

	INIT_CONFIG init_config;//CAN��ʼ�����ýṹ��
	init_config.AccCode = 0;
    init_config.AccMask =0xffffff;
    init_config.Filter = 0;
	init_config.Timing0 = 0;
    init_config.Timing1 =0x14;//������Ϊ10000kHZ
	init_config.Mode = 0;

	if( OpenDevice(m_DeviceType,0,0) != STATUS_OK )
	{
		MessageBox(_T("��CAN����ʧ�ܣ�"),_T("����"),MB_OK|MB_ICONQUESTION);
		return;
	}

	//MessageBox(_T("��CAN���ڳɹ���"),_T("����"),MB_OK|MB_ICONQUESTION);

	if( InitCAN(m_DeviceType,0,0,&init_config) != STATUS_OK )
	{
		MessageBox(_T("��ʼ��CAN����ʧ�ܣ�"),_T("����"),MB_OK|MB_ICONQUESTION);
		CloseDevice(m_DeviceType,0);
		return;
	}
	//MessageBox(_T("��ʼ��CAN���ڳɹ���"),_T("����"),MB_OK|MB_ICONQUESTION);

	//AfxBeginThread(ReceiveThread,this);//�����Ҫ�������ݣ������߳�

	if( StartCAN(m_DeviceType,0,0) == 1 )
	{
		MessageBox(_T("����CAN�ɹ���"),_T("����"),MB_OK|MB_ICONQUESTION);
		m_bFlagCanConnect = TRUE;		//��־λ
		GetDlgItem(IDC_BTN_OPEN_CAN_PORT)->SetWindowText(_T("�Ͽ�CAN����"));
	}
	else
	{
		MessageBox(_T("����CANʧ�ܣ�"),_T("����"),MB_OK|MB_ICONQUESTION);	
		CloseDevice(m_DeviceType,0);
	}

	Sleep(50);

}

//��ʼ��������
void COneDOFDlg::OnBnClickedBtnInitialDrive()
{
	//TODO: �ڴ���ӿؼ�֪ͨ����������
	//if( m_bFlagCanConnect == FALSE)
	//{
	//	MessageBox(_T("���ȴ�CAN���ڣ�"),_T("����"),MB_OK|MB_ICONQUESTION);
	//	return;
	//}

	//m_CANCmdFrame[0] = * m_CANCmd.GetFrame(MODE_STARTCAN);//��ȡ����֡����һ������can
	//m_CANCmdFrame[1] = * m_CANCmd.GetFrame(MODE_CANMODE);//�ڶ�����canģʽ
	//m_CANCmdFrame[2] = * m_CANCmd.GetFrame(MODE_OPERATIONMODE);//��������λ��ģʽ
	//m_CANCmdFrame[3] = * m_CANCmd.GetFrame(MODE_PROFILETYPE);//���Ĳ�������ģʽ
	//m_CANCmdFrame[4] = * m_CANCmd.GetFrame(MODE_ENABLEMOTION);//���岽��ʹ��

	//if( 5 == Transmit(m_DeviceType,0,0,m_CANCmdFrame,5) )
	//{
	//	m_CANCmdFrame[0] = * m_CANCmd.GetFrame(MODE_SETPOSITIONZERO);//!!!��Ҫɾ�������õ����ǰλ��Ϊ0��������ȷ
	//	Transmit(m_DeviceType,0,0,m_CANCmdFrame,1);
	//	MessageBox(_T("��ʼ����������ɣ�"),_T("����"),MB_OK|MB_ICONQUESTION);
	//	GetDlgItem(IDC_BTN_INITIAL_DRIVE)->EnableWindow(FALSE);
	//	m_bFlagInitDrive = TRUE;
	//}
	//else
	//	MessageBox(_T("��ʼ��������ʧ�ܣ�"),_T("����"),MB_OK|MB_ICONQUESTION);

	if( m_bFlagCanConnect == FALSE)
	{
		MessageBox(_T("���ȴ�CAN���ڣ�"),_T("����"),MB_OK|MB_ICONQUESTION);
		return;
	}

	m_CANCmdFrame[0] = * m_CANCmd.GetFrame(0, MODE_STARTCAN);//��ȡ����֡����һ������can
	Transmit(m_DeviceType,0,0,m_CANCmdFrame,1);

	m_CANCmdFrame[1] = * m_CANCmd.GetFrame(1, MODE_CANMODE);//�ڶ�����canģʽ
	m_CANCmdFrame[2] = * m_CANCmd.GetFrame(2, MODE_CANMODE);//�ڶ�����canģʽ
	m_CANCmdFrame[3] = * m_CANCmd.GetFrame(3, MODE_CANMODE);//�ڶ�����canģʽ

	m_CANCmdFrame[4] = * m_CANCmd.GetFrame(1, MODE_OPERATIONMODE);//��������λ��ģʽ
	m_CANCmdFrame[5] = * m_CANCmd.GetFrame(2, MODE_OPERATIONMODE);//��������λ��ģʽ
	m_CANCmdFrame[6] = * m_CANCmd.GetFrame(3, MODE_OPERATIONMODE);//��������λ��ģʽ

	m_CANCmdFrame[7] = * m_CANCmd.GetFrame(1, MODE_PROFILETYPE);//���Ĳ�������ģʽ
	m_CANCmdFrame[8] = * m_CANCmd.GetFrame(2, MODE_PROFILETYPE);//���Ĳ�������ģʽ
	m_CANCmdFrame[9] = * m_CANCmd.GetFrame(3, MODE_PROFILETYPE);//���Ĳ�������ģʽ

	m_CANCmdFrame[10] = * m_CANCmd.GetFrame(1, MODE_ENABLEMOTION);//���岽��ʹ��
	m_CANCmdFrame[11] = * m_CANCmd.GetFrame(2, MODE_ENABLEMOTION);//���岽��ʹ��
	m_CANCmdFrame[12] = * m_CANCmd.GetFrame(3, MODE_ENABLEMOTION);//���岽��ʹ��

	if( 13 == Transmit(m_DeviceType,0,0,m_CANCmdFrame,13) )
	{
		m_CANCmdFrame[0] = * m_CANCmd.GetFrame(1, MODE_SETPOSITIONZERO);//!!!��Ҫɾ�������õ����ǰλ��Ϊ0��������ȷ
		m_CANCmdFrame[1] = * m_CANCmd.GetFrame(2, MODE_SETPOSITIONZERO);//!!!��Ҫɾ�������õ����ǰλ��Ϊ0��������ȷ
		m_CANCmdFrame[2] = * m_CANCmd.GetFrame(3, MODE_SETPOSITIONZERO);//!!!��Ҫɾ�������õ����ǰλ��Ϊ0��������ȷ

		Transmit(m_DeviceType,0,0,m_CANCmdFrame,3);
		MessageBox(_T("��ʼ����������ɣ�"),_T("����"),MB_OK|MB_ICONQUESTION);
		GetDlgItem(IDC_BTN_INITIAL_DRIVE)->EnableWindow(FALSE);
		m_bFlagInitDrive = TRUE;
	
	
		m_CANCmdFrame[0] = * m_CANCmd.GetFrame(1, MODE_POSITION, 1000);
		m_CANCmdFrame[1] = * m_CANCmd.GetFrame(2, MODE_POSITION, 1000);
		m_CANCmdFrame[2] = * m_CANCmd.GetFrame(3, MODE_POSITION, 1000);
		m_CANCmdFrame[3] = * m_CANCmd.GetFrame(1, MODE_VELOCITY, (60.0/180.0*PI));//(360.0/180.0*PI)
		m_CANCmdFrame[4] = * m_CANCmd.GetFrame(2, MODE_VELOCITY, (60.0/180.0*PI));//(360.0/180.0*PI)
		m_CANCmdFrame[5] = * m_CANCmd.GetFrame(3, MODE_VELOCITY, (60.0/180.0*PI));//(360.0/180.0*PI)
		m_CANCmdFrame[6] = * m_CANCmd.GetFrame(1, MODE_ENABLEMOTION);//ʹ��
		m_CANCmdFrame[7] = * m_CANCmd.GetFrame(2, MODE_ENABLEMOTION);//ʹ��
		m_CANCmdFrame[8] = * m_CANCmd.GetFrame(3, MODE_ENABLEMOTION);//ʹ��
		m_CANCmdFrame[9] = * m_CANCmd.GetFrame(1, MODE_STARTMOTION);//��ʼ����
		m_CANCmdFrame[10] = * m_CANCmd.GetFrame(2, MODE_STARTMOTION);//��ʼ����
		m_CANCmdFrame[11] = * m_CANCmd.GetFrame(3, MODE_STARTMOTION);//��ʼ����
		Transmit(m_DeviceType,0,0,m_CANCmdFrame,12);//ע����4֡һ���ͣ�����һ�Σ�1-5֡�ķ�Χ��Ŷ���0.5ms
	
	
	}
	else
		MessageBox(_T("��ʼ��������ʧ�ܣ�"),_T("����"),MB_OK|MB_ICONQUESTION);





}

//��ʼ���򣬼����Ӿ��ʹ�����ý�嶨ʱ��
void COneDOFDlg::OnBnClickedBtnStart()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	
	//if( m_bFlagRunned == TRUE )	//����Ѿ����й�һ��
	//{
	//	MessageBox(_T("����������Դ����������"),_T("����"),MB_OK|MB_ICONQUESTION);
	//	return;
	//} 

	////�ж϶˿��Ƿ񶼴�
	//if( !m_ComPortForce.IsOpen() )
	//{
	//	MessageBox(_T("������������û�д򿪣�"),_T("����"),MB_OK|MB_ICONQUESTION);
	//	return;
	//} 
	//if( !m_ComPortDAQ.IsOpen() )
	//{
	//	MessageBox(_T("DAQ����û�д򿪣�"),_T("����"),MB_OK|MB_ICONQUESTION);
	//	return;
	//}
	//if( !m_CanConnectFlag )
	//{
	//	MessageBox(_T("CAN����û�д򿪣�"),_T("����"),MB_OK|MB_ICONQUESTION);
	//	return;
	//}
	//
	//if( ! m_bFlagInitDrive )
	//{
	//	MessageBox(_T("������û�г�ʼ����"),_T("����"),MB_OK|MB_ICONQUESTION);
	//	return;
	//}

	//if( ! m_bFlagInitForce )
	//{
	//	MessageBox(_T("��������û�����ã�"),_T("����"),MB_OK|MB_ICONQUESTION);
	//	return;
	//}




	//��ʼ����������������˳�ʼλ�á���������ʼλ�á������г�ʼֵ�����������Ӳ����
	//��Ҫ��ֹ�����������¹�����ɵĴ���
	//�������������������1���������й���������������������Ӳ����Դ����ÿ�����г��򣨰������ֹͣ�ٿ�ʼ��ǰ��Ҫ������Դ
	//2���Ե�ǰ�����ťʱ��״̬Ϊ��ʼ״̬������������һ����ʼ��F0��������һ����ʼ�Ƕ�E0�����Ӧ��Ҳ��һ��λ��M0
	//�������޷���ã��������������Դ������һ��ָ�����õ��λ��Ϊ�㣩��֮�����еļ��㶼�ǻ����������

	//��ʼ������

	INT temp= g_ForceIndex;				//�Ե�ǰ����Ϊ0��
	m_Force0[0] = g_ForceBuffer[temp][0];
	m_Force0[1] = g_ForceBuffer[temp][1];
	m_Force0[2] = g_ForceBuffer[temp][2];
	m_Force0[3] = g_ForceBuffer[temp][3];
	m_Force0[4] = g_ForceBuffer[temp][4];
	m_Force0[5] = g_ForceBuffer[temp][5];




	//��ʼ������Ⱦ
	CString str;
	GetDlgItem(IDC_BTN_START)->GetWindowText(str);
	if ( m_bTimerRunning == FALSE) //( str == _T("��ʼ") )
	{
		//��ʾ�û���ȷ����������ǰ����Ӳ��
		static WORD confirm = 0;
		confirm++;
		if ( confirm == 1 )
		{
			MessageBox(_T("ȷ��Ӳ������������"),_T("����"),MB_OK|MB_ICONQUESTION);
			return;
		}
		confirm = 0;

		//������ý�嶨ʱ��
		TIMECAPS tc;
		if( timeGetDevCaps(&tc, sizeof(TIMECAPS)) == TIMERR_NOERROR )//��ȡ���������ֱ���
		{
			g_uResolution = min(max(tc.wPeriodMin, 0), tc.wPeriodMax);//Ϊ���ֵ
			timeBeginPeriod(g_uResolution);//���÷ֱ���
		}
		g_TimerHR = timeSetEvent(
								g_uDelayHaptic,
								g_uResolution,
								HapticRendering,
								(DWORD)this,
								TIME_PERIODIC);//������GRʱ����ı���Ῠ�������ϴ��з���ֵ��can����ָ��Ῠ��

	/*	g_TimerVR = timeSetEvent(
								g_uDelayGraphic,
								g_uResolution,
								GraphicRendering,
								(DWORD)this,
								TIME_PERIODIC);*/
		
		GetDlgItem(IDC_BTN_START)->SetWindowText(_T("ֹͣ"));
		 m_bTimerRunning = TRUE;
	}
	else if ( m_bTimerRunning == TRUE )//(str == _T("ֹͣ"))
	{
		timeKillEvent(g_TimerVR);
		timeKillEvent(g_TimerHR);
		timeEndPeriod (g_uResolution);

		m_bTimerRunning = FALSE;

		GetDlgItem(IDC_BTN_START)->SetWindowText(_T("��ʼ"));

		m_bFlagInitDrive = FALSE;
		m_bFlagInitForce = FALSE;

		m_bFlagRunned = TRUE;
	}


}

void COneDOFDlg::OnDestroy()
{
	CDialogEx::OnDestroy();
	// TODO: �ڴ˴������Ϣ����������
	//��ý�嶨ʱ������
	timeKillEvent(g_TimerVR);
	timeKillEvent(g_TimerHR);
	timeEndPeriod (g_uResolution);

	//��������
	if(m_ComPortDAQ.IsOpen()) m_ComPortDAQ.ClosePort(); //�رմ���
	if(m_ComPortCAN.IsOpen()) m_ComPortCAN.ClosePort(); //�رմ���
	if(m_ComPortForce.IsOpen()) m_ComPortForce.ClosePort(); //�رմ���


}

