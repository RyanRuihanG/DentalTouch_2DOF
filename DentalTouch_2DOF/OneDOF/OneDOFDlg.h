
// OneDOFDlg.h : ͷ�ļ�
//

#pragma once

#include "SerialPort.h"//����ͨ��ͷ�ļ�
#include "mmsystem.h"  //head file����ý�嶨ʱ��ͷ�ļ�
#include "EcanVci.h"//can��ͷ�ļ�
#include "CANCmd.h"	//CANָ����Լ���д
#include "afxwin.h"
#include "afxcmn.h"
#include "FiveBarKinematics.h"
#include "bdaqctrl.h"			//�л�����ͷ�ļ�
using namespace Automation::BDaq;


#ifndef _PI
#define	_PI
	#define PI 3.141592654f//Բ����
#endif

#define POSITION_DIMENTION			2	//λ��ά��
#define NUMBER_JOINTS				2	//�����ؽ�����
#define LINK_ENCODER_COUNTS			72000.0	//���˱���������
#define INIT_X						110.99419336		//��ʼxy�Լ���Ӧ�Ĺؽڽ�rad
#define INIT_Y						78.70562001
#define INIT_ANGLE_DPC1				(-20.00193979/180.0*PI)
#define INIT_ANGLE_DPC2				(65.00292158/180.0*PI)
#define INIT_ANGLE_LINK1			(-20.00193979/180.0*PI)
#define INIT_ANGLE_LINK2			(65.00292158/180.0*PI)

#define MAX_ANGLE_DPC1				(116.0/180.0*PI)		//�����С�ؽ���λ
#define MIN_ANGLE_DPC1				(-21.0/180.0*PI)
#define MAX_ANGLE_DPC2				(201.0/180.0*PI)
#define MIN_ANGLE_DPC2				(64.0/180.0*PI)

#define MAX_X						100.0			//ĩ���������������ڣ�����ʱ�򲻸�������Լ���Ƕ�
#define MIN_X						(-100.0)
#define MAX_Y						100.0
#define MIN_Y						(-100.0)

#define MAX_WALL_X					100.0			//����ǽ����Ϊֱ�߶Σ�������ֱ��
#define MIN_WALL_X					(-100.0)
//#define MAX_WALL_Y					100.0
//#define MIN_WALL_Y					(-100.0)

#define ALFA1						(1.5/180.0*PI)//�ؽ�1���Ƽ�϶
#define ALFA2						(1.5/180.0*PI)//
#define BETA1						(17.0375/180.0*PI)//�ؽ�2��е��϶
#define BETA2						(16.85/180.0*PI)

typedef struct tagDevConfParam
{
	int deviceNumber;
	long channel;
	SignalCountingType cntType;
	//CHAR* profilePath;
}DevConfParam, *PDevConfParam;




// COneDOFDlg �Ի���
class COneDOFDlg : public CDialogEx
{
// ����
public:
	COneDOFDlg(CWnd* pParent = NULL);	// ��׼���캯��

// �Ի�������
	enum { IDD = IDD_ONEDOF_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��


public:

	//��ý�嶨ʱ�����õĴ������Ӿ���Ⱦ����
	void MMTimerHapticRendering(UINT nIDEvent);
	void MMTimerGraphicRendering(UINT nIDEvent);


public:
	//������صı����ͺ���
	CSerialPort m_ComPortForce;	//�������������ֻ࣬����
	CSerialPort m_ComPortCAN;	//CAN��ֻ����
	CSerialPort m_ComPortDAQ;	//DAQ��ֻ����
	afx_msg LRESULT OnComRecvData(WPARAM str, LPARAM commInfo);//���ڽ��ճ���
	BOOL ProcessForceData(DWORD len);//����������������
	DWORD m_ComPortNumForce,	//�����������ں�
	m_ComPortNumDAQ,//DAQ���ں�
	m_ComPortNumCAN;//CAN���ں�

	//CANͨ�����
	UINT m_DeviceType;	//����can�豸���
	//static UINT ReceiveThread(void *param);	//CAN����
	CCANCmd m_CANCmd;//CANָ�����
	CAN_OBJ m_CANCmdFrame[30];//��m_CANCmd��getframe������ȡ��ָ��,���һ�η���10֡����can���ӿڶ���

	//��������
	DOUBLE m_dAmpZero[6];
	DOUBLE m_dChnGain[6];
	DOUBLE m_dChnEx[6];//
	static CONST DOUBLE m_dDecouplingCoefficient[6][6];//�������

	//�˶�ѧ���
	CFiveBarKinematics m_FiveBarKinematics;

	//********************���Ʊ�������Ҫ��*************************
	DOUBLE m_Position_HapticTool[POSITION_DIMENTION];	//ĩ��λ��
	DOUBLE m_Cmd_DPCAngle[NUMBER_JOINTS];				//��������Լ���ؽڽ�
	DOUBLE m_Current_DPCAngle[NUMBER_JOINTS];			//��ǰ����Լ���ؽڽ�
	DOUBLE m_Read_LinkAngle[NUMBER_JOINTS];				//��ȡ�����˽Ƕ�
	DOUBLE m_Cartesian_Force[POSITION_DIMENTION];		//�ѿ�����
	
	DOUBLE m_Read_ForceSensor[6];						//���������Ķ���
	DOUBLE m_Read_ForceSensor0[6];		//��ʼ�ѿ�����

	BOOL m_bOnlyFree;		//TRUEֻ���棬����������
	BOOL m_bInitValue;		//��ʼ������

	//��ʼ����־λ�������жϡ���ʼ��ť����������״̬
	BOOL m_bFlagInitForce;//����������ʼ����־λ
	BOOL m_bFlagInitDAQ;//����������ʼ����־λ
	BOOL m_bFlagInitDrive;//��������ʼ����־λ



	BOOL m_bFlagRunned;//��ʾ���й�������ֹͣ��ť����ʱ����������Դ����������
	BOOL m_bTimerRunning;//��ý�嶨ʱ�����б�־λ
	BOOL m_bFlagCanConnect;	//���ӱ�־λ
	BOOL m_bFlagForceConnect;	//���ӱ�־λ
	BOOL m_bFlagDAQConnect;	//���ӱ�־λ
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

	//Բ������ǽ��������
	double m_circleX,m_circleY, m_circleR; 


// ʵ��
protected:
	HICON m_hIcon;

	// ���ɵ���Ϣӳ�亯��
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
	UdCounterCtrl*     m_udCounterCtrl;//�л����ɿ����ļ�
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
