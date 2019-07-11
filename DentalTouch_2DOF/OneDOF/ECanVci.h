#ifndef _ECANVCI_H_

#define _ECANVCI_H_

#include "stdafx.h"



//#ifndef _WIN64
typedef unsigned char     uint8_t;
typedef signed   char     int8_t;
typedef unsigned short    uint16_t;
typedef signed   short    int16_t;
typedef unsigned int     uint32_t;
typedef signed   int     int32_t;
//#else
//typedef unsigned  int8   uint8_t;
//typedef signed  int8     int8_t;
//typedef unsigned  int16  uint16_t;
//typedef signed  int16    int16_t;
//typedef unsigned  int32  uint32_t;
//typedef signed  int32    int32_t;
//#endif
typedef unsigned long long  uint64_t;
typedef signed long long    int64_t;
typedef UINT            ECAN_RESULT;
typedef HANDLE            ECAN_HANDLE;



typedef struct
{
  int32_t       id;             /* can-id                                   */
  uint8_t       len;            /* length of message: 0-8                   */
  uint8_t       msg_lost;       /* count of lost rx-messages                */
  uint8_t       reserved[2];    /* reserved                                 */
  uint8_t       data[8];        /* 8 data-bytes                             */
} CMSG;


typedef struct
{
  int32_t       evid;          /* event-id: possible range:EV_BASE...EV_LAST */
  uint8_t       len;           /* length of message: 0-8                     */
  uint8_t       reserved[3];   /* reserved                                   */
  union
  {
    uint8_t  c[8];
    uint16_t s[4];
    uint32_t l[2];
  } evdata;
} EVMSG;


typedef struct
{
  int32_t id;                   /* can-id                                    */
  uint8_t len;                  /* length of message: 0-8                    */
  uint8_t msg_lost ;            /* count of lost rx-messages                 */
  uint8_t reserved[2] ;         /* reserved                                  */
  uint8_t data[8] ;             /* 8 data-bytes                              */
  uint64_t timestamp ;          /* time stamp of this message                */
} CMSG_T;




//1.ϵ�нӿڿ���Ϣ���������͡�
typedef  struct  _BOARD_INFO{
		USHORT	hw_Version;
		USHORT	fw_Version;
		USHORT	dr_Version;
		USHORT	in_Version;
		USHORT	irq_Num;
		BYTE	can_Num;
		CHAR	str_Serial_Num[20];
		CHAR	str_hw_Type[40];
		USHORT	Reserved[4];
} BOARD_INFO,*P_BOARD_INFO; 

//2.����CAN��Ϣ֡���������͡�
typedef  struct  _CAN_OBJ{
	UINT	ID;
	UINT	TimeStamp;
	BYTE	TimeFlag;
	BYTE	SendType;
	BYTE	RemoteFlag;//�Ƿ���Զ��֡
	BYTE	ExternFlag;//�Ƿ�����չ֡
	BYTE	DataLen;
	BYTE	Data[8];
	BYTE	Reserved[3];
}CAN_OBJ,*P_CAN_OBJ;

//3.����CAN������״̬���������͡�
typedef struct _CAN_STATUS{
	UCHAR	ErrInterrupt;
	UCHAR	regMode;
	UCHAR	regStatus;
	UCHAR	regALCapture;
	UCHAR	regECCapture; 
	UCHAR	regEWLimit;
	UCHAR	regRECounter; 
	UCHAR	regTECounter;
	UINT	Reserved;
}CAN_STATUS,*P_CAN_STATUS;

//4.���������Ϣ���������͡�
typedef struct _ERR_INFO{
		UINT	ErrCode;
		BYTE	Passive_ErrData[3];
		BYTE	ArLost_ErrData;
} ERR_INFO,*P_ERR_INFO;

//5.�����ʼ��CAN����������
typedef struct _INIT_CONFIG{
	UINT	AccCode;
	UINT	AccMask;
	UINT	Reserved;
	UCHAR	Filter;
	UCHAR	Timing0;	
	UCHAR	Timing1;	
	UCHAR	Mode;
}INIT_CONFIG,*P_INIT_CONFIG;


typedef struct _FILTER_RECORD{
	UINT ExtFrame;	//�Ƿ�Ϊ��չ֡
	UINT Start;
	UINT End;
}FILTER_RECORD,*P_FILTER_RECORD;



//�������÷���״ֵ̬
#define	STATUS_OK					1
#define STATUS_ERR					0
#define ECAN_ERR					0

#define ECAN_SUCCESS                1




//CAN������
#define	ERR_CAN_OVERFLOW			0x0001	//CAN�������ڲ�FIFO���
#define	ERR_CAN_ERRALARM			0x0002	//CAN���������󱨾�
#define	ERR_CAN_PASSIVE				0x0004	//CAN��������������
#define	ERR_CAN_LOSE				0x0008	//CAN�������ٲö�ʧ
#define	ERR_CAN_BUSERR				0x0010	//CAN���������ߴ���
#define	ERR_CAN_REG_FULL			0x0020	//CAN���ռĴ�����
#define	ERR_CAN_REG_OVER			0x0040	//CAN���ռĴ������
#define	ERR_CAN_ZHUDONG	    		0x0080	//CAN��������������

//ͨ�ô�����
#define	ERR_DEVICEOPENED			0x0100	//�豸�Ѿ���
#define	ERR_DEVICEOPEN				0x0200	//���豸����
#define	ERR_DEVICENOTOPEN			0x0400	//�豸û�д� ���Ѿ�usb�γ�
#define	ERR_BUFFEROVERFLOW			0x0800	//���������
#define	ERR_DEVICENOTEXIST			0x1000	//���豸������
#define	ERR_LOADKERNELDLL			0x2000	//װ�ض�̬��ʧ��
#define ERR_CMDFAILED				0x4000	//ִ������ʧ�ܴ�����
#define	ERR_BUFFERCREATE			0x8000	//�ڴ治��

#define ECAN_RX_TIMEOUT                0xE0000001
#define ECAN_TX_TIMEOUT                0xE0000002
#define ECAN_TX_ERROR                  0xE0000004



/*------------------ Defines ------------------------------------------------*/

#define ECAN_EV_BASE                   0x40000000
#define ECAN_EV_USER                   0x40000080
#define ECAN_EV_LAST                   0x400000FF

#define ECAN_EV_CAN_ERROR              ECAN_EV_BASE
#define ECAN_EV_BAUD_CHANGE    (ECAN_EV_BASE + 0x1)

#define ECAN_EV_ADD_ID_ZERO    (ECAN_EV_BASE + 0x2)
#define ECAN_EV_ADD_ID    (ECAN_EV_BASE + 0x3)
#define ECAN_EV_ADD_ID_SA    (ECAN_EV_BASE + 0x4)
#define ECAN_EV_ADD_ID_RANG    (ECAN_EV_BASE + 0x5)
#define ECAN_EV_RESET    (ECAN_EV_BASE + 0x6)
#define ECAN_EV_ADD_MASK    (ECAN_EV_BASE + 0x7)
#define ECAN_EV_Mode   (ECAN_EV_BASE + 0xc)


#define ECAN_IOCTL_GET_TIMESTAMP_FREQ    0x0007   /* Get timestamp frequency in Hz  */
#define ECAN_IOCTL_GET_TIMESTAMP         0x0008   /* Get timestamp counter   */  

#define EXTERNC		extern "C"

//int __stdcall example(int value); 
//UINT __stdcall OpenDevice(UINT DeviceType,UINT DeviceInd,UINT Reserved);
//UINT __stdcall StartCAN(UINT DeviceType,UINT DeviceInd,UINT CANInd);
//UINT __stdcall ReadErrInfo(UINT DeviceType,UINT DeviceInd,UINT CANInd,P_ERR_INFO pErrInfo);

EXTERNC UINT __stdcall OpenDevice(UINT DeviceType,UINT DeviceInd,UINT Reserved);
EXTERNC UINT __stdcall CloseDevice(UINT DeviceType,UINT DeviceInd);
EXTERNC UINT __stdcall InitCAN(UINT DeviceType, UINT DeviceInd, UINT CANInd, P_INIT_CONFIG pInitConfig);
EXTERNC UINT __stdcall ReadBoardInfo(UINT DeviceType,UINT DeviceInd,P_BOARD_INFO pInfo);
EXTERNC UINT __stdcall ReadErrInfo(UINT DeviceType,UINT DeviceInd,UINT CANInd,P_ERR_INFO pErrInfo);
EXTERNC UINT __stdcall ReadCANStatus(UINT DeviceType,UINT DeviceInd,UINT CANInd,P_CAN_STATUS pCANStatus);
EXTERNC UINT __stdcall GetReference(UINT DeviceType,UINT DeviceInd,UINT CANInd,UINT RefType,PVOID pData);
EXTERNC UINT __stdcall SetReference(UINT DeviceType,UINT DeviceInd,UINT CANInd,UINT RefType,PVOID pData);
EXTERNC UINT __stdcall GetReceiveNum(UINT DeviceType,UINT DeviceInd,UINT CANInd);
EXTERNC UINT __stdcall ClearBuffer(UINT DeviceType,UINT DeviceInd,UINT CANInd);
EXTERNC UINT __stdcall StartCAN(UINT DeviceType,UINT DeviceInd,UINT CANInd);
EXTERNC UINT __stdcall ResetCAN(UINT DeviceType,UINT DeviceInd,UINT CANInd);
EXTERNC EXTERNC UINT __stdcall Transmit(UINT DeviceType,UINT DeviceInd,UINT CANInd,P_CAN_OBJ pSend,ULONG Len);
EXTERNC UINT __stdcall Receive(UINT DeviceType,UINT DeviceInd,UINT CANInd,P_CAN_OBJ pReceive,ULONG Len,INT WaitTime);








#endif