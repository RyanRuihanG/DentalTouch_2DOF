
// OneDOF.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������

#pragma comment(lib,"winmm")  //lib file����ý�嶨ʱ����
#pragma comment(lib,"ECanVci")  //CAN����

// COneDOFApp:
// �йش����ʵ�֣������ OneDOF.cpp
//

class COneDOFApp : public CWinApp
{
public:
	COneDOFApp();







// ��д
public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()





};

extern COneDOFApp theApp;