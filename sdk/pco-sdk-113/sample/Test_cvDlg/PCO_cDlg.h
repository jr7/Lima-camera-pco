// PCO_cDlg.h : main header file for the PCO_cDlg DLL
//

#if !defined(AFX_PCO_cDlg_H__F3E0ABC5_62D5_11D4_AD21_0050BAC6FF04__INCLUDED_)
#define AFX_PCO_cDlg_H__F3E0ABC5_62D5_11D4_AD21_0050BAC6FF04__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef __AFXWIN_H__
#error include 'stdafx.h' before including this file for PCH
#endif

//#include "res_senlutdia.h"		// main symbols

/////////////////////////////////////////////////////////////////////////////
// CPCO_cDlgApp
// See PCO_cDlg.cpp for the implementation of this class
//
typedef struct
{
  int iID;
  int ix;
  int iy;
  int ixs;
  int iys;
}posstruct;


class CPCO_cDlgApp : public CWinApp
{

private: 



public:
  CPCO_cDlgApp();

  // Overrides
  // ClassWizard generated virtual function overrides
  //{{AFX_VIRTUAL(CPCO_cDlgApp)
public:
  //	virtual int ExitInstance();
  virtual BOOL InitInstance();
  //}}AFX_VIRTUAL

  //{{AFX_MSG(CPCO_cDlgApp)
  // NOTE - the ClassWizard will add and remove member functions here.
  //    DO NOT EDIT what you see in these blocks of generated code !
  //}}AFX_MSG
  DECLARE_MESSAGE_MAP()
  virtual int ExitInstance();
};


/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_PCO_cDlg_H__F3E0ABC5_62D5_11D4_AD21_0050BAC6FF04__INCLUDED_)
