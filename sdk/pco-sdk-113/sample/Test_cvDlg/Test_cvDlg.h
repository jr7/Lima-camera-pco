// Test_cvDlg.h : Hauptheaderdatei für die Test_cvDlg-Anwendung
//
#pragma once

#ifndef __AFXWIN_H__
#error "\"stdafx.h\" vor dieser Datei für PCH einschließen"
#endif

#include "resource.h"       // Hauptsymbole


// CTest_cvDlgApp:
// Siehe Test_cvDlg.cpp für die Implementierung dieser Klasse
//

class CTest_cvDlgApp : public CWinApp
{
public:
  CTest_cvDlgApp();


  // Überschreibungen
public:
  virtual BOOL InitInstance();
  virtual BOOL ExitInstance();
  int m_iNum;
  // Implementierung

public:
  afx_msg void OnAppAbout();
  DECLARE_MESSAGE_MAP()
};

extern CTest_cvDlgApp theApp;