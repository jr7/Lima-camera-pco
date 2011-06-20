// MainFrm.h : Schnittstelle der Klasse CMainFrame
//

#pragma once

#include "ChildView.h"

class CMainFrame : public CFrameWnd
{
public:
  HWND hwnd_frame;
  byte *output_image;
  WORD *input_image;
  int pic_width;
  int pic_height;
  int m_iMode;
  int m_iColMode;
  PCO_Convert *col_lut;

  HANDLE m_hLut;
  HANDLE m_hLutDialog;
  HANDLE m_hLutDialog16;

public:
  CMainFrame();
protected: 
  DECLARE_DYNAMIC(CMainFrame)

  // Attribute
public:

  // Vorgänge
public:

  // Überschreibungen
public:
  virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
  virtual BOOL OnCmdMsg(UINT nID, int nCode, void* pExtra, AFX_CMDHANDLERINFO* pHandlerInfo);
  void SetMode(int bayer, bool bflip, bool bmirror);
  void DoConvert();
  double TT();
public:
  virtual ~CMainFrame();
#ifdef _DEBUG
  virtual void AssertValid() const;
  virtual void Dump(CDumpContext& dc) const;
#endif

protected:  // Eingebettete Member der Steuerleiste
  HANDLE      m_hConvertThread;

public:
  CChildView    m_wndView;


  // Generierte Funktionen für die Meldungstabellen
protected:
  afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
  afx_msg void OnSetFocus(CWnd *pOldWnd);
  DECLARE_MESSAGE_MAP()
public:
  afx_msg void OnOpenConvertDialog();
  afx_msg void OnClose();
  afx_msg LRESULT OnProcessDialog(WPARAM a, LPARAM b);
  afx_msg void OnFileOpenfile();
  afx_msg void OnBayer0();
  afx_msg void OnUpdateBayer0(CCmdUI *pCmdUI);
  afx_msg void OnBayer1();
  afx_msg void OnUpdateBayer1(CCmdUI *pCmdUI);
  afx_msg void OnBayer2();
  afx_msg void OnUpdateBayer2(CCmdUI *pCmdUI);
  afx_msg void OnBayer3();
  afx_msg void OnUpdateBayer3(CCmdUI *pCmdUI);
  afx_msg void OnUpdateOpenConvertdialog(CCmdUI *pCmdUI);
  afx_msg void OnFPSTest();
  afx_msg void OnUpdateFPSTest(CCmdUI *pCmdUI);
};


