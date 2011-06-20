// ChildView.h : Schnittstelle der Klasse CChildView
//

#pragma once

// CChildView-Fenster

class CChildView : public CWnd
{
  // Konstruktion
public:

  CChildView();

  // Attribute
public:

  // Vorgänge
public:
  int SetBitmap(byte* pdata, int iwidth, int iheight);

  // Überschreibungen
protected:
  virtual BOOL PreCreateWindow(CREATESTRUCT& cs);

  // Implementierung
public:
  virtual ~CChildView();
protected:
  HBITMAP m_handleBitmap;
  int m_iXRes, m_iYRes;
  // Generierte Funktionen für die Meldungstabellen
protected:

  DECLARE_MESSAGE_MAP()
  afx_msg void OnPaint();
public:
  afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
  afx_msg void OnSize(UINT nType, int cx, int cy);
  afx_msg BOOL OnEraseBkgnd(CDC* pDC);
};

