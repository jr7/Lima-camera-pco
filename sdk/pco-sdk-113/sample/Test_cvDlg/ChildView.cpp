// ChildView.cpp : Implementierung der Klasse CChildView
//

#include "stdafx.h"
#include "Test_cvDlg.h"
#include "ChildView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CChildView
CChildView::CChildView()
{
  m_iXRes = -1;
  m_iYRes = -1;
  m_handleBitmap = NULL;
}

CChildView::~CChildView()
{
  ::DeleteObject(m_handleBitmap);
}

BEGIN_MESSAGE_MAP(CChildView, CWnd)
  ON_WM_CREATE()
  ON_WM_SIZE()
  ON_WM_PAINT()
  ON_WM_ERASEBKGND()
END_MESSAGE_MAP()

int CChildView::SetBitmap(byte* pdata, int iwidth, int iheight)
{
  BITMAPINFO bmi;
  char* pbitmap;
  HDC hchelper;

  ZeroMemory(&bmi, sizeof(bmi));

  // Fill out the fields you care about.
  bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
  bmi.bmiHeader.biWidth = iwidth;
  bmi.bmiHeader.biHeight = iheight;
  bmi.bmiHeader.biSizeImage = iwidth * iheight * 3;
  bmi.bmiHeader.biPlanes = 1;
  bmi.bmiHeader.biBitCount = 24;
  bmi.bmiHeader.biCompression = BI_RGB;
  hchelper = ::GetDC(NULL);

  m_iXRes = iwidth;
  m_iYRes = iheight;
  // Create the surface.
  ::DeleteObject(m_handleBitmap);
  m_handleBitmap = CreateDIBSection(hchelper, &bmi, DIB_RGB_COLORS,(void **)&pbitmap, NULL, 0);
  memcpy(pbitmap, pdata, m_iXRes * m_iYRes * 3);
  return 0;
}

BOOL CChildView::PreCreateWindow(CREATESTRUCT& cs) 
{
  if (!CWnd::PreCreateWindow(cs))
    return FALSE;

  cs.dwExStyle |= WS_EX_CLIENTEDGE;
  cs.style &= ~WS_BORDER;
  cs.lpszClass = AfxRegisterWndClass(CS_HREDRAW|CS_VREDRAW|CS_DBLCLKS, 
    ::LoadCursor(NULL, IDC_ARROW), reinterpret_cast<HBRUSH>(COLOR_WINDOW+1), NULL);

  return TRUE;
}

void CChildView::OnPaint() 
{
  CPaintDC dc(this);
  CDC memdc;
  HBITMAP* pold;
  CRect rect;

  dc.SetStretchBltMode(HALFTONE);

  GetClientRect(&rect);

  if(m_handleBitmap)
  {
    memdc.CreateCompatibleDC(&dc);
    pold = (HBITMAP*)memdc.SelectObject(m_handleBitmap);
    int err = GetLastError();
    dc.StretchBlt(0,0, rect.Width(), rect.Height(), &memdc, 0, 0, m_iXRes, m_iYRes, SRCCOPY);
    memdc.SelectObject(pold);
    memdc.DeleteDC();
  }
}

BOOL CChildView::OnEraseBkgnd(CDC* pDC)
{
  return TRUE;//CWnd::OnEraseBkgnd(pDC);
}

int CChildView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
  if (CWnd::OnCreate(lpCreateStruct) == -1)
    return -1;
  return 0;
}

void CChildView::OnSize(UINT nType, int cx, int cy)
{
  CWnd::OnSize(nType, cx, cy);
  CRect rect;

  GetClientRect(&rect);
}
