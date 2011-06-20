// MainFrm.cpp : Implementation of classe CMainFrame
//

#include "stdafx.h"
#include "Test_cvDlg.h"

#include "FILE12.H"
#include "MainFrm.h"
#include "PCO_cDlg.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CMainFrame

IMPLEMENT_DYNAMIC(CMainFrame, CFrameWnd)
BEGIN_MESSAGE_MAP(CMainFrame, CFrameWnd)
  ON_MESSAGE(WM_APP+1011, OnProcessDialog)
  ON_WM_CREATE()
  ON_WM_SETFOCUS()
  ON_COMMAND(ID_OPEN_CONVERTDIALOG, OnOpenConvertDialog)
  ON_WM_CLOSE()
  ON_COMMAND(ID_FILE_OPENFILE, OnFileOpenfile)
  ON_COMMAND(ID_BAYER0, OnBayer0)
  ON_UPDATE_COMMAND_UI(ID_BAYER0, OnUpdateBayer0)
  ON_COMMAND(ID_BAYER1, OnBayer1)
  ON_UPDATE_COMMAND_UI(ID_BAYER1, OnUpdateBayer1)
  ON_COMMAND(ID_BAYER2, OnBayer2)
  ON_UPDATE_COMMAND_UI(ID_BAYER2, OnUpdateBayer2)
  ON_COMMAND(ID_BAYER3, OnBayer3)
  ON_UPDATE_COMMAND_UI(ID_BAYER3, OnUpdateBayer3)
  ON_UPDATE_COMMAND_UI(ID_OPEN_CONVERTDIALOG, &CMainFrame::OnUpdateOpenConvertdialog)
  ON_COMMAND(ID_FPSTEST, OnFPSTest)
  ON_UPDATE_COMMAND_UI(ID_FPSTEST, OnUpdateFPSTest)
END_MESSAGE_MAP()

CMainFrame::CMainFrame()
{
  input_image = NULL;
  output_image = NULL;
  m_hLutDialog = NULL;
  m_hConvertThread = NULL;
  col_lut = NULL;
}

CMainFrame::~CMainFrame()
{
  if(input_image)
    delete(	input_image);
  if(output_image)
    delete(output_image);
}

int CMainFrame::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
  hwnd_frame = GetSafeHwnd();
  //::MessageBox(NULL, "blah", MB_OK, 0);
  if (CFrameWnd::OnCreate(lpCreateStruct) == -1)
    return -1;

  if (!m_wndView.Create(NULL, NULL, AFX_WS_DEFAULT_VIEW,
    CRect(0, 0, 0, 0), this, AFX_IDW_PANE_FIRST, NULL))
  {
    return -1;
  }

  int width; 
  int height;
  bool mode_sp;

  ////load test picture 
  printf("Loading TIF-file test\n");	 

  if((getsize_tif("test.tif", &width, &height, &mode_sp))!=0)
  {
    printf("Error loading file test.tif.\n"); 
    ::AfxMessageBox("Test.tif not found!", MB_ICONERROR, 0);
    exit(-1);
  }

  pic_width  = width; 
  pic_height = height;

  int size = width*height;
  Bild bild;

  // Allocate 16bit buffer
  input_image = (WORD *)malloc(size*sizeof(WORD));
  // Allocate 24bit RGB buffer
  output_image = (byte *)malloc(3*size*sizeof(byte));

  bild.bAlignUpper = TRUE;
  bild.iBitRes = 12;
  bild.pic12 = input_image;            // Set image pointer to Bild struct

  if(read_tif("test.tif", &bild, 0) != 0)
  {
    printf("Error reading tif.\n");
    exit(-1);
  };

  PCO_SensorInfo strsensorinf;
  // This structure holds all the information necessary to setup a convert object.

  memset((void*)&strsensorinf.wSize, 0, sizeof(PCO_SensorInfo));
  strsensorinf.wSize = sizeof(PCO_SensorInfo);// Set size of structure
  strsensorinf.iConversionFactor = 8;  // Set conversion factor of camera
  strsensorinf.iDarkOffset = 32;       // Set dark offset of camera
  strsensorinf.iDataBits = 12;         // Set bit resolution of camera
  // Set color type and alignment of image data.
  strsensorinf.iSensorInfoBits = CONVERT_SENSOR_COLORIMAGE;// Set color camera type; 

  // Set the color correction matrix information of the camera.
  strsensorinf.strColorCoeff.da11 = 1.4474; strsensorinf.strColorCoeff.da12 = -0.5856; strsensorinf.strColorCoeff.da13 = 0.1382;
  strsensorinf.strColorCoeff.da21 = -0.1961; strsensorinf.strColorCoeff.da22 = 1.4444; strsensorinf.strColorCoeff.da23 = -0.2401;
  strsensorinf.strColorCoeff.da31 = 0.1027; strsensorinf.strColorCoeff.da32 = -0.6059; strsensorinf.strColorCoeff.da33 = 1.5350;
  m_hLut = NULL;
  // PCO_BW_CONVERT (1)     -> Creates a bw convert object
  // PCO_COLOR_CONVERT (2)  -> Creates a color convert object
  // PCO_PSEUDO_CONVERT (3) -> Creates a pseudo convert object
  // PCO_COLOR16_CONVERT (4)-> Creates a 16bit color convert object

  // Create a color convert object
  int err = PCO_ConvertCreate(&m_hLut, (PCO_SensorInfo*)&strsensorinf.wSize, PCO_COLOR_CONVERT);
  SetMode(0, TRUE, FALSE);

  PCO_Display strDisplay;
  memset((void*)&strDisplay.wSize, 0, sizeof(PCO_Display));
  strDisplay.wSize = sizeof(PCO_Display);
  // Gets the PCO_Display structure
  err = PCO_ConvertGetDisplay(m_hLut, (PCO_Display*) &strDisplay.wSize);
  strDisplay.iScale_min = 64;          // min value for convert. This makes the test.tif looking good!
  strDisplay.iScale_max = 410;         // max value for convert.  
  // Sets the PCO_Display structure
  err = PCO_ConvertSetDisplay(m_hLut, (PCO_Display*) &strDisplay.wSize);

  // Convert 16bit data
  PCO_Convert16TOCOL(m_hLut, m_iMode, m_iColMode, width, height, input_image, output_image);
  m_wndView.SetBitmap(output_image, pic_width, pic_height);

  return 0;
}

void CMainFrame::SetMode(int bayer, bool bflip, bool bmirror)
{
  int imosaiker[4] = {BAYER_UPPER_LEFT_IS_RED, BAYER_UPPER_LEFT_IS_GREEN_RED,
    BAYER_UPPER_LEFT_IS_GREEN_BLUE, BAYER_UPPER_LEFT_IS_BLUE};
  m_iColMode = imosaiker[bayer];       // In case of a PixelFly or Sensicam this parameter is 0
  // pco.camera series depend on the descriptor and ROI
  m_iMode = 0;
  if(bflip)
    m_iMode |= CONVERT_MODE_OUT_FLIPIMAGE;
  if(bmirror)
    m_iMode |= CONVERT_MODE_OUT_MIRRORIMAGE;
  DoConvert();
}

BOOL CMainFrame::PreCreateWindow(CREATESTRUCT& cs)
{
  if( !CFrameWnd::PreCreateWindow(cs) )
    return FALSE;

  cs.dwExStyle &= ~WS_EX_CLIENTEDGE;
  cs.lpszClass = AfxRegisterWndClass(0);
  return TRUE;
}

#ifdef _DEBUG
void CMainFrame::AssertValid() const
{
  CFrameWnd::AssertValid();
}

void CMainFrame::Dump(CDumpContext& dc) const
{
  CFrameWnd::Dump(dc);
}

#endif //_DEBUG


// CMainFrame Event handler

void CMainFrame::OnSetFocus(CWnd* /*pOldWnd*/)
{
  m_wndView.SetFocus();
}

BOOL CMainFrame::OnCmdMsg(UINT nID, int nCode, void* pExtra, AFX_CMDHANDLERINFO* pHandlerInfo)
{
  if (m_wndView.OnCmdMsg(nID, nCode, pExtra, pHandlerInfo))
    return TRUE;

  return CFrameWnd::OnCmdMsg(nID, nCode, pExtra, pHandlerInfo);
}

void CMainFrame::OnOpenConvertDialog()
{
  if(m_hLutDialog != NULL)             // Already open -> close dialog
  {
    PCO_CloseConvertDialog(m_hLutDialog);
    m_hLutDialog = NULL;
    return;
  }

  int width  = pic_width;
  int height = pic_height;

  m_hLutDialog = NULL;                 // Set dialog handle to NULL
  // Open a new dialog with WM_APP+1011 as message
  // and the previously created convert object for control.
  int err = PCO_OpenConvertDialog(&m_hLutDialog, GetSafeHwnd(), "Convert Dialog", WM_APP+1011, m_hLut, 410, 252);
  DoConvert();
}

void CMainFrame::OnClose()
{
  PCO_ConvertDelete(m_hLut);           // Delete convert object
  if(m_hLutDialog != NULL)             // Close dialog in case one is opened.
  {
    PCO_CloseConvertDialog(m_hLutDialog);
    m_hLutDialog = NULL;
  }
  CFrameWnd::OnClose();
}

LRESULT CMainFrame::OnProcessDialog(WPARAM a, LPARAM b)// All convert actions will run up here
{
  int color_temp;
  int tint;
  PCO_ConvDlg_Message *gl_strMessageInfo;

  gl_strMessageInfo = (PCO_ConvDlg_Message*)b;

  if(gl_strMessageInfo->wCommand == PCO_CNV_DLG_CMD_CLOSING)// Dialog is closing
    m_hLutDialog = NULL;
  if(gl_strMessageInfo->wCommand == PCO_CNV_DLG_CMD_WHITEBALANCE)// White balance button pressed
  {
    // Get color_temp and tint in order to get a white balanced image.
    PCO_GetWhiteBalance(m_hLut, &color_temp, &tint, 0, pic_width, pic_height, input_image, 0, 0, pic_width, pic_height );

    PCO_Display strDisplay;
    strDisplay.wSize = sizeof(PCO_Display);
    PCO_ConvertGetDisplay(m_hLut, (PCO_Display*)&strDisplay.wSize);
    strDisplay.iColor_temp = color_temp;
    strDisplay.iColor_tint = tint;
    PCO_ConvertSetDisplay(m_hLut, (PCO_Display*)&strDisplay.wSize);

    PCO_SetConvertDialog(m_hLutDialog, NULL);// Reload actual data
  }

  DoConvert();
  return 0;
}

void CMainFrame::DoConvert()
{
  PCO_Convert16TOCOL(m_hLut, m_iMode, m_iColMode, pic_width, pic_height, input_image, output_image);
  // Display conversion result as histogram
  PCO_SetDataToDialog(m_hLutDialog, pic_width, pic_height, input_image, output_image);
  m_wndView.SetBitmap(output_image, pic_width ,pic_height);
  m_wndView.Invalidate();
}

void CMainFrame::OnFileOpenfile()
{
  int width, height;
  bool mode_sp;

  CFileDialog fo(TRUE, "tif-file|*.tif", "tif", 0, 0, this, 0, 1);
  if(fo.DoModal() != IDOK)
    return;

  if((getsize_tif((char*)(LPCTSTR)fo.GetPathName(), &width, &height, &mode_sp))!=0)
  {
    printf("error, load picture:\n"); 
    exit(-1);
  }

  pic_width  = width; 
  pic_height = height;

  int size = width*height;
  Bild bild;
  if(input_image != NULL)
    free(input_image);
  if(output_image != NULL)
    free(output_image);
  input_image = (WORD *)malloc(size*sizeof(WORD));
  output_image = (byte *)malloc(3*size*sizeof(byte));

  bild.pic12 = input_image;

  if(read_tif((char*)(LPCTSTR)fo.GetPathName(), &bild, 0) != 0)
  {
    printf("read tif pic error;\n"); 
    exit(-1);
  };
  DoConvert();
}

void CMainFrame::OnBayer0()
{
  SetMode(0, TRUE, FALSE);
}

void CMainFrame::OnUpdateBayer0(CCmdUI *pCmdUI)
{
  if((m_iColMode & 0xFF) == BAYER_UPPER_LEFT_IS_RED)
    pCmdUI->SetCheck(1);
  else
    pCmdUI->SetCheck(0);
}

void CMainFrame::OnBayer1()
{
  SetMode(1, TRUE, FALSE);
}

void CMainFrame::OnUpdateBayer1(CCmdUI *pCmdUI)
{
  if((m_iColMode & 0xFF) == BAYER_UPPER_LEFT_IS_GREEN_RED)
    pCmdUI->SetCheck(1);
  else
    pCmdUI->SetCheck(0);
}

void CMainFrame::OnBayer2()
{
  SetMode(2, TRUE, FALSE);
}

void CMainFrame::OnUpdateBayer2(CCmdUI *pCmdUI)
{
  if((m_iColMode & 0xFF) == BAYER_UPPER_LEFT_IS_GREEN_BLUE)
    pCmdUI->SetCheck(1);
  else
    pCmdUI->SetCheck(0);
}

void CMainFrame::OnBayer3()
{
  SetMode(3, TRUE, FALSE);
}

void CMainFrame::OnUpdateBayer3(CCmdUI *pCmdUI)
{
  if((m_iColMode & 0xFF) == BAYER_UPPER_LEFT_IS_BLUE)
    pCmdUI->SetCheck(1);
  else
    pCmdUI->SetCheck(0);
}

void CMainFrame::OnUpdateOpenConvertdialog(CCmdUI *pCmdUI)
{
  if(m_hLutDialog != NULL)
    pCmdUI->SetCheck(1);
  else
    pCmdUI->SetCheck(0);
}


void CMainFrame::OnFPSTest()
{
  CWaitCursor wait;
  double dt = 0.0;
  CString csh;

  TT();
  for(int i = 0; i < 100; i++)
  {
    PCO_Convert16TOCOL(m_hLut, m_iMode, m_iColMode, pic_width, pic_height, input_image, output_image);
  }
  dt = TT();
  csh.Format("Done 100 conversions in %4.2f sec. %4.2f fps", dt, 100.0/dt);
  AfxMessageBox((LPCTSTR)csh, MB_ICONINFORMATION | MB_OK, 0);
}

void CMainFrame::OnUpdateFPSTest(CCmdUI *pCmdUI)
{
  pCmdUI->SetCheck(0);
}

double CMainFrame::TT()
{
  __int64 T1, T2;
  double dT;
  static double dTTF = 0.0;// Float for frequ.
  static LARGE_INTEGER LITime1, LITime2;// Time buffer

  if(dTTF == 0.0)
  {
    LARGE_INTEGER LIFrequ;

    QueryPerformanceFrequency(&LIFrequ);
    QueryPerformanceCounter(&LITime1);
    dTTF = (double)LIFrequ.QuadPart;
    return 0.0;
  }

  QueryPerformanceCounter(&LITime2);
  T1 = LITime1.QuadPart;
  T2 = LITime2.QuadPart;

  dT = (double)T2 - (double)T1;
  dT /= dTTF;

  LITime1.QuadPart = LITime2.QuadPart;
  return dT;
}
