// Test_cvDlg.cpp : Definiert das Klassenverhalten für die Anwendung.
//

#include "stdafx.h"
#include "Test_cvDlg.h"
#include "MainFrm.h"

#if defined _WIN64
#pragma comment(lib, "pco_cryptdll64.lib") 
#else
#pragma comment(lib, "pco_cryptdll.lib") 
#endif
extern "C" int PCO_SetAppName(const unsigned char szname[]);
extern "C" int PCO_RemoveAppName(int inum);


#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CTest_cvDlgApp

BEGIN_MESSAGE_MAP(CTest_cvDlgApp, CWinApp)
  ON_COMMAND(ID_APP_ABOUT, &CTest_cvDlgApp::OnAppAbout)
END_MESSAGE_MAP()


// CTest_cvDlgApp-Erstellung

CTest_cvDlgApp::CTest_cvDlgApp()
{
}


// Das einzige CTest_cvDlgApp-Objekt

CTest_cvDlgApp theApp;

int CTest_cvDlgApp::ExitInstance()
{
  // TODO: Add your specialized code here and/or call the base class
  PCO_RemoveAppName(m_iNum);
  return CWinApp::ExitInstance();
}
// CTest_cvDlgApp-Initialisierung

BOOL CTest_cvDlgApp::InitInstance()
{
  unsigned char ucappname[20];
  sprintf_s((char*)&ucappname[0], 20, "pcotest_cvdlg");
  m_iNum = PCO_SetAppName(ucappname);

  // InitCommonControlsEx() ist für Windows XP erforderlich, wenn ein Anwendungsmanifest
  // die Verwendung von ComCtl32.dll Version 6 oder höher zum Aktivieren
  // von visuellen Stilen angibt. Ansonsten treten beim Erstellen von Fenstern Fehler auf.
  INITCOMMONCONTROLSEX InitCtrls;
  InitCtrls.dwSize = sizeof(InitCtrls);
  // Legen Sie dies fest, um alle allgemeinen Steuerelementklassen einzubeziehen,
  // die Sie in Ihrer Anwendung verwenden möchten.
  InitCtrls.dwICC = ICC_WIN95_CLASSES;
  InitCommonControlsEx(&InitCtrls);

  CWinApp::InitInstance();

  // OLE-Bibliotheken initialisieren
  if (!AfxOleInit())
  {
    AfxMessageBox(IDP_OLE_INIT_FAILED);
    return FALSE;
  }
  AfxEnableControlContainer();
  // Standardinitialisierung
  // Wenn Sie diese Features nicht verwenden und die Größe
  // der ausführbaren Datei verringern möchten, entfernen Sie
  // die nicht erforderlichen Initialisierungsroutinen.
  // Ändern Sie den Registrierungsschlüssel, unter dem Ihre Einstellungen gespeichert sind.
  // TODO: Ändern Sie diese Zeichenfolge entsprechend,
  // z.B. zum Namen Ihrer Firma oder Organisation.
  SetRegistryKey(_T("Vom lokalen Anwendungs-Assistenten generierte Anwendungen"));
  // Dieser Code erstellt ein neues Rahmenfensterobjekt und legt dieses
  // als Hauptfensterobjekt der Anwendung fest, um das Hauptfenster zu erstellen.
  CMainFrame* pFrame = new CMainFrame;
  if (!pFrame)
    return FALSE;
  m_pMainWnd = pFrame;
  // Rahmen mit Ressourcen erstellen und laden
  pFrame->LoadFrame(IDR_MAINFRAME,
    WS_OVERLAPPEDWINDOW | FWS_ADDTOTITLE, NULL,
    NULL);

  // Das einzige Fenster ist initialisiert und kann jetzt angezeigt und aktualisiert werden.
  pFrame->ShowWindow(SW_SHOW);
  pFrame->UpdateWindow();
  // Rufen Sie DragAcceptFiles nur auf, wenn eine Suffix vorhanden ist.
  //  In einer SDI-Anwendung ist dies nach ProcessShellCommand erforderlich
  return TRUE;
}


// CTest_cvDlgApp-Meldungshandler




// CAboutDlg-Dialogfeld für Anwendungsbefehl "Info"

class CAboutDlg : public CDialog
{
public:
  CAboutDlg();

  // Dialogfelddaten
  enum { IDD = IDD_ABOUTBOX };

protected:
  virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV-Unterstützung

  // Implementierung
protected:
  DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
  CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()

// Anwendungsbefehl zum Ausführen des Dialogfelds
void CTest_cvDlgApp::OnAppAbout()
{
  CAboutDlg aboutDlg;
  aboutDlg.DoModal();
}


// CTest_cvDlgApp-Meldungshandler

