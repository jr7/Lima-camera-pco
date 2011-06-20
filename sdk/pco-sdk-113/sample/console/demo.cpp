// demo.cpp : Defines the entry point for the console application.
/**********************************************************************
The following program is intended to illustrate 
the calling sequence of SC2 camera SDK and 
to provide some code segment that may be reused to
save development time. 
Some comments may amend the basic description of library
 in SDK header files,  and the manual of "camera control commands". 
**********************************************************************/
// THIS IS ONLY A DEMO TO SHOW BASIC FUNTIONALITY!

#include "stdafx.h"
#include <stdio.h>

#include "..\..\include\sc2_SDKStructures.h"
#include "..\..\include\sc2_defs.h"
#include "..\..\include\PCO_err.h"
#define PCO_ERRT_H_CREATE_OBJECT
#include "..\..\include\PCO_errt.h"
#include "load.h"

#pragma pack (4)
#define FILEVERSION302 302
#define HEADERLEN 128
typedef struct
{
  WORD *pic12;                         // original image pointer
  SYSTEMTIME sTime;                    // shows the exact time stamp of the image  // 20 byte
  int        iTicks;                   // milliseconds gone after start in stime
  int        iXRes;                    // X Resolution
  int        iYRes;                    // Y Resolution                             // 32 byte
  char cText[40];                      // text which should be placed inside the image// 72 byte
  bool       bDouble;                  // shows if pic is Doubleshutter image
  bool       bDummy[3];                // since bool is only one byte, we've to fill up to four bytes// 76 byte
  int        iBWMin;                   // Lut bw min                               // 80 byte
  int        iBWMax;                   // Lut bw max
  int        iBWLut;                   // Lut lin log
  int        iRMin;                    // red min                                  // 92 byte
  int        iRMax;                    // red max
  int        iGMin;                    // green min                                // 100 byte
  int        iGMax;                    // green max
  int        iBMin;                    // blue min
  int        iBMax;                    // blue max                                 // 112 byte
  int        iColLut;                  // Lut lin log color
  int        iColor;                   // image from Color-CCD: 1 otherwise 0      // 120 byte
  int        iVersion;                 // Version of b16 extended info
  int        iBWMin2;                   // Lut bw min
  int        iBWMax2;                   // Lut bw max                              // 132 byte
  int        iBWLut2;                   // Lut lin log
  int        iRMin2;                    // red min                                 // 140 byte
  int        iRMax2;                    // red max
  int        iGMin2;                    // green min
  int        iGMax2;                    // green max                               // 152 byte
  int        iBMin2;                    // blue min
  int        iBMax2;                    // blue max                                // 160 byte
  int        iColLut2;                  // Lut lin log color
  bool       bAlignUpper;               // Align MSB (0-MSB is bit14, 1-MSB is bit 16)
  bool       bDummy2[3];                // since bool is only one byte, we've to fill up to four bytes // 168 byte
  double     dGammaLut;                 // Gamma value b/w
  double     dGammaLutC;                // Gamma value color
  double     dGammaLut2;                // Gamma value b/w 2
  double     dGammaLutC2;               // Gamma value color 2                     // 200 byte
  int        iColorPatternType;         // Demosaicking type for the color pattern
  int        iBitRes;                   // Bit resolution of image                 // 208 byte
  double     dSaturation;               // Color saturation common for both ds images // 216 byte
}Bild;// ACHTUNG: noch 172 Bytes frei, sonst muss headerlen in file12 angepasst werden!
//   Headerlen         = 512
// - alter Header      = 128
// - Bild (ohne WORD*) = 212
// ergibt freie bytes  = 172

typedef struct
{
  char ucPco[4];
  unsigned int uiFileLen;
  unsigned int uiHeaderLen;
  unsigned int uiXRes;
  unsigned int uiYRes;
  unsigned int uiLutSign;
  unsigned int uiColor;
  unsigned int uiBWMin;
  unsigned int uiBWMax;
  unsigned int uiBWLut;
  unsigned int uiRMin;
  unsigned int uiRMax;
  unsigned int uiGMin;
  unsigned int uiGMax;
  unsigned int uiBMin;
  unsigned int uiBMax;
  unsigned int uiColLut;
  unsigned int uiDS;
  unsigned int uiDummy[HEADERLEN-18];
}B16_HEADER;
#pragma pack ()

#define NOFILE	-100 

static double dwExposure;
static short hbin, vbin;
static short roix1, roix2, roiy1, roiy2;
HINSTANCE SC2Lib = NULL;
HANDLE hCamera;
static int err;
PCO_Description caminfo;
PCO_CameraType strCamType;
WORD wStorageMode;
static unsigned short recstate;
static WORD expbase;
static SHORT sBufNr;
static DWORD imgsize;
static DWORD bufsize;
static WORD* wBuf;
static	WORD trigger_result;

WORD wCameraBusyState;
WORD wActSeg;
DWORD dwStatusDll;
DWORD dwStatusDrv;
DWORD dwValidImageCnt;
DWORD dwMaxImageCnt;
WORD wXResAct = 640; // Actual X Resolution
WORD wYResAct = 480; // Actual Y Resolution
WORD wXResMax; // Maximum X Resolution
WORD wYResMax; // Maximum Y 
short ccdtemp, camtemp, powtemp;
HANDLE hEvent = NULL;

int getlib(void);
int store_b16(char *filename, int width, int height, void *buf, Bild *strBild);

#define EBUFS 400


int main(int argc, char* argv[])
{
  char errbuffer[EBUFS];
  Bild pic;
  unsigned int uiBitMax;

  bool bbufferextern = TRUE;
  //; FALSE// Set to TRUE, if you want to allocate the buffer on your own.
  /************************************************************
  load SC2 library; please note SC2_Cam.dll has dependency on 
  sc2_1394.dll, this dll must be present in your dll search path.
  ************************************************************/

  printf("pco.camera demo program.\n");
  printf("\r\n");
  err = getlib();
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nCan not load SC2 library. Error: %d\n", err);
    goto abnormal;
  }
  printf("Libraries loaded.\r\n");
  
  err =OpenCamera(&hCamera, 0);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_OpenCamera error (hex): %lx\n", err);
    goto abnormal;
  }
  printf("Camera opened.\n");
  
  err = GetSizes(hCamera, &wXResAct, &wYResAct, &wXResMax, &wYResMax);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_GetSizes error(hex): %lx\n", err);
    goto abnormal;
  }
  else 
    printf("\nImage Resolution=%dx%d\n", wXResAct, wYResAct);
  /*********************************************************
  wSize of any defined structure must be filled before used 
  in funciton call.
  **********************************************************/
  
  strCamType.wSize = sizeof(strCamType);
  err = GetCameraType(hCamera, &strCamType);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_GetCameraType error (hex): %lx\n", err);
    goto abnormal;
  }
  
  switch (strCamType.wCamType)
  {
    case CAMERATYPE_PCO1200HS:
      printf("PCO.Camera 1200 hs found \n");
      break;
    case CAMERATYPE_PCO1300:
      printf("PCO.Camera 1300 found \n");
      break;
    case CAMERATYPE_PCO1600:
      printf("PCO.Camera 1600 found \n");
      break;
    case CAMERATYPE_PCO2000:
      printf("PCO.Camera 2000 found \n");
      break;
    case CAMERATYPE_PCO4000:
      printf("PCO.Camera 4000 found \n");
      break;
    default:
      printf("PCO.Camera undefined type");
  }
  
  /***********************************************************
  Note the return of ccd temperature must be divided 
  by 10 to get its true value
  *************************************************************/
  err = GetTemperature(hCamera, &ccdtemp, &camtemp, &powtemp);
  printf("\nOperating temperatures: \nCCD: %d C \nElectronics: %d C \nPower: %d C\n", ccdtemp/10, camtemp, powtemp);
  
  caminfo.wSize =sizeof(PCO_Description);
  err = GetCameraDescription(hCamera, &caminfo);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_GetCameraDescription error (hex): %lx\n", err);
    goto abnormal;
  }
  
  if (caminfo.wMaxBinHorzDESC>2)
    hbin = 2;
  else 
    hbin = 1;
  
  if (caminfo.wMaxBinVertDESC>2)
    vbin = 2;
  else 
    vbin = 1;
  
  err = SetBinning(hCamera, hbin, vbin);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_SetBinning error (hex): %lx\n", err);
    goto abnormal;
  }
  else
    printf("\n%dx%d binning set\n", hbin, vbin);
  
  
  /***********************************************************
    The following step is a must. 
    In SC2, bin proceeds ROI in control.  
    ROI "field of definition" is subject to bin settings.
    Please be advised that this is opposite to SensiCam SDK
  *************************************************************/
  
  roix1 = 1;
  // to maximize in x dimension
  if(caminfo.wRoiHorStepsDESC > 0)
  {
    roix2 = caminfo.wMaxHorzResStdDESC/hbin/caminfo.wRoiHorStepsDESC;
    roix2 *= caminfo.wRoiHorStepsDESC;
  }
  else
    roix2 = caminfo.wMaxHorzResStdDESC/hbin;
  roiy1 = 1;
  // to maximize in y dimnesion
  if(caminfo.wRoiVertStepsDESC > 0)
  {
    roiy2 = caminfo.wMaxVertResStdDESC/vbin/caminfo.wRoiVertStepsDESC;
    roiy2 *= caminfo.wRoiVertStepsDESC;
  }
  else
    roiy2 = caminfo.wMaxVertResStdDESC/vbin;
  err = SetROI(hCamera, roix1, roiy1, roix2, roiy2);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_SetROI error (hex): %lx\n", err);
    goto abnormal;
  }
  else 
    printf("\nROI: \n (%d, %d, %d, %d)\n", roix1, roiy1, roix2, roiy2);
  
  // auto trigger
  err = SetTriggerMode(hCamera, 0x0000);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_SetTriggerMode error (hex): %lx\n", err);
    goto abnormal;
  }
  
  // recorder mode auto
  err = SetStorageMode(hCamera, 0);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_SetStorageMode error (hex): %lx\n", err);
    goto abnormal;
  }
  
  // recorder submode ---- ring buffer
  err = SetRecorderSubmode(hCamera, 1);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_SetRecorderSubmode (hex): %lx\n", err);
    goto abnormal;
  }
  
  // all the images taken be stored
  err = SetAcquireMode(hCamera, 0);
  
  // prepare delay exposure time
  dwExposure = 10;
  expbase = 2;
  err = SetDelayExposureTime(hCamera, // Timebase: 0-ns; 1-us; 2-ms  
    0,		// DWORD dwDelay
    (DWORD)dwExposure,
    0,		// WORD wTimeBaseDelay,
    expbase);	// WORD wTimeBaseExposure
  
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_SetDelayExposureTime (hex): %lx\n", err);
    goto abnormal;
  }
  else
    printf("\nDelay:ExposureTime-->0:%d ms\n", (int)dwExposure);
  
  
  // set gain
  if (caminfo.wConvFactDESC[1]>0)
  {
    err = SetConversionFactor(hCamera, caminfo.wConvFactDESC[1]);
    if (err != 0)
    {
      sprintf_s(errbuffer, EBUFS, "\nPCO_SetDelayExposureTime (hex): %lx\n", err);
      goto abnormal;
    }
  }
  
  /***********************************************************
  Cam Ram can be partitioned and set active. 
  by deafult, it is a single piece. An ID is returned
  *************************************************************/
  if((caminfo.dwGeneralCapsDESC1 & GENERALCAPS1_NO_RECORDER) == 0)// camera has got recorder memory
  {
    err = GetActiveRamSegment(hCamera, &wActSeg);
    if (err != 0)
    {
      sprintf_s(errbuffer, EBUFS, "\nPCO_GetActiveRamSegment(hex): %lx\n", err);
      goto abnormal;
    }
  }
  
  /***********************************************************
  ArmCamera validates settings.  
  recorder must be turned off to ArmCamera
  *************************************************************/
  
  err = GetRecordingState(hCamera, &recstate);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_GetRecordingState(hex): %lx\n", err);
    goto abnormal;
  }
  
  if (recstate>0)
  {
    err = SetRecordingState(hCamera, 0x0000);
    if (err != 0)
    {
      sprintf_s(errbuffer, EBUFS, "\nPCO_SetRecordingState(hex): %lx\n", err);
      goto abnormal;
    }
  }
  err = ArmCamera(hCamera);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_ArmCamera(hex): %lx\n", err);
    goto abnormal;
  }
  
  /***********************************************************
  GetSizes gets correct resolutions following ArmCamera.
  buffer is allocated accordingly
  *************************************************************/
  
  err = GetSizes(hCamera, &wXResAct, &wYResAct, &wXResMax, &wYResMax);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_GetSizes error(hex): %lx\n", err);
    goto abnormal;
  }
  else 
    printf("\nImage Resolution=%dx%d\n", wXResAct, wYResAct);
  
  imgsize = wXResAct*wYResAct*2;
  
  bufsize = imgsize;
  
  if (bbufferextern)
  {
    if (bufsize % 0x1000)
    {
      bufsize = imgsize / 0x1000;
      bufsize += 2;
      bufsize *= 0x1000;
    }
    else
      bufsize += 0x1000;
    
    wBuf = (WORD*)malloc(bufsize);
  }
  else
    wBuf = NULL;
  
  sBufNr = -1;                         // -1 produces a new buffer
  
  err = AllocateBuffer(hCamera, &sBufNr, bufsize, &wBuf, &hEvent);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_AllocateBuffer error(hex): %lx\n", err);
    goto abnormal;
  }

  err = CamLinkSetImageParameters(hCamera, wXResAct, wYResAct);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_CamLinkSetImageParameters error(hex): %lx\n", err);
    goto abnormal;
  }
  
  // to turn recorder
  err = SetRecordingState(hCamera, 0x0001);
  err = GetCameraBusyStatus(hCamera, &wCameraBusyState);
  
  if (bbufferextern)
    err = AddBufferEx(hCamera, 0, 0, sBufNr, wXResAct, wYResAct, caminfo.wDynResDESC);
  else
    err = AddBuffer(hCamera, 0, 0, sBufNr);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_AddBuffer error(hex): %lx\n", err);
    goto abnormal;
  }
  
  do
  {
    Sleep(10);
    err = GetBufferStatus(hCamera, sBufNr, &dwStatusDll, &dwStatusDrv);
    if (err != 0)
    {
      sprintf_s(errbuffer, EBUFS, "\nPCO_GetBufferStatus error(hex): %lx\n", err);
      goto abnormal;
    }
    
    if (dwStatusDrv != 0)
    {
      sprintf_s(errbuffer, EBUFS, "\nPCO_GetBufferStatus dwStatusDrv(hex): %lx\n", dwStatusDrv);
      goto abnormal;
    }
  } while (!(dwStatusDll & 0x00008000));
  
  
  err = SetRecordingState(hCamera, 0x0000);
  /***********************************************************
  The image should be in wBuf->. 
  *************************************************************/
  
  if((caminfo.dwGeneralCapsDESC1 & GENERALCAPS1_NO_RECORDER) == 0)// camera has got recorder memory
  {
    err = GetNumberOfImagesInSegment(hCamera, wActSeg, &dwValidImageCnt, &dwMaxImageCnt);

    GetImage(hCamera, wActSeg, 1, 1, sBufNr);
  
    if (err != 0)
    {
      sprintf_s(errbuffer, EBUFS, "\nPCO_GetNumberOfImagesInSegment error(hex): %lx\n", err);
      goto abnormal;
    }
    else 
      printf("\nNumber of valid images: %d \nMaximum possible: %d\n\n", dwValidImageCnt, dwMaxImageCnt);
  }

  memset(&pic.pic12, 0, sizeof(Bild));
  pic.bAlignUpper = 1;
  pic.bDouble = 0;
  sprintf_s(pic.cText, 40, "Demo");
  pic.iBitRes = caminfo.wDynResDESC;
  uiBitMax = (1 << pic.iBitRes) - 1;

  pic.iRMax = uiBitMax;
  pic.iRMax2 = uiBitMax;
  pic.iRMin = 0;
  pic.iRMin2 = 0;
  pic.iGMax = uiBitMax;
  pic.iGMax2 = uiBitMax;
  pic.iGMin = 0;
  pic.iGMin2 = 0;
  pic.iBMax = uiBitMax;
  pic.iBMax2 = uiBitMax;
  pic.iBMin = 0;
  pic.iBMin2 = 0;
  pic.dGammaLut = 1.0;
  pic.dGammaLut2 = 1.0;
  pic.dGammaLutC = 1.0;
  pic.dGammaLutC2 = 1.0;
  pic.dSaturation = 100;
  pic.iColLut = 0;
  pic.iColLut2 = 0;
  pic.iColor = 0;
  pic.iColorPatternType = 0;

  pic.iBWMin = 0;
  pic.iBWMin2 = 0;
  pic.iBWMax = uiBitMax;
  pic.iBWMax2 = uiBitMax;
  pic.iBWLut = 0;
  pic.iBWLut2 = 0;
  pic.iTicks = 0;
  pic.iXRes = wXResAct;
  pic.iYRes = wYResAct;
  pic.pic12 = wBuf;

  GetSystemTime(&pic.sTime);

  err = store_b16("test.b16", wXResAct, wYResAct, wBuf, (Bild*)&pic.pic12);
  if (!err)
    printf("Image stored in file test.b16, you can view it in CamWare.\n");
  else
    printf("Image not saved beacuse of file error. Probably an access rights problem.");
  
  err = FreeBuffer(hCamera, sBufNr);
  if (err != 0)
  {
    sprintf_s(errbuffer, EBUFS, "\nPCO_FreeBuffer error(hex): %lx\n", err);
    goto abnormal;
  }
  
  printf("\r\nNow we produce an error to show the function PCO_GetErrorText.\r\n");
  err = CloseCamera(&hCamera);// This call produces an error!
  if (err != 0)
  {
    PCO_GetErrorText(err, errbuffer, 400);
    
    printf("Here you can see the error explanation:\r\n%s", errbuffer);
    err = CloseCamera(hCamera);// Correct code...
  }
  
  printf("\r\nClosing program without error.");
  free(wBuf);
  return 0;
  
abnormal:   
  printf(errbuffer);
  printf("Refer to PCO_err.h for explanation of some error codes\n");
  if (sBufNr >= 0)
	  	err = FreeBuffer(hCamera, sBufNr);
  
  if (hCamera != NULL)
		  err = CloseCamera(hCamera);
  free(wBuf);
  exit(-1);
  return -1;
}


int getlib(void)
{
  DWORD liberror;
  SC2Lib = LoadLibrary("SC2_Cam");
  if (SC2Lib == NULL)
  {
	   liberror = GetLastError();
     return liberror;
  }
  
  if ((GetGeneral = (int(__stdcall *)(HANDLE ph, PCO_General *strGeneral))
    GetProcAddress(SC2Lib, "PCO_GetGeneral")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetCameraType = (int(__stdcall *)(HANDLE ph, PCO_CameraType *strCamType))
    GetProcAddress(SC2Lib, "PCO_GetCameraType")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetCameraHealthStatus = (int(__stdcall *)(HANDLE ph, DWORD* dwWarn, DWORD* dwErr, DWORD* dwStatus))
    GetProcAddress(SC2Lib, "PCO_GetCameraHealthStatus")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((ResetSettingsToDefault = (int(__stdcall *)(HANDLE ph))
    GetProcAddress(SC2Lib, "PCO_ResetSettingsToDefault")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((InitiateSelftestProcedure = (int(__stdcall *)(HANDLE ph))
    GetProcAddress(SC2Lib, "PCO_InitiateSelftestProcedure")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetTemperature = (int(__stdcall *)(HANDLE ph, SHORT* sCCDTemp, SHORT* sCamTemp, SHORT* sPowTemp))
    GetProcAddress(SC2Lib, "PCO_GetTemperature")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetSensorStruct = (int(__stdcall *)(HANDLE ph, PCO_Sensor *strSensor))
    GetProcAddress(SC2Lib, "PCO_GetSensorStruct")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetSensorStruct = (int(__stdcall *)(HANDLE ph, PCO_Sensor *strSensor))
    GetProcAddress(SC2Lib, "PCO_SetSensorStruct")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetCameraDescription = (int(__stdcall *)(HANDLE ph, PCO_Description *strDescription))
    GetProcAddress(SC2Lib, "PCO_GetCameraDescription")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetSensorFormat = (int(__stdcall *)(HANDLE ph, WORD* wSensor))
    GetProcAddress(SC2Lib, "PCO_GetSensorFormat")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetSensorFormat = (int(__stdcall *)(HANDLE ph, WORD wSensor))
    GetProcAddress(SC2Lib, "PCO_SetSensorFormat")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetSizes = (int(__stdcall *)(HANDLE ph, WORD *wXResAct, WORD *wYResAct, WORD *wXResMax, WORD *wYResMax))
    GetProcAddress(SC2Lib, "PCO_GetSizes")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetROI = (int(__stdcall *)(HANDLE ph, WORD *wRoiX0, WORD *wRoiY0, WORD *wRoiX1, WORD *wRoiY1))
    GetProcAddress(SC2Lib, "PCO_GetROI")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetROI = (int(__stdcall *)(HANDLE ph, WORD wRoiX0, WORD wRoiY0, WORD wRoiX1, WORD wRoiY1))
    GetProcAddress(SC2Lib, "PCO_SetROI")) == NULL)
    return SC2_ERROR_SDKDLL; 
  if ((GetBinning = (int(__stdcall *)(HANDLE ph, WORD *wBinHorz, WORD *wBinVert))
    GetProcAddress(SC2Lib, "PCO_GetBinning")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetBinning = (int(__stdcall *)(HANDLE ph, WORD wBinHorz, WORD wBinVert))
    GetProcAddress(SC2Lib, "PCO_SetBinning")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetPixelRate = (int(__stdcall *)(HANDLE ph, DWORD *dwPixelRate))
    GetProcAddress(SC2Lib, "PCO_GetPixelRate")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetPixelRate = (int(__stdcall *)(HANDLE ph, DWORD dwPixelRate))
    GetProcAddress(SC2Lib, "PCO_SetPixelRate")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetConversionFactor = (int(__stdcall *)(HANDLE ph, WORD *wConvFact))
    GetProcAddress(SC2Lib, "PCO_GetConversionFactor")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetConversionFactor = (int(__stdcall *)(HANDLE ph, WORD wConvFact))
    GetProcAddress(SC2Lib, "PCO_SetConversionFactor")) == NULL)
    return SC2_ERROR_SDKDLL; 
  if ((GetDoubleImageMode = (int(__stdcall *)(HANDLE ph, WORD *wDoubleImage))
    GetProcAddress(SC2Lib, "PCO_GetDoubleImageMode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetDoubleImageMode = (int(__stdcall *)(HANDLE ph, WORD wDoubleImage))
    GetProcAddress(SC2Lib, "PCO_SetDoubleImageMode")) == NULL)
    return SC2_ERROR_SDKDLL; 
  if ((GetADCOperation = (int(__stdcall *)(HANDLE ph, WORD *wADCOperation))
    GetProcAddress(SC2Lib, "PCO_GetADCOperation")) == NULL)
    return SC2_ERROR_SDKDLL; 
  if ((SetADCOperation = (int(__stdcall *)(HANDLE ph, WORD wADCOperation))
    GetProcAddress(SC2Lib, "PCO_SetADCOperation")) == NULL)
    return SC2_ERROR_SDKDLL; 
  if ((GetIRSensitivity = (int(__stdcall *)(HANDLE ph, WORD *wIR))
    GetProcAddress(SC2Lib, "PCO_GetIRSensitivity")) == NULL)
    return SC2_ERROR_SDKDLL; 
  if ((SetIRSensitivity = (int(__stdcall *)(HANDLE ph, WORD wIR))
    GetProcAddress(SC2Lib, "PCO_SetIRSensitivity")) == NULL)
    return SC2_ERROR_SDKDLL; 
  if ((GetCoolingSetpointTemperature = (int(__stdcall *)(HANDLE ph, SHORT *sCoolSet))
    GetProcAddress(SC2Lib, "PCO_GetCoolingSetpointTemperature")) == NULL)
    return SC2_ERROR_SDKDLL; 
  if ((SetCoolingSetpointTemperature = (int(__stdcall *)(HANDLE ph, SHORT sCoolSet))
    GetProcAddress(SC2Lib, "PCO_SetCoolingSetpointTemperature")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetOffsetMode = (int(__stdcall *)(HANDLE ph, WORD *wOffsetRegulation))
    GetProcAddress(SC2Lib, "PCO_GetOffsetMode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetOffsetMode = (int(__stdcall *)(HANDLE ph, WORD wOffsetRegulation))
    GetProcAddress(SC2Lib, "PCO_SetOffsetMode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetTimingStruct = (int(__stdcall *)(HANDLE ph, PCO_Timing *strTiming))
    GetProcAddress(SC2Lib, "PCO_GetTimingStruct")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetTimingStruct = (int(__stdcall *)(HANDLE ph, PCO_Timing *strTiming))
    GetProcAddress(SC2Lib, "PCO_SetTimingStruct")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetDelayExposureTime = (int(__stdcall *)(HANDLE ph, DWORD* dwDelay, DWORD* dwExposure, WORD* wTimeBaseDelay, WORD* wTimeBaseExposure))
    GetProcAddress(SC2Lib, "PCO_GetDelayExposureTime")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetDelayExposureTime = (int(__stdcall *)(HANDLE ph, DWORD dwDelay, DWORD dwExposure, WORD wTimeBaseDelay, WORD wTimeBaseExposure))
    GetProcAddress(SC2Lib, "PCO_SetDelayExposureTime")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetDelayExposureTimeTable = (int(__stdcall *)(HANDLE ph, DWORD* dwDelay, DWORD* dwExposure, WORD* wTimeBaseDelay, WORD* wTimeBaseExposure, WORD wCount))
    GetProcAddress(SC2Lib, "PCO_GetDelayExposureTimeTable")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetDelayExposureTimeTable = (int(__stdcall *)(HANDLE ph, DWORD* dwDelay, DWORD* dwExposure, WORD wTimeBaseDelay, WORD wTimeBaseExposure, WORD wCount))
    GetProcAddress(SC2Lib, "PCO_SetDelayExposureTimeTable")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetTriggerMode = (int(__stdcall *)(HANDLE ph, WORD* wTriggerMode))
    GetProcAddress(SC2Lib, "PCO_GetTriggerMode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetTriggerMode = (int(__stdcall *)(HANDLE ph, WORD wTriggerMode))
    GetProcAddress(SC2Lib, "PCO_SetTriggerMode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((ForceTrigger = (int(__stdcall *)(HANDLE ph, WORD *wTriggered))
    GetProcAddress(SC2Lib, "PCO_ForceTrigger")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetCameraBusyStatus = (int(__stdcall *)(HANDLE ph, WORD* wCameraBusyState))
    GetProcAddress(SC2Lib, "PCO_GetCameraBusyStatus")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetPowerDownMode = (int(__stdcall *)(HANDLE ph, WORD* wPowerDownMode))
    GetProcAddress(SC2Lib, "PCO_GetPowerDownMode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetPowerDownMode = (int(__stdcall *)(HANDLE ph, WORD wPowerDownMode))
    GetProcAddress(SC2Lib, "PCO_SetPowerDownMode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetUserPowerDownTime = (int(__stdcall *)(HANDLE ph, DWORD* dwPowerDownTime))
    GetProcAddress(SC2Lib, "PCO_GetUserPowerDownTime")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetUserPowerDownTime = (int(__stdcall *)(HANDLE ph, DWORD dwPowerDownTime))
    GetProcAddress(SC2Lib, "PCO_SetUserPowerDownTime")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetExpTrigSignalStatus = (int(__stdcall *)(HANDLE ph, WORD* wExpTrgSignal))
    GetProcAddress(SC2Lib, "PCO_GetExpTrigSignalStatus")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetCOCRuntime = (int(__stdcall *)(HANDLE ph, DWORD* dwTime_s, DWORD* dwTime_us))
    GetProcAddress(SC2Lib, "PCO_GetCOCRuntime")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetStorageStruct = (int(__stdcall *)(HANDLE ph, PCO_Storage *strStorage))
    GetProcAddress(SC2Lib, "PCO_GetStorageStruct")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetStorageStruct = (int(__stdcall *)(HANDLE ph, PCO_Storage *strStorage))
    GetProcAddress(SC2Lib, "PCO_SetStorageStruct")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetCameraRamSize = (int(__stdcall *)(HANDLE ph, DWORD* dwRamSize, WORD* wPageSize))
    GetProcAddress(SC2Lib, "PCO_GetCameraRamSize")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetCameraRamSegmentSize = (int(__stdcall *)(HANDLE ph, DWORD* dwRamSegSize))
    GetProcAddress(SC2Lib, "PCO_GetCameraRamSegmentSize")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetCameraRamSegmentSize = (int(__stdcall *)(HANDLE ph, DWORD* dwRamSegSize))
    GetProcAddress(SC2Lib, "PCO_SetCameraRamSegmentSize")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((ClearRamSegment = (int(__stdcall *)(HANDLE ph))
    GetProcAddress(SC2Lib, "PCO_ClearRamSegment")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetActiveRamSegment = (int(__stdcall *)(HANDLE ph, WORD* wActSeg))
    GetProcAddress(SC2Lib, "PCO_GetActiveRamSegment")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetActiveRamSegment = (int(__stdcall *)(HANDLE ph, WORD wActSeg))
    GetProcAddress(SC2Lib, "PCO_SetActiveRamSegment")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetRecordingStruct = (int(__stdcall *)(HANDLE ph, PCO_Recording *strRecording))
    GetProcAddress(SC2Lib, "PCO_GetRecordingStruct")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetRecordingStruct = (int(__stdcall *)(HANDLE ph, PCO_Recording *strRecording))
    GetProcAddress(SC2Lib, "PCO_SetRecordingStruct")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetStorageMode = (int(__stdcall *)(HANDLE ph, WORD* wStorageMode))
    GetProcAddress(SC2Lib, "PCO_GetStorageMode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetStorageMode = (int(__stdcall *)(HANDLE ph, WORD wStorageMode))
    GetProcAddress(SC2Lib, "PCO_SetStorageMode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetRecorderSubmode = (int(__stdcall *)(HANDLE ph, WORD* wRecSubmode))
    GetProcAddress(SC2Lib, "PCO_GetRecorderSubmode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetRecorderSubmode = (int(__stdcall *)(HANDLE ph, WORD wRecSubmode))
    GetProcAddress(SC2Lib, "PCO_SetRecorderSubmode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetRecordingState = (int(__stdcall *)(HANDLE ph, WORD* wRecState))
    GetProcAddress(SC2Lib, "PCO_GetRecordingState")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetRecordingState = (int(__stdcall *)(HANDLE ph, WORD wRecState))
    GetProcAddress(SC2Lib, "PCO_SetRecordingState")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((ArmCamera = (int(__stdcall *)(HANDLE ph))
    GetProcAddress(SC2Lib, "PCO_ArmCamera")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetAcquireMode = (int(__stdcall *)(HANDLE ph, WORD* wAcquMode))
    GetProcAddress(SC2Lib, "PCO_GetAcquireMode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetAcquireMode = (int(__stdcall *)(HANDLE ph, WORD wAcquMode))
    GetProcAddress(SC2Lib, "PCO_SetAcquireMode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetAcqEnblSignalStatus = (int(__stdcall *)(HANDLE ph, WORD* wAcquEnableState))
    GetProcAddress(SC2Lib, "PCO_GetAcqEnblSignalStatus")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetDateTime = (int(__stdcall *)(HANDLE ph, BYTE ucDay, BYTE ucMonth, WORD wYear, WORD wHour, BYTE ucMin, BYTE ucSec))
    GetProcAddress(SC2Lib, "PCO_SetDateTime")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetTimestampMode = (int(__stdcall *)(HANDLE ph, WORD* wTimeStampMode))
    GetProcAddress(SC2Lib, "PCO_GetTimestampMode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((SetTimestampMode = (int(__stdcall *)(HANDLE ph, WORD wTimeStampMode))
    GetProcAddress(SC2Lib, "PCO_SetTimestampMode")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetImageStruct = (int(__stdcall *)(HANDLE ph, PCO_Image *strImage))
    GetProcAddress(SC2Lib, "PCO_GetImageStruct")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetSegmentStruct = (int(__stdcall *)(HANDLE ph, WORD wSegment, PCO_Segment *strSegment))
    GetProcAddress(SC2Lib, "PCO_GetSegmentStruct")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetSegmentImageSettings = (int(__stdcall *)(HANDLE ph, WORD wSegment, WORD* wXRes, WORD* wYRes, WORD* wBinHorz, WORD* wBinVert, WORD* wRoiX0, WORD* wRoiY0, WORD* wRoiX1, WORD* wRoiY1))
    GetProcAddress(SC2Lib, "PCO_GetSegmentImageSettings")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetNumberOfImagesInSegment = (int(__stdcall *)(HANDLE ph, WORD wSegment, DWORD* dwValidImageCnt, DWORD* dwMaxImageCnt))
    GetProcAddress(SC2Lib, "PCO_GetNumberOfImagesInSegment")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((OpenCamera = (int(__stdcall *)(HANDLE *ph, WORD wCamNum))
    GetProcAddress(SC2Lib, "PCO_OpenCamera")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((CloseCamera = (int(__stdcall *)(HANDLE ph))
    GetProcAddress(SC2Lib, "PCO_CloseCamera")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((AllocateBuffer = (int(__stdcall *)(HANDLE ph, SHORT* sBufNr, DWORD size, WORD** wBuf, HANDLE *hEvent))
    GetProcAddress(SC2Lib, "PCO_AllocateBuffer")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((FreeBuffer = (int(__stdcall *)(HANDLE ph, SHORT sBufNr))
    GetProcAddress(SC2Lib, "PCO_FreeBuffer")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((AddBuffer = (int(__stdcall *)(HANDLE ph, DWORD dw1stImage, DWORD dwLastImage, SHORT sBufNr))
    GetProcAddress(SC2Lib, "PCO_AddBuffer")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((AddBufferEx = (int(__stdcall *)(HANDLE ph, DWORD dw1stImage, DWORD dwLastImage, SHORT sBufNr, WORD wXRes, WORD wYRes, WORD wBitRes))
    GetProcAddress(SC2Lib, "PCO_AddBufferEx")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetBufferStatus = (int(__stdcall *)(HANDLE ph, SHORT sBufNr, DWORD *dwStatusDll, DWORD *dwStatusDrv))
    GetProcAddress(SC2Lib, "PCO_GetBufferStatus")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((RemoveBuffer = (int(__stdcall *)(HANDLE ph))
    GetProcAddress(SC2Lib, "PCO_RemoveBuffer")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetImage = (int(__stdcall *)(HANDLE ph, WORD dwSegment, DWORD dw1stImage, DWORD dwLastImage, SHORT sBufNr))
    GetProcAddress(SC2Lib, "PCO_GetImage")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((GetPendingBuffer = (int(__stdcall *)(HANDLE ph, int *count))
    GetProcAddress(SC2Lib, "PCO_GetPendingBuffer")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((CancelImages = (int(__stdcall *)(HANDLE ph))
    GetProcAddress(SC2Lib, "PCO_CancelImages")) == NULL)
    return SC2_ERROR_SDKDLL;
  if ((CheckDeviceAvailability = (int(__stdcall *)(HANDLE ph, WORD wNum))
    GetProcAddress(SC2Lib, "PCO_CheckDeviceAvailability")) == NULL)
    return SC2_ERROR_SDKDLL;
  if((CamLinkSetImageParameters = (int (__stdcall *)(HANDLE ph, WORD wXResAct, WORD wYResAct))
    GetProcAddress(SC2Lib, "PCO_CamLinkSetImageParameters")) == NULL)
    return SC2_ERROR_SDKDLL;

  return FALSE;
}


int store_b16(char *filename, int width, int height, void *buf, Bild *strBild)
{
  unsigned char *cptr;
//  unsigned char *c1;
  B16_HEADER *pb16;
//  int *b1;
  int e;
  unsigned char *pi;  
  HANDLE hfstore;
  unsigned long z, zz;
  DWORD headerl;
  //  char of[20];
  
  cptr = (unsigned char *)malloc(2000);
  memset(cptr, 0, 2000);
  headerl = 512;
  
  pb16 = (B16_HEADER*) cptr;
  pb16->ucPco[0] = 'P';
  pb16->ucPco[1] = 'C';
  pb16->ucPco[2] = 'O';
  pb16->ucPco[3] = '-';
  pb16->uiFileLen = (width*height*2) + headerl;
  pb16->uiHeaderLen = headerl;
  pb16->uiXRes = width;
  pb16->uiYRes = height;
  pb16->uiLutSign = 0xFFFFFFFF;
  pb16->uiColor = strBild->iColor;
  pb16->uiBMin = strBild->iBWMin;
  pb16->uiBWMax = strBild->iBWMax;
  pb16->uiBWLut = strBild->iBWLut;
  pb16->uiRMin = strBild->iRMin;
  pb16->uiRMax = strBild->iRMax;
  pb16->uiGMin = strBild->iGMin;
  pb16->uiGMax = strBild->iGMax;
  pb16->uiBMin = strBild->iBMin;
  pb16->uiBMax = strBild->iBMax;
  pb16->uiColLut = strBild->iColLut;
  pb16->uiDS;
  if (strBild->bDouble) // bei Doubleshutter Bild, Kennung ablegen
    pb16->uiDS = 0x5344;// DS (SD) -> Kennung für 'DoubleShutter'
  else
  {                     // kein Doubleshutter: Lut-Werte kopieren in 2. Wertesatz
    strBild->iBWMin2 = strBild->iBWMin;                   // Lut bw min
    strBild->iBWMax2 = strBild->iBWMax;                   // Lut bw max
    strBild->iBWLut2 = strBild->iBWLut;                   // Lut lin log
    strBild->iRMin2 = strBild->iRMin;                    // red min
    strBild->iRMax2 = strBild->iRMax;                    // red max
    strBild->iGMin2 = strBild->iGMin;                    // green min
    strBild->iGMax2 = strBild->iGMax;                    // green max
    strBild->iBMin2 = strBild->iBMin;                    // blue min
    strBild->iBMax2 = strBild->iBMax;                    // blue max
    strBild->iColLut2 = strBild->iColLut;                  // Lut lin log color
  }
  strBild->iVersion = FILEVERSION302;
 
  pi = cptr + 128;
  memset(pi, 0, 384);// Strukturdaten auf 0
  memcpy(pi, &strBild->sTime, sizeof(Bild) - sizeof(WORD*));// Struktur ablegen

  hfstore = CreateFile(filename,
    GENERIC_WRITE,
    0,
    NULL,
    CREATE_ALWAYS, 
    FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN,
    0);
  if (hfstore== INVALID_HANDLE_VALUE)
  {
    free(cptr);
    return (NOFILE);
  }
  
  
  z = headerl;
  e = WriteFile(hfstore, (void *)cptr, z, &zz, NULL);
  if ((e == 0) ||(z != zz))
  {
    CloseHandle(hfstore);
    DeleteFile(filename);
    free(cptr);
    return (NOFILE);
  }
  
  z = width*height*2;
  e = WriteFile(hfstore, (void *)buf, z, &zz, NULL);
  if ((e == 0) ||(z != zz))
  {
    CloseHandle(hfstore);
    DeleteFile(filename);
    free(cptr);
    return (NOFILE);
  }
  
  CloseHandle(hfstore);
  free(cptr);
  return 0;
}