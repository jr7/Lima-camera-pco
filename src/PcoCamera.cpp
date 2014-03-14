/**************************************************************************
###########################################################################
 This file is part of LImA, a Library for Image Acquisition

 Copyright (C) : 2009-2011
 European Synchrotron Radiation Facility
 BP 220, Grenoble 38043
 FRANCE

 This is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3 of the License, or
 (at your option) any later version.

 This software is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, see <http://www.gnu.org/licenses/>.
###########################################################################
**************************************************************************/

#define PCO_ERRT_H_CREATE_OBJECT
#define BYPASS

#define TOUT_MIN_DIMAX 500
#define ERROR_MSG_LINE 128

//#define BUFF_INFO_SIZE 5000


#include <cstdlib>
#include <process.h>
#include <sys/stat.h>
#include <sys/timeb.h>
#include <time.h>

#include "Exceptions.h"

#include "PcoCamera.h"
#include "PcoSyncCtrlObj.h"
#include "PcoBufferCtrlObj.h"
#include "PcoCameraUtils.h"
#include "PcoGitVersion.h"

using namespace lima;
using namespace lima::Pco;

static char *timebaseUnits[] = {"ns", "us", "ms"};

char *_checkLogFiles();
void _pco_acq_thread_dimax(void *argin);
void _pco_acq_thread_dimax_live(void *argin);
void _pco_acq_thread_edge(void *argin);
void _pco_shutter_thread_edge(void *argin);
void _pco_time2dwbase(double exp_time, DWORD &dwExp, WORD &wBase);

char * _timestamp_pcosyncctrlobj();
char * _timestamp_pcointerface();
char * _timestamp_pcobufferctrlobj();
char * _timestamp_pcodetinfoctrlobj();
char * _timestamp_pcocamerautils();
char * _timestamp_pcoroictrlobj();
char *_split_date(char *s);

//=========================================================================================================
char* _timestamp_pcocamera() {return ID_TIMESTAMP ;}
char * _timestamp_pcogitversion(){return PCO_GIT_VERSION ;};
char * _timestamp_proclibgitversion(){return PROCLIB_GIT_VERSION ;};
char * _timestamp_libconfiggitversion(){return LIBCONFIG_GIT_VERSION ;};
char * _timestamp_limagitversion(){return LIMA_GIT_VERSION ;};

//=========================================================================================================

//=========================================================================================================
//=========================================================================================================
char *xlatCode2Str(int code, struct stcXlatCode2Str *stc) {

	char *type;

	while( (type = stc->str) != NULL) {
		if(stc->code == code) return type;
		stc++;
	}

	return NULL;

}

//=========================================================================================================
//=========================================================================================================

enum tblXlatCode2Str {ModelType, InterfaceType};

char *xlatPcoCode2Str(int code, tblXlatCode2Str table, int &err) {
	struct stcXlatCode2Str modelType[] = {
		{CAMERATYPE_PCO1200HS, "PCO 1200 HS"},
		{CAMERATYPE_PCO1300, "PCO 1300"},
		{CAMERATYPE_PCO1600, "PCO 1600"},
		{CAMERATYPE_PCO2000, "PCO 2000"},
		{CAMERATYPE_PCO4000, "PCO 4000"},
		{CAMERATYPE_PCO_DIMAX_STD, "PCO DIMAX STD"},
		{CAMERATYPE_PCO_DIMAX_TV, "PCO DIMAX TV"},
		{CAMERATYPE_PCO_DIMAX_AUTOMOTIVE, "PCO DIMAX AUTOMOTIVE"},
		{CAMERATYPE_PCO_EDGE, "PCO EDGE"},
		{CAMERATYPE_PCO_EDGE_GL, "PCO EDGE GL"},
		{0, NULL}
	};

  struct stcXlatCode2Str interfaceType[] = {
		{INTERFACE_FIREWIRE, "FIREWIRE"},
		{INTERFACE_CAMERALINK, "CAMERALINK"},
		{INTERFACE_USB, "USB"},
		{INTERFACE_ETHERNET, "ETHERNET"},
		{INTERFACE_SERIAL, "SERIAL"},
		{0, NULL}
	};

  struct stcXlatCode2Str *stc;
	char *ptr;
	static char buff[BUFF_XLAT_SIZE+1];

  switch(table) {
    case ModelType: stc = modelType; break;
    case InterfaceType: stc = interfaceType; break;
    default:
  		sprintf_s(buff, BUFF_XLAT_SIZE, "UNKNOWN XLAT TABLE [%d]", table);
  		err = 1;
	  	return buff;
  }

	if((ptr = xlatCode2Str(code, stc)) != NULL) {
		err = 0;
		return ptr;
	} else {
		sprintf_s(buff, BUFF_XLAT_SIZE, "UNKNOWN %s code [0x%04x]", (table == ModelType) ? "MODEL" : "INTERFACE", code);
		err = 1;
		return buff;
	}
}



//=========================================================================================================
//=========================================================================================================




stcPcoData::stcPcoData(){

	char *ptr, *ptrMax;
	int i;

	memset(this, 0, sizeof(struct stcPcoData));

	ptr = version; *ptr = 0;
	ptrMax = ptr + sizeof(version) - 1;

	ptr += sprintf_s(ptr, ptrMax - ptr, "\n");
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _split_date(_timestamp_pcocamera()));
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _split_date(_timestamp_pcosyncctrlobj()));
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _split_date(_timestamp_pcointerface()));
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _split_date(_timestamp_pcobufferctrlobj()));
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _split_date(_timestamp_pcodetinfoctrlobj()));
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _split_date(_timestamp_pcocamerautils()));
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _split_date(_timestamp_pcoroictrlobj()));
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcogitversion());
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_limagitversion());
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_proclibgitversion());
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_libconfiggitversion());
	


	stcPcoGeneral.wSize = sizeof(stcPcoGeneral);
	stcPcoGeneral.strCamType.wSize = sizeof(stcPcoGeneral.strCamType);
	stcPcoCamType.wSize = sizeof(stcPcoCamType);
	stcPcoSensor.wSize = sizeof(stcPcoSensor);
	stcPcoSensor.strDescription.wSize = sizeof(stcPcoSensor.strDescription);
	stcPcoSensor.strDescription2.wSize = sizeof(stcPcoSensor.strDescription2);
	stcPcoDescription.wSize = sizeof(stcPcoDescription);
	stcPcoTiming.wSize = sizeof(stcPcoTiming);
	stcPcoStorage.wSize = sizeof(stcPcoStorage);
	stcPcoRecording.wSize = sizeof(stcPcoRecording);

	for(i=0; i < SIZEARR_stcPcoHWIOSignal; i++) {
		stcPcoHWIOSignal[i].wSize = sizeof(stcPcoHWIOSignal[i]);
		stcPcoHWIOSignalDesc[i].wSize = sizeof(stcPcoHWIOSignalDesc[i]);
	}

	bAllocatedBufferDone = 
		false;
	
	msAcqRecTimestamp = msAcqXferTimestamp =
			getTimestamp();

	/* msAcqRec = msAcqXfer =
	iAllocatedBufferNumber = 
	dwAllocatedBufferSize = 
	iAllocatedBufferNumberLima =
		0;

	debugLevel = 0;
	*/
}

//=========================================================================================================
//=========================================================================================================
Camera::Camera(const char *camPar) :
	m_cam_connected(false),
	m_acq_frame_nb(1),
	m_sync(NULL)
{
	DEF_FNID;

	DEB_CONSTRUCTOR();
	int error=0;
	m_config = TRUE;
	DebParams::checkInit();

	m_msgLog = new ringLog(100) ;
	if(m_msgLog == NULL)
		throw LIMA_HW_EXC(Error, "m_msgLog > creation error");


	m_pcoData =new stcPcoData();
	if(m_pcoData == NULL)
		throw LIMA_HW_EXC(Error, "m_pcoData > creation error");
	//memset((char *)m_pcoData, 0, sizeof(stcPcoData));
    DEB_ALWAYS()  << DEB_VAR1(m_pcoData->version) << _checkLogFiles();

	m_bin.changed = Invalid;
	m_roi.changed = Invalid;

	_init();
	m_config = FALSE;

}


//=========================================================================================================
//=========================================================================================================
void Camera::_init(){
	DEB_CONSTRUCTOR();
	char msg[MSG_SIZE + 1];
	int error=0;
	char *errMsg;

	m_config = FALSE;

	m_log.clear();
	sprintf_s(msg, MSG_SIZE, "*** Pco log %s\n", getTimestamp(Iso));
	m_log.append(msg);


		// --- Open Camera
	error = PcoCheckError(__LINE__, __FILE__, PCO_OpenCamera(&m_handle, 0));
	PCO_THROW_OR_TRACE(error, "PCO_OpenCamera") ;

	errMsg = _pcoGet_Camera_Type(error);
	PCO_THROW_OR_TRACE(error, errMsg) ;


	// -- Initialise ADC
	//-------------------------------------------------------------------------------------------------
	// PCO_SetADCOperation
    // Set analog-digital-converter (ADC) operation for reading the image sensor data. Pixel data can be
    // read out using one ADC (better linearity) or in parallel using two ADCs (faster). This option is
    // only available for some camera models. If the user sets 2ADCs he must center and adapt the ROI
    // to symmetrical values, e.g. pco.1600: x1,y1,x2,y2=701,1,900,500 (100,1,200,500 is not possible).
    //
	// DIMAX -> 1 adc
	//-------------------------------------------------------------------------------------------------
	
	m_pcoData->wNumADC = m_pcoData->stcPcoDescription.wNumADCsDESC; // nr of ADC in the system

	if(m_pcoData->wNumADC > 1) {
		WORD wADCOperation;

		error = PcoCheckError(__LINE__, __FILE__, PCO_GetADCOperation(m_handle, &wADCOperation));

		if(wADCOperation != 1){
			wADCOperation = 1;
			error = PcoCheckError(__LINE__, __FILE__, PCO_SetADCOperation(m_handle, wADCOperation));
			error = PcoCheckError(__LINE__, __FILE__, PCO_GetADCOperation(m_handle, &wADCOperation));
		}
		m_pcoData->wNowADC= wADCOperation;
	} else {
		m_pcoData->wNowADC= 1;
	}

		// -- Initialise size, bin, roi
	m_pcoData->maxWidth = (unsigned int) m_pcoData->stcPcoDescription.wMaxHorzResStdDESC; // ds->ccd.size.xmax,
	m_pcoData->maxHeight= (unsigned int) m_pcoData->stcPcoDescription.wMaxVertResStdDESC; // ds->ccd.size.ymax,
	m_pcoData->bitsPerPix = (unsigned int) m_pcoData->stcPcoDescription.wDynResDESC; // ds->ccd.size.bits
	m_pcoData->bytesPerPix = (m_pcoData->bitsPerPix <= 8)?1:2; // nr de bytes por pixel  12 bits -> 2 bytes


	m_pcoData->maxwidth_step= (unsigned int) m_pcoData->stcPcoDescription.wRoiHorStepsDESC;   // ds->ccd.roi.xstep
	m_pcoData->maxheight_step= (unsigned int) m_pcoData->stcPcoDescription.wRoiVertStepsDESC; // ds->ccd.roi.ystep,

	m_roi.x[0] = m_roi.y[0] = 1;
	m_roi.x[1] = m_pcoData->maxWidth;
	m_roi.y[1] = m_pcoData->maxHeight;
	m_roi.changed = Changed;

	_get_MaxRoi(m_RoiLima);

	sprintf_s(msg, MSG_SIZE, "* CCD Size = X[%d] * Y[%d] (%d bits)\n", m_pcoData->maxWidth, m_pcoData->maxHeight, m_pcoData->bitsPerPix);
	DEB_TRACE() <<   msg;
	m_log.append(msg);
	
	sprintf_s(msg, MSG_SIZE, "* ROI Steps = x:%d, y:%d\n", m_pcoData->maxwidth_step, m_pcoData->maxheight_step);
	DEB_TRACE() <<   msg;
	m_log.append(msg);

	errMsg = _pcoGet_TemperatureInfo(error);
	PCO_THROW_OR_TRACE(error, errMsg) ;

	_pcoSet_RecordingState(0, error);
	
	if(_isCameraType(Dimax)) _init_dimax();
	else if(_isCameraType(Pco2k)) _init_dimax();
	else if(_isCameraType(Pco4k)) _init_dimax();
	else if(_isCameraType(Edge)) _init_edge();
	else {
		char msg[MSG_SIZE+1];
		sprintf_s(msg, MSG_SIZE, "Camera type not supported! [x%04x]", _getCameraType());
		DEB_ALWAYS() <<  msg;

		throw LIMA_HW_EXC(Error, msg);
	}


	m_cam_connected = true;
	error = 0;

	if(!m_cam_connected)
		throw LIMA_HW_EXC(Error, "Camera not found!");

	_pco_initHWIOSignal(0, error);

  DEB_TRACE() << m_log;
  DEB_TRACE() << "END OF CAMERA";

}

//=========================================================================================================
//=========================================================================================================
void  Camera::_init_dimax() {

	DEB_CONSTRUCTOR();
	char msg[MSG_SIZE + 1];

	int error=0;
	DWORD _dwValidImageCnt, _dwMaxImageCnt;

	
	// block #1 -- Get RAM size
	{
		int segmentPco, segmentArr;

		DWORD ramSize;
		WORD pageSize;
		
		error = PcoCheckError(__LINE__, __FILE__, PCO_GetCameraRamSize(m_handle, &ramSize, &pageSize));
		PCO_THROW_OR_TRACE(error, "PCO_GetCameraRamSize") ;

		m_pcoData->dwRamSize = ramSize;     // nr of pages of the ram
		m_pcoData->wPixPerPage = pageSize;    // nr of pixels of the page

		sprintf_s(msg, MSG_SIZE, "* ramPages[%ld] pixPerPage[%d] bitsPerPix[%d]\n",  
				m_pcoData->dwRamSize, m_pcoData->wPixPerPage, m_pcoData->bitsPerPix);
		DEB_TRACE() <<   msg;
		m_log.append(msg);
		
		double nrBytes = (double) m_pcoData->dwRamSize  * (double) m_pcoData->wPixPerPage * 
			(double)m_pcoData->bitsPerPix / 9.; // 8 bits data + 1 bit CRC -> 9
		
		sprintf_s(msg, MSG_SIZE, "* camMemorySize [%lld B] [%g GB]\n",  
				(long long int) nrBytes, nrBytes/GIGABYTE);
		DEB_TRACE() <<   msg;
		m_log.append(msg);

		// ----------------- get initial seg Size - images & print

		// ---- get the size in pages of each of the 4 segments

		DWORD   segSize[4];
		error = PcoCheckError(__LINE__, __FILE__, PCO_GetCameraRamSegmentSize(m_handle, segSize));
		PCO_THROW_OR_TRACE(error, "PCO_GetCameraRamSegmentSize") ;

		for(segmentArr=0; segmentArr < PCO_MAXSEGMENTS ; segmentArr++) {
			segmentPco = segmentArr +1;		// PCO segment (1 ... 4)
			m_pcoData->dwSegmentSize[segmentArr] = segSize[segmentArr];

			sprintf_s(msg, MSG_SIZE, "* segment[%d] number of pages[%ld]\n", segmentPco,m_pcoData->dwSegmentSize[segmentArr]);
			DEB_TRACE() <<   msg;
			m_log.append(msg);

		}

		//---- get nr de images in each segment & nr max of img on each segmente
		for(segmentArr=0;  segmentArr< PCO_MAXSEGMENTS ; segmentArr++) {
			segmentPco = segmentArr +1;

			error = PcoCheckError(__LINE__, __FILE__, PCO_GetNumberOfImagesInSegment(m_handle, segmentPco, &_dwValidImageCnt, &_dwMaxImageCnt));
			PCO_THROW_OR_TRACE(error, "PCO_GetNumberOfImagesInSegment") ;

			m_pcoData->dwValidImageCnt[segmentArr] = _dwValidImageCnt;
			m_pcoData->dwMaxImageCnt[segmentArr] = _dwMaxImageCnt;

			sprintf_s(msg, MSG_SIZE, "* segment[%d] nr images [%ld]  max imag [%ld]\n", segmentPco, _dwValidImageCnt,  _dwMaxImageCnt);
			DEB_TRACE() <<   msg;
			m_log.append(msg);

		} // for	


		// set the first segment to the max ram size, the others = 0
		// This function will result in all segments being cleared. 
		// All previously recorded images will be lost!

		//m_pcoData->dwSegmentSize[0] = m_pcoData->dwRamSize;
		for(segmentArr=1;  segmentArr < PCO_MAXSEGMENTS ; segmentArr++) {
			m_pcoData->dwSegmentSize[0] += m_pcoData->dwSegmentSize[segmentArr];
			m_pcoData->dwSegmentSize[segmentArr] = 0;
		}
		sprintf_s(msg, MSG_SIZE, "* m_pcoData->dwSegmentSize0 [%ld]  m_pcoData->dwRamSize [%ld]\n", m_pcoData->dwSegmentSize[0], m_pcoData->dwRamSize);
		DEB_TRACE() <<   msg;
		m_log.append(msg);


		error = PcoCheckError(__LINE__, __FILE__, PCO_SetCameraRamSegmentSize(m_handle, &m_pcoData->dwSegmentSize[0]));
		PCO_THROW_OR_TRACE(error, "PCO_SetCameraRamSegmentSize") ;
	}  // block #1 

	DEB_TRACE() <<  "end block 1 / get initial seg Size - images";

	{
		int segmentPco, segmentArr;
		DWORD pages_per_image = m_pcoData->maxWidth * m_pcoData->maxHeight / m_pcoData->wPixPerPage;

		///------------------------------------------------------------------------TODO ?????
		for(segmentArr=0; segmentArr < PCO_MAXSEGMENTS ; segmentArr++) {
			segmentPco = segmentArr +1;
			if(m_pcoData->dwMaxImageCnt[segmentArr] == 0){
				m_pcoData->dwMaxImageCnt[segmentArr] = m_pcoData->dwSegmentSize[segmentArr] / pages_per_image;
				if(m_pcoData->dwMaxImageCnt[segmentArr] > 4) m_pcoData->dwMaxImageCnt[segmentArr] -= 2;
			}
		}	
	} // block


	// -- Get Active RAM segment 

		error = PcoCheckError(__LINE__, __FILE__, PCO_GetActiveRamSegment(m_handle, &m_pcoData->activeRamSegment));
		PCO_THROW_OR_TRACE(error, "PCO_GetActiveRamSegment") ;

		error = PcoCheckError(__LINE__, __FILE__, PCO_GetNumberOfImagesInSegment(m_handle, m_pcoData->activeRamSegment, &_dwValidImageCnt, &_dwMaxImageCnt));
		PCO_THROW_OR_TRACE(error, "PCO_GetNumberOfImagesInSegment") ;



	DEB_TRACE() <<  "original DONE";


}


//=========================================================================================================
//=========================================================================================================
void Camera::_init_edge() {

	m_pcoData->fTransferRateMHzMax = 550.;



}



//=========================================================================================================
//=========================================================================================================
Camera::~Camera()
{
  DEB_DESTRUCTOR();
  DEB_TRACE() << "~Camera";
	int error;

  if(m_cam_connected){
		//m_sync->_getBufferCtrlObj()->_pcoAllocBuffersFree();
	}
	m_cam_connected = false;

	m_sync->_getBufferCtrlObj()->_pcoAllocBuffersFree();
	error = PcoCheckError(__LINE__, __FILE__, PCO_CloseCamera(m_handle));
	char *msg ="PCO_CloseCamera";
	PCO_PRINT_ERR(error, msg); 

}



//=================================================================================================
//=================================================================================================
void Camera::getCameraName(std::string& name)
{
  DEB_MEMBER_FUNCT();
  DEB_RETURN() << DEB_VAR1(m_pcoData->camera_name);

  name = m_pcoData->camera_name;
}


//=================================================================================================
//=================================================================================================
void Camera::startAcq()
{
    DEB_MEMBER_FUNCT();

	m_acq_frame_nb = -1;
	m_pcoData->pcoError = 0;
	m_pcoData->pcoErrorMsg[0] = 0;


//=====================================================================
	DEF_FNID;
    WORD state;
    HANDLE hEvent= NULL;

	int error;
	char *msg;

    int iRequestedFrames;

			// live video requested frames = 0
    m_sync->getNbFrames(iRequestedFrames);

    //------------------------------------------------- set binning if needed
    WORD wBinHorz, wBinVert;
    if (m_bin.changed == Changed) {
		wBinHorz = (WORD)m_bin.x;
		wBinVert = (WORD)m_bin.y;
        error = PcoCheckError(__LINE__, __FILE__, PCO_SetBinning(m_handle, wBinHorz, wBinVert));
        PCO_THROW_OR_TRACE(error, "PCO_SetBinning") ;
        m_bin.changed= Valid;
    }

    error = PcoCheckError(__LINE__, __FILE__, PCO_GetBinning(m_handle, &wBinHorz, &wBinVert));
    PCO_THROW_OR_TRACE(error, "PCO_GetBinning") ;
	DEB_TRACE() << DEB_VAR2(wBinHorz, wBinVert);

    //------------------------------------------------- set roi if needed
    WORD wRoiX0, wRoiY0; // Roi upper left x y
    WORD wRoiX1, wRoiY1; // Roi lower right x y

    
	if(m_roi.changed == Valid) m_roi.changed = Changed;    //+++++++++ TEST / FORCE WRITE ROI
	m_roi.changed = Changed;
	if (m_roi.changed == Changed) {

		{
			Point top_left = m_RoiLima.getTopLeft();
			Point bot_right = m_RoiLima.getBottomRight();
			Size size = m_RoiLima.getSize();

			wRoiX0 = (WORD)top_left.x + 1;
			wRoiY0 = (WORD)top_left.y + 1;
			wRoiX1 = (WORD)bot_right.x + 1; 
			wRoiY1 = (WORD)bot_right.y + 1;
		}

        wRoiX0 = (WORD)m_roi.x[0]; wRoiX1 = (WORD)m_roi.x[1];
        wRoiY0 = (WORD)m_roi.y[0]; wRoiY1 = (WORD)m_roi.y[1];

		if(_getDebug(DBG_ROI)) {
			DEB_ALWAYS() << DEB_VAR5(m_RoiLima, wRoiX0, wRoiY0, wRoiX1, wRoiY1);
		}

        error = PcoCheckError(__LINE__, __FILE__, PCO_SetROI(m_handle, wRoiX0, wRoiY0, wRoiX1, wRoiY1));
        PCO_THROW_OR_TRACE(error, "PCO_SetROI") ;

        m_roi.changed= Valid;
    }

	error = PcoCheckError(__LINE__, __FILE__, PCO_GetROI(m_handle, &wRoiX0, &wRoiY0, &wRoiX1, &wRoiY1));
    PCO_THROW_OR_TRACE(error, "PCO_GetROI") ;

	if(_getDebug(DBG_ROI)) {
		DEB_ALWAYS() << DEB_VAR4(wRoiX0, wRoiY0, wRoiX1, wRoiY1);
	}


	//------------------------------------------------- triggering mode 
    //------------------------------------- acquire mode : ignore or not ext. signal
	msg = _pcoSet_Trig_Acq_Mode(error);
    PCO_THROW_OR_TRACE(error, msg) ;

    // ----------------------------------------- storage mode (recorder + sequence)
    if(_isCameraType(Dimax)) {
		
			// live video requested frames = 0
		enumPcoStorageMode mode = (iRequestedFrames > 0) ? RecSeq : RecRing;

		msg = _pcoSet_Storage_subRecord_Mode(mode, error);
		PCO_THROW_OR_TRACE(error, msg) ;
	}

	//----------------------------------- set exposure time & delay time
	msg = _pcoSet_Exposure_Delay_Time(error,0);
	PCO_THROW_OR_TRACE(error, msg) ;


    //------------------------------------------------- check recording state
    error = PcoCheckError(__LINE__, __FILE__, PCO_GetRecordingState(m_handle, &state));
    PCO_THROW_OR_TRACE(error, "PCO_GetRecordingState") ;

    if (state>0) {
        DEB_TRACE() << "Force recording state to 0x0000" ;

		_pcoSet_RecordingState(0, error);
        PCO_THROW_OR_TRACE(error, "PCO_SetRecordingState") ;
	}

//-----------------------------------------------------------------------------------------------
//	5. Arm the camera.
//	6. Get the sizes and allocate a buffer:
//		PCO_GETSIZES(hCam, &actualsizex, &actualsizey, &ccdsizex, &ccdsizey)
//		PCO_ALLOCATEBUFFER(hCam, &bufferNr, actualsizex * actualsizey * sizeof(WORD), &data, &hEvent)
//		In case of CamLink and GigE interface: PCO_CamLinkSetImageParameters(actualsizex, actualsizey)
//		PCO_ArmCamera(hCam)
//-----------------------------------------------------------------------------------------------
	
	msg = _set_metadata_mode(0, error); PCO_THROW_OR_TRACE(error, msg) ;

	// ------------------------------------------------- arm camera
    error = PcoCheckError(__LINE__, __FILE__, PCO_ArmCamera(m_handle)); PCO_THROW_OR_TRACE(error, "PCO_ArmCamera") ;

		if(_isCameraType(Edge)) {
			error = PcoCheckError(__LINE__, __FILE__, PCO_GetPixelRate(m_handle, &m_pcoData->dwPixelRate));
		    PCO_THROW_OR_TRACE(error, "PCO_GetPixelRate") ;

			if(_isValid_pixelRate(m_pcoData->dwPixelRateRequested) && 
					(m_pcoData->dwPixelRate != m_pcoData->dwPixelRateRequested)) {

				error = PcoCheckError(__LINE__, __FILE__, PCO_SetPixelRate(m_handle, m_pcoData->dwPixelRateRequested));
			    PCO_THROW_OR_TRACE(error, "PCO_SetPixelRate") ;

				error = PcoCheckError(__LINE__, __FILE__, PCO_GetPixelRate(m_handle, &m_pcoData->dwPixelRate));
			    PCO_THROW_OR_TRACE(error, "PCO_GetPixelRate") ;

				error = PcoCheckError(__LINE__, __FILE__, PCO_ArmCamera(m_handle));
			    PCO_THROW_OR_TRACE(error, "PCO_ArmCamera") ;
			}
			m_pcoData->dwPixelRateRequested = 0;
		}

		if(_isCameraType(Pco2k | Pco4k)) {
			DWORD _dwPixelRate;

			error = PcoCheckError(__LINE__, __FILE__, PCO_GetPixelRate(m_handle, &m_pcoData->dwPixelRate));
		    PCO_THROW_OR_TRACE(error, "PCO_GetPixelRate") ;

			_dwPixelRate = m_pcoData->dwPixelRate;

			if(m_pcoData->dwPixelRateMax > m_pcoData->dwPixelRate) {

				error = PcoCheckError(__LINE__, __FILE__, PCO_SetPixelRate(m_handle,m_pcoData->dwPixelRateMax));
			    PCO_THROW_OR_TRACE(error, "PCO_SetPixelRate") ;

				error = PcoCheckError(__LINE__, __FILE__, PCO_GetPixelRate(m_handle, &m_pcoData->dwPixelRate));
			    PCO_THROW_OR_TRACE(error, "PCO_GetPixelRate") ;
				DEB_ALWAYS() << DEB_VAR2(_dwPixelRate, m_pcoData->dwPixelRate) ;

				error = PcoCheckError(__LINE__, __FILE__, PCO_ArmCamera(m_handle));
			    PCO_THROW_OR_TRACE(error, "PCO_ArmCamera") ;
			}

		}

		
		
	//====================================== get the coc runtime 
    //---- only valid if it was used PCO_SetDelayExposureTime
    //---- and AFTER armed the cam
	msg = _get_coc_runtime(error); PCO_THROW_OR_TRACE(error, msg) ;


    //--------------------------- PREPARE / getSizes, pixelRate, clXferParam, LUT, setImgParam, Arm

	error = PcoCheckError(__LINE__, __FILE__, PCO_GetSizes(m_handle, &m_pcoData->wXResActual, &m_pcoData->wYResActual, &m_pcoData->wXResMax, &m_pcoData->wYResMax));
    PCO_THROW_OR_TRACE(error, "PCO_GetSizes") ;

	m_sync->_getBufferCtrlObj()->_pcoAllocBuffers();

	msg = _prepare_cameralink_interface(error); PCO_THROW_OR_TRACE(error, msg) ;
	msg = _pcoSet_Exposure_Delay_Time(error,1); PCO_THROW_OR_TRACE(error, msg) ;
	error = PcoCheckError(__LINE__, __FILE__, PCO_ArmCamera(m_handle)); PCO_THROW_OR_TRACE(error, "PCO_ArmCamera") ;
	msg = _get_coc_runtime(error); PCO_THROW_OR_TRACE(error, msg) ;

    //------------------------------------------------- checking nr of frames
    if(_isCameraType(Dimax)){
        unsigned long framesMax;
        framesMax = pcoGetFramesMax(m_pcoData->activeRamSegment);

        if ((((unsigned long) iRequestedFrames) > framesMax)) {
            throw LIMA_HW_EXC(Error, "frames OUT OF RANGE");
        }
    } 
	

	//------------------------------------------------- start acquisition

	m_sync->setStarted(true);
	m_sync->setExposing(pcoAcqRecordStart);

	if(_isCameraType(Edge)){
		_beginthread( _pco_acq_thread_edge, 0, (void*) this);
		return;
	}

	if(_isCameraType(Dimax | Pco2k | Pco4k)){
		_pcoSet_RecordingState(1, error);
		if(iRequestedFrames > 0 ) {
			_beginthread( _pco_acq_thread_dimax, 0, (void*) this);
		} else {
			_beginthread( _pco_acq_thread_dimax_live, 0, (void*) this);
		}
		return;
	}

	throw LIMA_HW_EXC(Error, "unkown camera type");
	return;
}


//==========================================================================================================
//==========================================================================================================

long msElapsedTime(struct __timeb64 &t0) {
	struct __timeb64 tNow;
	_ftime64_s(&tNow);

	return (long)((tNow.time - t0.time)*1000) + (tNow.millitm - t0.millitm);
}

void msElapsedTimeSet(struct __timeb64 &t0) {
	_ftime64_s(&t0);
}

//==========================================================================================================
//==========================================================================================================

void _pco_acq_thread_dimax(void *argin) {
	DEF_FNID;
	int error;

	DWORD _dwValidImageCnt, _dwMaxImageCnt;

	printf("=== %s [%d]> %s ENTRY\n",  fnId, __LINE__,getTimestamp(Iso));

	Camera* m_cam = (Camera *) argin;
	SyncCtrlObj* m_sync = m_cam->_getSyncCtrlObj();
	BufferCtrlObj* m_buffer = m_sync->_getBufferCtrlObj();

	struct stcPcoData *m_pcoData = m_cam->_getPcoData();
	m_pcoData->traceAcqClean();
	
	m_pcoData->traceAcq.fnId = fnId;
	
	char *msg;
	struct __timeb64 tStart, tStart0;
	msElapsedTimeSet(tStart);
	tStart0 = tStart;

	long timeout, timeout0, msNow, msRec, msXfer, msAll;
	int nb_acq_frames;
	int requestStop = stopNone;

	HANDLE m_handle = m_cam->getHandle();

	
	WORD wSegment = m_cam->pcoGetActiveRamSegment(); 
	double msPerFrame = (m_cam->pcoGetCocRunTime() * 1000.);
	m_pcoData->traceAcq.msImgCoc = msPerFrame;

	DWORD dwMsSleepOneFrame = (DWORD) (msPerFrame + 0.5);	// 4/5 rounding
	if(dwMsSleepOneFrame == 0) dwMsSleepOneFrame = 1;		// min sleep

	bool nb_frames_fixed = false;
	int nb_frames; 	m_sync->getNbFrames(nb_frames);
	//m_pcoData->traceAcq.nrImgRequested = nb_frames;
	m_pcoData->traceAcq.nrImgRequested0 = nb_frames;

	m_sync->setAcqFrames(0);

	timeout = timeout0 = (long) (msPerFrame * (nb_frames * 1.3));	// 30% guard
	if(timeout < TOUT_MIN_DIMAX) timeout = TOUT_MIN_DIMAX;
    
	m_pcoData->traceAcq.msTout = m_pcoData->msAcqTout = timeout;
	_dwValidImageCnt = 0;

	m_sync->getExpTime(m_pcoData->traceAcq.sExposure);
	m_sync->getLatTime(m_pcoData->traceAcq.sDelay);

	m_sync->setExposing(pcoAcqRecordStart);

	while(true) {
		msg = m_cam->_PcoCheckError(__LINE__, __FILE__, PCO_GetNumberOfImagesInSegment(m_handle, wSegment, &_dwValidImageCnt, &_dwMaxImageCnt), error);
		if(error) {
			printf("=== %s [%d]> ERROR %s\n", fnId, __LINE__, msg);
			throw LIMA_HW_EXC(Error, "PCO_GetNumberOfImagesInSegment");
		}

		m_pcoData->dwValidImageCnt[wSegment-1] = 
			m_pcoData->traceAcq.nrImgRecorded = _dwValidImageCnt;
		m_pcoData->dwMaxImageCnt[wSegment-1] =
			m_pcoData->traceAcq.maxImgCount = _dwMaxImageCnt;

		m_pcoData->msAcqTnow = msNow = msElapsedTime(tStart);
		m_pcoData->traceAcq.msRecordLoop = msNow;
	
		if( ((DWORD) nb_frames > _dwMaxImageCnt) ){
			nb_frames_fixed = true;
			printf("=== %s [%d]> ERROR INVALID NR FRAMES fixed nb_frames[%d] _dwMaxImageCnt[%d]\n", 
				fnId, __LINE__, nb_frames, _dwMaxImageCnt);
			m_sync->setExposing(pcoAcqError);
			break;
		}

		if(  (_dwValidImageCnt >= (DWORD) nb_frames)) break;

		if((timeout < msNow) && !m_pcoData->bExtTrigEnabled) { 
			//m_sync->setExposing(pcoAcqRecordTimeout);
			m_sync->stopAcq();
			m_sync->setExposing(pcoAcqStop);
			printf("=== %s [%d]> TIMEOUT!!! tout[(%ld) 0(%ld)] recLoopTime[%ld ms] lastImgRecorded[%ld] nrImgRequested[%d]\n", 
				fnId, __LINE__, timeout, timeout0, msNow, _dwValidImageCnt, nb_frames);
			break;
		}
	
		if(requestStop = m_buffer->_getRequestStop()) {
			m_sync->setExposing(pcoAcqStop);
			printf("=== %s> STOP REQUESTED\n", fnId);
			break;
		}

		Sleep(dwMsSleepOneFrame);	// sleep 1 frame
	}

	msg = m_cam->_pcoSet_RecordingState(0, error);
	if(error) {
		printf("=== %s [%d]> ERROR %s\n", fnId, __LINE__, msg);
		throw LIMA_HW_EXC(Error, "_pcoSet_RecordingState");
	}

	if(!nb_frames_fixed) {
		if(m_sync->getExposing() == pcoAcqRecordStart) m_sync->setExposing(pcoAcqRecordEnd);

		msg = m_cam->_PcoCheckError(__LINE__, __FILE__, PCO_GetNumberOfImagesInSegment(m_handle, wSegment, &_dwValidImageCnt, &_dwMaxImageCnt), error);
		if(error) {
			printf("=== %s [%d]> ERROR %s\n", fnId, __LINE__, msg);
			throw LIMA_HW_EXC(Error, "PCO_GetNumberOfImagesInSegment");
		}

		m_pcoData->dwValidImageCnt[wSegment-1] = 
			m_pcoData->traceAcq.nrImgRecorded = _dwValidImageCnt;

		nb_acq_frames = (_dwValidImageCnt < (DWORD) nb_frames) ? _dwValidImageCnt : nb_frames;
		//m_sync->setAcqFrames(nb_acq_frames);


		// dimax recording time
		m_pcoData->msAcqRec = msRec = msElapsedTime(tStart);
		m_pcoData->traceAcq.endRecordTimestamp = m_pcoData->msAcqRecTimestamp = getTimestamp();
		m_pcoData->traceAcq.nrImgAcquired = nb_acq_frames;
		m_pcoData->traceAcq.nrImgRequested = nb_frames;

		msElapsedTimeSet(tStart);

		if(m_buffer->_getRequestStop()) {
			m_sync->setExposing(pcoAcqStop);
		} else {
				pcoAcqStatus status;

				if(m_cam->_isCameraType(Pco2k | Pco4k)){
					status = (pcoAcqStatus) m_buffer->_xferImag();
					if(nb_frames_fixed) status = pcoAcqError;
				}else{
					status = (pcoAcqStatus) m_buffer->_xferImagMult();
					//status = (pcoAcqStatus) m_buffer->_xferImagTest(); <---- DIMAX
				}
				m_sync->setExposing(status);

				if(!m_buffer->_getRequestStop()) m_sync->stopAcq();
		}
	} // if nb_frames_fixed

	//m_sync->setExposing(status);
	m_pcoData->dwMaxImageCnt[wSegment-1] =
			m_pcoData->traceAcq.maxImgCount = _dwMaxImageCnt;

	// dimax xfer time
	m_pcoData->msAcqXfer = msXfer = msElapsedTime(tStart);
	m_pcoData->msAcqAll = msAll = msElapsedTime(tStart0);
	m_pcoData->traceAcq.endXferTimestamp = m_pcoData->msAcqXferTimestamp = getTimestamp();

	m_pcoData->traceAcq.msRecord = msRec;
	m_pcoData->traceAcq.msRecordLoop = msNow;
	m_pcoData->traceAcq.msXfer = msXfer;
	m_pcoData->traceAcq.msTotal= msAll;

	


	printf("=== %s [%d]> EXIT imgRecorded[%d] coc[%g] recLoopTime[%ld] "
			"tout[(%ld) 0(%ld)] rec[%ld] xfer[%ld] all[%ld](ms)\n", 
			fnId, __LINE__, _dwValidImageCnt, msPerFrame, msNow, timeout, timeout0, msRec, msXfer, msAll);

	_endthread();
}

//==========================================================================================================
//==========================================================================================================

char *sPcoAcqStatus[] ={
	"pcoAcqIdle", 
	"pcoAcqStart", 
	"pcoAcqRecordStart", 
	"pcoAcqRecordEnd",  
	"pcoAcqTransferStart", 
	"pcoAcqTransferEnd", 
	"pcoAcqStop", 
	"pcoAcqTransferStop", 
	"pcoAcqRecordTimeout",
	"pcoAcqWaitTimeout",
	"pcoAcqWaitError",
	"pcoAcqError",
	"pcoAcqPcoError",
};

//=====================================================================
//=====================================================================
void _pco_shutter_thread_edge(void *argin) {
	DEF_FNID;
	int error;

	printf("=== %s %s> ENTRY\n", fnId, getTimestamp(Iso));

	Camera* m_cam = (Camera *) argin;
	m_cam->_pco_set_shutter_rolling_edge(error);

	printf("=== %s> EXIT\n", fnId);

	_endthread();
}


//=====================================================================
//=====================================================================

void _pco_acq_thread_edge(void *argin) {
	DEF_FNID;


	printf("=== %s> ENTRY\n", fnId);

	Camera* m_cam = (Camera *) argin;
	SyncCtrlObj* m_sync = m_cam->_getSyncCtrlObj();
	BufferCtrlObj* m_buffer = m_sync->_getBufferCtrlObj();

	struct stcPcoData *m_pcoData = m_cam->_getPcoData();

	struct __timeb64 tStart;
	msElapsedTimeSet(tStart);
	int error;
	long msXfer;
	int requestStop = stopNone;

	HANDLE m_handle = m_cam->getHandle();

	m_sync->setAcqFrames(0);

	pcoAcqStatus status = (pcoAcqStatus) m_buffer->_xferImag();
	m_sync->setExposing(status);
	m_sync->stopAcq();
	char *msg = m_cam->_pcoSet_RecordingState(0, error);
	if(error) {
		printf("=== %s [%d]> ERROR %s\n", fnId, __LINE__, msg);
		//throw LIMA_HW_EXC(Error, "_pcoSet_RecordingState");
	}


	m_pcoData->msAcqXfer = msXfer = msElapsedTime(tStart);
	printf("=== %s> EXIT xfer[%ld] (ms) status[%s]\n", 
			fnId, msXfer, sPcoAcqStatus[status]);
	_endthread();
}

//=====================================================================
//=====================================================================

void _pco_acq_thread_dimax_live(void *argin) {
	DEF_FNID;

	printf("=== %s> ENTRY\n", fnId);

	Camera* m_cam = (Camera *) argin;
	SyncCtrlObj* m_sync = m_cam->_getSyncCtrlObj();
	BufferCtrlObj* m_buffer = m_sync->_getBufferCtrlObj();

	struct stcPcoData *m_pcoData = m_cam->_getPcoData();

	struct __timeb64 tStart;
	msElapsedTimeSet(tStart);
	int error;
	long msXfer;
	int requestStop = stopNone;

	HANDLE m_handle = m_cam->getHandle();

	m_sync->setAcqFrames(0);

	// dimax recording time -> live NO record
	m_pcoData->msAcqRec  = 0;
	m_pcoData->msAcqRecTimestamp = getTimestamp();


	pcoAcqStatus status = (pcoAcqStatus) m_buffer->_xferImag();
	m_sync->setExposing(status);
	m_sync->stopAcq();
	char *msg = m_cam->_pcoSet_RecordingState(0, error);
	if(error) {
		printf("=== %s [%d]> ERROR %s\n", fnId, __LINE__, msg);
		//throw LIMA_HW_EXC(Error, "_pcoSet_RecordingState");
	}

	// dimax xfer time
	m_pcoData->msAcqXfer = msXfer = msElapsedTime(tStart);
	m_pcoData->msAcqXferTimestamp = getTimestamp();
	printf("=== %s> EXIT xfer[%ld] (ms) status[%s]\n", 
			fnId, msXfer, sPcoAcqStatus[status]);
	_endthread();
}

//=====================================================================
//=====================================================================
void Camera::reset()
{
  DEB_MEMBER_FUNCT();
  //@todo maybe something to do!
}



//=========================================================================================================
//=========================================================================================================
int Camera::PcoCheckError(int line, char *file, int err) {
	DEB_MEMBER_FUNCT();

	static char lastErrorMsg[500];
	char *msg;
	int lg;

	if (err != 0) {
		DWORD dwErr = err;
		m_pcoData->pcoError = err;
		msg = m_pcoData->pcoErrorMsg;

		PCO_GetErrorText(dwErr, msg, ERR_SIZE-14);
        
		lg = strlen(msg);
		sprintf_s(msg+lg,ERR_SIZE - lg, " [%s][%d]", file, line);

		if(err & PCO_ERROR_IS_WARNING) {
			DEB_WARNING() << "--- WARNING - IGNORED --- " << DEB_VAR1(m_pcoData->pcoErrorMsg);
			return 0;
		}
		DEB_ALWAYS() << DEB_VAR1(msg);
		return (err);
	}
	return (err);
}


//=========================================================================================================
//=========================================================================================================
char* Camera::_PcoCheckError(int line, char *file, int err, int &error) {
	static char lastErrorMsg[ERR_SIZE];
	char *msg;
	int lg;

	error = m_pcoData->pcoError = err;
	msg = m_pcoData->pcoErrorMsg;

	if (err != 0) {
		PCO_GetErrorText(err, lastErrorMsg, ERR_SIZE-14);
		strncpy_s(msg, ERR_SIZE, lastErrorMsg, _TRUNCATE); 

		lg = strlen(msg);
		sprintf_s(msg+lg,ERR_SIZE - lg, " [%s][%d]", file, line);

		return lastErrorMsg;
	}
	return NULL;
}

//=========================================================================================================
//=========================================================================================================
unsigned long Camera::pcoGetFramesMax(int segmentPco){
	DEF_FNID;

		int segmentArr = segmentPco-1;
		unsigned long framesMax;
		unsigned long xroisize,yroisize;
		unsigned long long pixPerFrame, pagesPerFrame;

		if(_isCameraType(Edge)) {
			return LONG_MAX;
		}


		if(!_isCameraType(Dimax | Pco2k | Pco4k)) {
			printf("=== %s> unknow camera type [%d]\n", fnId, _getCameraType());
			return -1;
		}

		if((segmentPco <1) ||(segmentPco > PCO_MAXSEGMENTS)) {
			printf("=== %s> ERROR segmentPco[%d]\n", fnId, segmentPco);
			return -1;
		}

		xroisize = m_RoiLima.getSize().getWidth();
		yroisize = m_RoiLima.getSize().getHeight();

		xroisize = m_roi.x[1] - m_roi.x[0] + 1;
		yroisize = m_roi.y[1] - m_roi.y[0] + 1;

		pixPerFrame = (unsigned long long)xroisize * (unsigned long long)yroisize;

		if(pixPerFrame <0) {
			printf("=== %s> ERROR pixPerFrame[%lld]\n", fnId, pixPerFrame);
			return -1;
		}

		if(m_pcoData->wPixPerPage < 1) {
			printf("=== %s> ERROR m_pcoData->wPixPerPage[%d]\n", fnId, m_pcoData->wPixPerPage);
			return -1;
		}
		pagesPerFrame = (pixPerFrame / m_pcoData->wPixPerPage) + 1;
		if(pixPerFrame % m_pcoData->wPixPerPage) pagesPerFrame++;

		framesMax = m_pcoData->dwMaxFramesInSegment[segmentArr] = (unsigned long)(((long long) m_pcoData->dwSegmentSize[segmentArr] ) / pagesPerFrame);

		return framesMax;
	}


//=================================================================================================
//=================================================================================================
char * Camera::_pcoSet_Trig_Acq_Mode(int &error){
	DEB_MEMBER_FUNCT();
	
	DEF_FNID;
	//------------------------------------------------- triggering mode 
	WORD trigmode = m_sync->xlatLimaTrigMode2PcoTrigMode(m_pcoData->bExtTrigEnabled);
    error = PcoCheckError(__LINE__, __FILE__, PCO_SetTriggerMode(m_handle, trigmode));
	if(error) return "PCO_SetTriggerMode";
	//PCO_THROW_OR_TRACE(error, "PCO_SetTriggerMode") ;
	//DEB_TRACE() << DEB_VAR1(trigmode);

    //------------------------------------- acquire mode : ignore or not ext. signal

	WORD acqmode = m_sync->xlatLimaTrigMode2PcoAcqMode();
	error = PcoCheckError(__LINE__, __FILE__, PCO_SetAcquireMode(m_handle, acqmode));
	if(error) return "PCO_SetAcquireMode";
   //PCO_THROW_OR_TRACE(error, "PCO_SetAcquireMode") ;
	return fnId;
}



//=================================================================================================
// ----------------------------------------- storage mode (recorder + sequence)
// current storage mode
//
// case RecSeq
// case RecRing
// - 0x0000 = [recorder] mode
//		. images are recorded and stored within the internal camera memory (camRAM)
//      . Live View transfers the most recent image to the PC (for viewing / monitoring)
//      . indexed or total image readout after the recording has been stopped
//
// case Fifo
// - 0x0001 = [FIFO buffer] mode
//      . all images taken are transferred to the PC in chronological order
//      . camera memory (camRAM) is used as huge FIFO buffer to bypass short bottlenecks in data transmission
//      . if buffer overflows, the oldest images are overwritten
//      . if Set Recorder = [stop] is sent, recording is stopped and the transfer of the current image to the PC is finished.
//      . Images not read are stored within the segment and can be read with the Read Image From Segment command.
//
// current recorder submode:
//
// case RecSeq
// - 0x0000 = [sequence]
//      . recording is stopped when the allocated buffer is full
//
// case RecRing
// - 0x0001 = [ring buffer].
//      . camera records continuously into ring buffer
//      . if the allocated buffer overflows, the oldest images are overwritten
//      . recording is stopped by software or disabling acquire signal (<acq enbl>)
//=================================================================================================
char * Camera::_pcoSet_Storage_subRecord_Mode(enumPcoStorageMode mode, int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;


	switch(mode) {
		case RecSeq:  m_pcoData->storage_mode = 0; m_pcoData->recorder_submode = 0; break;
		case RecRing: m_pcoData->storage_mode = 0; m_pcoData->recorder_submode = 1; break;
		case Fifo:    m_pcoData->storage_mode = 1; m_pcoData->recorder_submode = 0; break;
		default: 
			throw LIMA_HW_EXC(Error,"FATAL - invalid storage mode!" );
	}

    error = PcoCheckError(__LINE__, __FILE__, PCO_SetStorageMode(m_handle, m_pcoData->storage_mode));
	if(error) return "PCO_SetStorageMode";
    //PCO_THROW_OR_TRACE(error, "PCO_SetStorageMode") ;

    error = PcoCheckError(__LINE__, __FILE__, PCO_SetRecorderSubmode(m_handle, m_pcoData->recorder_submode));
	if(error) return "PCO_SetRecorderSubmode";
    //PCO_THROW_OR_TRACE(error, "PCO_SetRecorderSubmode") ;

	return fnId;
}


//=================================================================================================
//=================================================================================================

// 4294967295.0 = double(DWORD(0xFFFFFFFF)) 
#define DWORD_MAX_FLOAT 4294967295.0

#define	MAX_DWORD_MS (double(4294967295.0e-3))
#define	MAX_DWORD_US (double(4294967295.0e-6))
#define	MAX_DWORD_NS (double(4294967295.0e-9))


void _pco_time2dwbase(double exp_time, DWORD &dwExp, WORD &wBase) {
	// conversion time(s) to PCO standard DWORD + UNIT(ms, us, ns)
		// exp & lat time is saved in seconds (LIMA). 
		// PCO requires them expressed in DWORD as ms(base=2), us(base=1) or ns(base=0)
		// max DWORD 0xFFFFFFFF = 4294967295.0
		// find the lowest unit (ns -> us -> ms) which does not overflow DWORD
    
	if(exp_time <= MAX_DWORD_NS) {   
		dwExp = DWORD(exp_time * 1.0e9); wBase = 0; // ns
	} else 	if(exp_time <= MAX_DWORD_US) {  
		dwExp = DWORD(exp_time * 1.0e6); wBase = 1; // us
	} else {  
		dwExp = DWORD(exp_time * 1.0e3); wBase = 2; // ms
	}

	DWORD mask = 0x7;
	DWORD min = 0x1000;

	if(dwExp > min){
		dwExp |= mask;
		dwExp ^= mask;
	}

	return;
}


//=================================================================================================
//=================================================================================================
char* Camera::_pcoSet_Exposure_Delay_Time(int &error, int ph){
	DEB_MEMBER_FUNCT();
	DEF_FNID;
	bool doIt;


    DWORD dwExposure, dwDelay;
	WORD wExposure_base, wDelay_base;
    double _exposure, _delay;
    m_sync->getExpTime(_exposure);
    m_sync->getLatTime(_delay);
	double _delay0 = _delay;

	doIt = TRUE;
	
	if(ph != 0){ 
		doIt = FALSE;

		if((_isCameraType(Edge)) && (m_pcoData->dwPixelRate >= PCO_EDGE_PIXEL_RATE_HIGH) ) {
			double pixels = ((double) m_pcoData->wXResActual)* ((double) m_pcoData->wYResActual);
			double bytes = (m_pcoData->wLUT_Identifier == PCO_EDGE_LUT_SQRT) ? 1.5 : 2.0;
			double period = bytes * pixels / (m_pcoData->fTransferRateMHzMax * 1000000.);

			printf("--- %s>period[%g] -> cocRunTime[%g]\n", fnId, period , m_pcoData->cocRunTime);
			if(period > m_pcoData->cocRunTime) {
				_delay += period - m_pcoData->cocRunTime;
				doIt = TRUE;
				printf("--- %s> delay forced [%g] -> [%g]\n", fnId, _delay0, _delay);
			}
		}
	}

	if(!doIt) return fnId;

	_pco_time2dwbase(_exposure, dwExposure, wExposure_base);
	_pco_time2dwbase(_delay,  dwDelay, wDelay_base);

	error = PcoCheckError(__LINE__, __FILE__, PCO_SetDelayExposureTime(m_handle, dwDelay, dwExposure, wDelay_base, wExposure_base));

	if(error || _getDebug(DBG_EXP)) {
		DEB_ALWAYS() << DEB_VAR3(_exposure, dwExposure, wExposure_base);
		DEB_ALWAYS() << DEB_VAR3(_delay,  dwDelay, wDelay_base);
	}

	if(error) {
		return "PCO_SetDelayExposureTime";
	}

	return fnId;
}

//=================================================================================================
//=================================================================================================
char *Camera::_pcoSet_Cameralink_GigE_Parameters(int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	//------------------------------------------------- set image size for CamLink and GigE

	switch (m_pcoData->stcPcoCamType.wInterfaceType) {
        case INTERFACE_CAMERALINK:

            error = PcoCheckError(__LINE__, __FILE__, PCO_GetTransferParameter(m_handle, &m_pcoData->clTransferParam, sizeof(m_pcoData->clTransferParam)));
			if(error) return "PCO_GetTransferParameter";
            //PCO_THROW_OR_TRACE(error, "PCO_GetTransferParameter") ;


            if((m_pcoData->clTransferParam.baudrate != 115200) || (m_pcoData->clTransferParam.DataFormat != 2)) {
                m_pcoData->clTransferParam.baudrate=115200;
                m_pcoData->clTransferParam.DataFormat=2;

                error = PcoCheckError(__LINE__, __FILE__, PCO_SetTransferParameter(m_handle, &m_pcoData->clTransferParam, sizeof(m_pcoData->clTransferParam)));
				if(error) return "PCO_SetTransferParameter";
 				//PCO_THROW_OR_TRACE(error, "PCO_SetTransferParameter") ;
            }

            // ---- no break
            
        case INTERFACE_ETHERNET:
		    WORD wXres, wYres;

            wXres= m_pcoData->wXResActual;
            wYres= m_pcoData->wYResActual;
			error = PcoCheckError(__LINE__, __FILE__, PCO_CamLinkSetImageParameters(m_handle, wXres, wYres));
			if(error) return "PCO_CamLinkSetImageParameters";

        default: break;
    } // case

	return fnId;
}



//=================================================================================================
//=================================================================================================
char *Camera::_prepare_cameralink_interface(int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;
	bool bDoArm = FALSE;
	struct stcPcoData _pcoData;
	char msg[ERRMSG_SIZE + 1];

	
	//error = PcoCheckError(__LINE__, __FILE__, PCO_GetTransferParameter(m_handle, &m_pcoData->clTransferParam, sizeof(m_pcoData->clTransferParam)));
	error = PcoCheckError(__LINE__, __FILE__, PCO_GetTransferParameter(m_handle, &m_pcoData->clTransferParam, sizeof(PCO_SC2_CL_TRANSFER_PARAM)));
    PCO_THROW_OR_TRACE(error, "PCO_GetTransferParameter") ;
	memcpy(&_pcoData.clTransferParam, &m_pcoData->clTransferParam,sizeof(PCO_SC2_CL_TRANSFER_PARAM));
	
	m_pcoData->clTransferParam.baudrate = PCO_CL_BAUDRATE_115K2;

	if(_isCameraType(Dimax)){
			//m_pcoData->clTransferParam.Transmit = 1;
			//_pcoData.clTransferParam.Transmit = m_pcoData->clTransferParam.Transmit;
			m_pcoData->clTransferParam.DataFormat=PCO_CL_DATAFORMAT_2x12; //=2
	} else
	if(_isCameraType(Pco2k | Pco4k)){
		// -------------------- pco2k -> GigE		
	} else
	if(_isCameraType(EdgeGL)) {
		m_pcoData->clTransferParam.Transmit = 1;
		m_pcoData->clTransferParam.DataFormat=PCO_CL_DATAFORMAT_5x12 | 
			SCCMOS_FORMAT_TOP_CENTER_BOTTOM_CENTER;
			//SCCMOS_FORMAT_TOP_BOTTOM;
		m_pcoData->wLUT_Identifier = 0; //Switch LUT->off
	} else 
	if(_isCameraType(EdgeRolling)){
			m_pcoData->clTransferParam.Transmit = 1;

			if(m_pcoData->dwPixelRate <= PCO_EDGE_PIXEL_RATE_LOW){
				m_pcoData->clTransferParam.DataFormat=PCO_CL_DATAFORMAT_5x16 | 
					SCCMOS_FORMAT_TOP_CENTER_BOTTOM_CENTER;
				m_pcoData->wLUT_Identifier = PCO_EDGE_LUT_NONE; // Switch LUT->off
			} else 
			if( ((m_pcoData->dwPixelRate >= PCO_EDGE_PIXEL_RATE_HIGH) & 
					(m_pcoData->wXResActual > PCO_EDGE_WIDTH_HIGH))) {
				m_pcoData->clTransferParam.DataFormat=PCO_CL_DATAFORMAT_5x12L | 
					SCCMOS_FORMAT_TOP_CENTER_BOTTOM_CENTER;
				m_pcoData->wLUT_Identifier = PCO_EDGE_LUT_SQRT; //Switch LUT->sqrt
			} else {
				m_pcoData->clTransferParam.DataFormat = PCO_CL_DATAFORMAT_5x16 | 
					SCCMOS_FORMAT_TOP_CENTER_BOTTOM_CENTER;
				m_pcoData->wLUT_Identifier = PCO_EDGE_LUT_NONE; // Switch LUT->off
			}
	} else {
			sprintf_s(msg,ERRMSG_SIZE, "ERROR DEFAULT - pixelRate[%d] / width[%d]",
				m_pcoData->dwPixelRate, m_pcoData->wXResActual);
			throw LIMA_HW_EXC(Error, msg);
	}


	if((_pcoData.clTransferParam.baudrate != m_pcoData->clTransferParam.baudrate) ||
		(_pcoData.clTransferParam.DataFormat != m_pcoData->clTransferParam.DataFormat) ||
		(_pcoData.clTransferParam.Transmit != m_pcoData->clTransferParam.Transmit)	)
	{
		error = PcoCheckError(__LINE__, __FILE__, PCO_SetTransferParameter(m_handle, &m_pcoData->clTransferParam, sizeof(m_pcoData->clTransferParam)));
		if(error){
			sprintf_s(msg,ERRMSG_SIZE, "PCO_SetTransferParameter - baudrate[%d][%d] dataFormat[x%08x][x%08x] trasmit[%d][%d]",
				_pcoData.clTransferParam.baudrate, m_pcoData->clTransferParam.baudrate,
				_pcoData.clTransferParam.DataFormat, m_pcoData->clTransferParam.DataFormat,
				_pcoData.clTransferParam.Transmit, m_pcoData->clTransferParam.Transmit);
		} else msg[0]=0;
		PCO_THROW_OR_TRACE(error, msg) ;
		bDoArm = TRUE;
	}

	if(_isCameraType(Edge)) {
		WORD _wLUT_Identifier, _wLUT_Parameter;
		error = PcoCheckError(__LINE__, __FILE__, 
			PCO_GetActiveLookupTable(m_handle, &_wLUT_Identifier, &_wLUT_Parameter));
	    PCO_THROW_OR_TRACE(error, "PCO_GetActiveLookupTable") ;

		if(_wLUT_Identifier != m_pcoData->wLUT_Identifier) {
			bDoArm = TRUE;
			error = PcoCheckError(__LINE__, __FILE__, 
				PCO_SetActiveLookupTable(m_handle, &m_pcoData->wLUT_Identifier, &m_pcoData->wLUT_Parameter));
				PCO_THROW_OR_TRACE(error, "PCO_SetActiveLookupTable") ;

			error = PcoCheckError(__LINE__, __FILE__, 
				PCO_GetActiveLookupTable(m_handle, &m_pcoData->wLUT_Identifier, &m_pcoData->wLUT_Parameter));
				PCO_THROW_OR_TRACE(error, "PCO_GetActiveLookupTable") ;
		}
	}

	if(bDoArm) {
		error = PcoCheckError(__LINE__, __FILE__, PCO_ArmCamera(m_handle));
		PCO_THROW_OR_TRACE(error, "PCO_ArmCamera") ;
	}

	error = PcoCheckError(__LINE__, __FILE__, PCO_CamLinkSetImageParameters(m_handle, m_pcoData->wXResActual, m_pcoData->wYResActual));
    PCO_THROW_OR_TRACE(error, "PCO_CamLinkSetImageParameters") ;


	return fnId;
}


//=================================================================================================
//=================================================================================================
char *Camera::_pcoGet_Camera_Type(int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;
	char *msg;
	m_pcoData->frames_per_buffer = 1; // for PCO DIMAX

	// --- Get camera type
	{
		char *ptr;
		int errTot = 0;

		//m_pcoData->stcPcoCamType.wSize= sizeof(m_pcoData->stcPcoCamType);
		
		error = PcoCheckError(__LINE__, __FILE__, PCO_GetCameraType(m_handle, &m_pcoData->stcPcoCamType));
		msg = "PCO_GetCameraType";
		PCO_PRINT_ERR(error, msg); 	if(error) return msg;

		ptr = xlatPcoCode2Str(_getCameraType(), ModelType, error);
		strcpy_s(m_pcoData->model, MODEL_TYPE_SIZE, ptr);
		errTot |= error;

		ptr = xlatPcoCode2Str(m_pcoData->stcPcoCamType.wInterfaceType, InterfaceType, error);
		strcpy_s(m_pcoData->iface, INTERFACE_TYPE_SIZE, ptr);
		errTot |= error;

		sprintf_s(m_pcoData->camera_name, CAMERA_NAME_SIZE, "%s %s", m_pcoData->model, m_pcoData->iface);
		DEB_ALWAYS() <<  DEB_VAR3(m_pcoData->model, m_pcoData->iface, m_pcoData->camera_name);

		if(errTot) return m_pcoData->camera_name;

	}

	// -- Reset to default settings

	error = PcoCheckError(__LINE__, __FILE__, PCO_SetRecordingState(m_handle, 0));
	msg = "PCO_SetRecordingState";
	if(error) return msg;


	error = PcoCheckError(__LINE__, __FILE__, PCO_ResetSettingsToDefault(m_handle));
	msg = "PCO_ResetSettingsToDefault";
	PCO_PRINT_ERR(error, msg); 	if(error) return msg;
	

	// -- Get camera description
	//m_pcoData->stcPcoDescription.wSize= sizeof(m_pcoData->stcPcoDescription);

	error = PcoCheckError(__LINE__, __FILE__, PCO_GetCameraDescription(m_handle, &m_pcoData->stcPcoDescription));
	msg = "PCO_GetCameraDescription";
	PCO_PRINT_ERR(error, msg); 	if(error) return msg;

	m_pcoData->dwPixelRateMax = 0;
	for(int i=0; i<4; i++) {
		if(m_pcoData->dwPixelRateMax < m_pcoData->stcPcoDescription.dwPixelRateDESC[i])
					m_pcoData->dwPixelRateMax = m_pcoData->stcPcoDescription.dwPixelRateDESC[i];
	}	
	

	m_pcoData->bMetaDataAllowed = !!(m_pcoData->stcPcoDescription.dwGeneralCapsDESC1 & GENERALCAPS1_METADATA) ;


	// -- Get General
	//m_pcoData->stcPcoGeneral.wSize= sizeof(m_pcoData->stcPcoGeneral);
	//m_pcoData->stcPcoGeneral.strCamType.wSize= sizeof(m_pcoData->stcPcoGeneral.strCamType);

	error = PcoCheckError(__LINE__, __FILE__, PCO_GetGeneral(m_handle, &m_pcoData->stcPcoGeneral));
	msg = "PCO_GetGeneral";
	PCO_PRINT_ERR(error, msg); 	if(error) return msg;


	// -- Get Sensor struct
	//m_pcoData->stcPcoSensor.wSize= sizeof(m_pcoData->stcPcoSensor);
	//m_pcoData->stcPcoSensor.strDescription.wSize= sizeof(m_pcoData->stcPcoSensor.strDescription);
	//m_pcoData->stcPcoSensor.strDescription2.wSize= sizeof(m_pcoData->stcPcoSensor.strDescription2);

	error = PcoCheckError(__LINE__, __FILE__, PCO_GetSensorStruct(m_handle, &m_pcoData->stcPcoSensor));
	msg = "PCO_GetSensorStruct";
	PCO_PRINT_ERR(error, msg); 	if(error) return msg;
\
	// -- Get timing struct
	//m_pcoData->stcPcoTiming.wSize= sizeof(m_pcoData->stcPcoTiming);

	error = PcoCheckError(__LINE__, __FILE__, PCO_GetTimingStruct(m_handle, &m_pcoData->stcPcoTiming));
	msg = "PCO_GetTimingStruct";
	PCO_PRINT_ERR(error, msg); 	if(error) return msg;


	// -- Get recording struct
	//m_pcoData->stcPcoRecording.wSize= sizeof(m_pcoData->stcPcoRecording);

	error = PcoCheckError(__LINE__, __FILE__, PCO_GetRecordingStruct(m_handle, &m_pcoData->stcPcoRecording));
	msg = "PCO_GetRecordingStruct";
	PCO_PRINT_ERR(error, msg); 	if(error) return msg;


	// -- Get storage struct
	//m_pcoData->stcPcoStorage.wSize= sizeof(m_pcoData->stcPcoStorage);

	error = PcoCheckError(__LINE__, __FILE__, PCO_GetStorageStruct(m_handle, &m_pcoData->stcPcoStorage));
	msg = "PCO_GetStorageStruct";
	PCO_PRINT_ERR(error, msg); 	if(error) return msg;



	return fnId;
}



//=================================================================================================
//=================================================================================================
char *Camera::_pcoGet_TemperatureInfo(int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;
	char msg[MSG_SIZE + 1];



	// -- Print out current temperatures
	error = PcoCheckError(__LINE__, __FILE__, PCO_GetTemperature(m_handle, &m_pcoData->temperature.wCcd, &m_pcoData->temperature.wCam, &m_pcoData->temperature.wPower));
	if(error) return "PCO_GetTemperature";
	//PCO_THROW_OR_TRACE(error, "PCO_GetTemperature") ;

	sprintf_s(msg, MSG_SIZE, "* temperature: CCD[%.1f]  CAM[%d]  PS[%d]\n", m_pcoData->temperature.wCcd/10., m_pcoData->temperature.wCam, m_pcoData->temperature.wPower);
	//DEB_TRACE() <<   msg;
	m_log.append(msg);


	m_pcoData->temperature.wMinCoolSet = m_pcoData->stcPcoDescription.sMinCoolSetDESC;
	m_pcoData->temperature.wMaxCoolSet = m_pcoData->stcPcoDescription.sMaxCoolSetDESC;

	sprintf_s(msg, MSG_SIZE, "* cooling temperature: MIN [%d]  Max [%d]\n",  m_pcoData->temperature.wMinCoolSet, m_pcoData->temperature.wMaxCoolSet);
	//DEB_TRACE() <<   msg;
	m_log.append(msg);

	// -- Set/Get cooling temperature
	if (m_pcoData->temperature.wSetpoint != -1) {
		if (m_pcoData->temperature.wSetpoint < m_pcoData->temperature.wMinCoolSet)	m_pcoData->temperature.wSetpoint = m_pcoData->temperature.wMinCoolSet;
		if (m_pcoData->temperature.wSetpoint > m_pcoData->temperature.wMaxCoolSet)	m_pcoData->temperature.wSetpoint= m_pcoData->temperature.wMaxCoolSet;
	} else {
		error = PcoCheckError(__LINE__, __FILE__, PCO_GetCoolingSetpointTemperature(m_handle, &m_pcoData->temperature.wSetpoint));
		if(error) return "PCO_GetCoolingSetpointTemperature";
		//PCO_THROW_OR_TRACE(error, "PCO_GetCoolingSetpointTemperature") ;
	}
	sprintf_s(msg, MSG_SIZE, "* Cooling Setpoint = %d\n", m_pcoData->temperature.wSetpoint);
	//DEB_TRACE() <<   msg;
	m_log.append(msg);


	return fnId;
}


//=================================================================================================
//=================================================================================================
char * Camera::_pcoSet_RecordingState(int state, int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;
	char *msg;


	WORD wRecState_new, wRecState_actual;

	wRecState_new = state ? 0x0001 : 0x0000 ; // 0x0001 => START acquisition

	error = PcoCheckError(__LINE__, __FILE__, PCO_GetRecordingState(m_handle, &wRecState_actual));
	msg = "PCO_GetRecordingState";
	PCO_PRINT_ERR(error, msg); 	if(error) return msg;

	//if(wRecState_new == wRecState_actual) {error = 0; return fnId; }

	error = PcoCheckError(__LINE__, __FILE__, PCO_SetRecordingState(m_handle, wRecState_new));
	msg = "PCO_SetRecordingState";
	PCO_PRINT_ERR(error, msg); 	if(error) return msg;

	if(wRecState_new == 0) {
		error = PcoCheckError(__LINE__, __FILE__, PCO_CancelImages(m_handle));
		msg = "PCO_CancelImages";
		PCO_PRINT_ERR(error, msg); 	if(error) return msg;
	}

	DEB_ALWAYS() <<  DEB_VAR4(error, state, wRecState_actual, wRecState_new);
	return fnId;

}

//=================================================================================================
//=================================================================================================
int Camera::dumpRecordedImages(int &nrImages, int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;
	char *msg;

	HANDLE m_handle = getHandle();
	WORD wSegment = pcoGetActiveRamSegment(); 
	DWORD _dwValidImageCnt, _dwMaxImageCnt;


	WORD wRecState_actual;

	nrImages = -1;

	if(!_isCameraType(Dimax)) return -2;

	error = PcoCheckError(__LINE__, __FILE__, PCO_GetRecordingState(m_handle, &wRecState_actual));
	msg = "PCO_GetRecordingState";
	PCO_PRINT_ERR(error, msg); 	
	
	if (error) return -100;
	if(wRecState_actual != 0) return -1;


	msg = _PcoCheckError(__LINE__, __FILE__, PCO_GetNumberOfImagesInSegment(m_handle, wSegment, &_dwValidImageCnt, &_dwMaxImageCnt), error);
	if(error) {
		printf("=== %s [%d]> ERROR %s\n", fnId, __LINE__, msg);
		throw LIMA_HW_EXC(Error, "PCO_GetNumberOfImagesInSegment");
	}

	nrImages = _dwValidImageCnt;

	return 0;

}


//=================================================================================================
//=================================================================================================
char *Camera::_get_coc_runtime(int &error){
		
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	//====================================== get the coc runtime 
    //---- only valid if it was used PCO_SetDelayExposureTime
	//---- and AFTER armed the cam

	// Get and split the 'camera operation code' runtime into two DWORD. One will hold the longer
	// part, in seconds, and the other will hold the shorter part, in nanoseconds. This function can be
	// used to calculate the FPS. The sum of dwTime_s and dwTime_ns covers the delay, exposure and
	// readout time. If external exposure is active, it returns only the readout time.

	DWORD dwTime_s, dwTime_ns;
    double runTime;

    error = PcoCheckError(__LINE__, __FILE__, PCO_GetCOCRuntime(m_handle, &dwTime_s, &dwTime_ns));
	char *msg = "PCO_GetCOCRuntime";
	PCO_PRINT_ERR(error, msg); 	if(error) return msg;

    m_pcoData->cocRunTime = runTime = ((double) dwTime_ns * 1.0E-9) + (double) dwTime_s;
    m_pcoData->frameRate = (dwTime_ns | dwTime_s) ? 1.0 / runTime : 0.0;

    DEB_TRACE() << DEB_VAR2(m_pcoData->frameRate, m_pcoData->cocRunTime);

	return fnId;

}

//=================================================================================================
//=================================================================================================
char *Camera::_set_metadata_mode(WORD wMetaDataMode, int &error){
		
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	m_pcoData->wMetaDataSize = m_pcoData->wMetaDataVersion = 0;
	if(_isCameraType(Dimax)) {
		m_pcoData->wMetaDataMode = wMetaDataMode;
		error = PcoCheckError(__LINE__, __FILE__, 
			PCO_SetMetaDataMode(m_handle, wMetaDataMode, &m_pcoData->wMetaDataSize, &m_pcoData->wMetaDataVersion));
		char *msg ="PCO_SetMetaDataMode";
		PCO_PRINT_ERR(error, msg); 	if(error) return msg;
	}
	return fnId;
}





void Camera::_set_shutter_rolling_edge(bool rolling, int &error){
		
	DEB_MEMBER_FUNCT();


	error = 0;

	if(!_isCameraType(Edge)) {
		error = -1;
		return ;
	}

	m_pcoData->bRollingShutter = rolling;


	_beginthread( _pco_shutter_thread_edge, 0, (void*) this);

	return;

}

//=================================================================================================
//=================================================================================================
void Camera::_pco_set_shutter_rolling_edge(int &error){
		
	DEB_MEMBER_FUNCT();
	DEF_FNID;
	char *msg;

	DWORD _dwSetup;
	DWORD m_dwSetup[10];
	WORD m_wLen = 10;
	WORD m_wType;
	int ts[3] = { 2000, 3000, 250}; // command, image, channel timeout


	if(!_isCameraType(Edge)) {
		return ;
	}

	m_config = TRUE;


	// DWORD m_dwSetup[10];
	// WORD m_wLen = 10;
	// WORD m_wType;
	// int ts[3] = { 2000, 3000, 250}; // command, image, channel timeout
	// PCO_OpenCamera(&m_hCam,0);
	// PCO_GetCameraSetup(m_hCam, &m_wType, &m_dwSetup[0], &m_wLen);

	// m_dwSetup[0] = PCO_EDGE_SETUP_GLOBAL_SHUTTER;
	// PCO_SetTimeouts(m_hCam, &ts[0], sizeof(ts));
	// PCO_SetCameraSetup(m_hCam, m_wType, &m_dwSetup[0], m_wLen);
	// PCO_RebootCamera(m_hCam);
	// PCO_CloseCamera(m_hCam);
	// Camera setup parameter for pco.edge:
	// #define PCO_EDGE_SETUP_ROLLING_SHUTTER 0x00000001         // rolling shutter
	// #define PCO_EDGE_SETUP_GLOBAL_SHUTTER  0x00000002         // global shutter

	
	_dwSetup = m_pcoData->bRollingShutter ? PCO_EDGE_SETUP_ROLLING_SHUTTER : PCO_EDGE_SETUP_GLOBAL_SHUTTER;

    error = PcoCheckError(__LINE__, __FILE__, PCO_GetCameraSetup(m_handle, &m_wType, &m_dwSetup[0], &m_wLen));
	msg = "PCO_GetCameraSetup";
	PCO_PRINT_ERR(error, msg); 	if(error) return;

	if(m_dwSetup[0] == _dwSetup) { m_config = FALSE;return;}

	m_dwSetup[0] = _dwSetup;

    error = PcoCheckError(__LINE__, __FILE__, PCO_SetTimeouts(m_handle, &ts[0], sizeof(ts)));
	msg = "PCO_SetTimeouts";
	PCO_PRINT_ERR(error, msg); 	if(error) return;

    error = PcoCheckError(__LINE__, __FILE__, PCO_SetCameraSetup(m_handle, m_wType, &m_dwSetup[0], m_wLen));
	msg = "PCO_SetCameraSetup";
	PCO_PRINT_ERR(error, msg); 	if(error) return;

    error = PcoCheckError(__LINE__, __FILE__, PCO_RebootCamera(m_handle));
	msg = "PCO_RebootCamera";
	PCO_PRINT_ERR(error, msg); 	if(error) return;

	m_sync->_getBufferCtrlObj()->_pcoAllocBuffersFree();

    error = PcoCheckError(__LINE__, __FILE__, PCO_CloseCamera(m_handle));
	msg = "PCO_CloseCamera";
	PCO_PRINT_ERR(error, msg); 	if(error) return;

	::Sleep(PCO_EDGE_SLEEP_SHUTTER_MS);

	_init();

	m_config = FALSE;
	return;

}

//=================================================================================================
//=================================================================================================
bool Camera::_get_shutter_rolling_edge(int &error){
		
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	DWORD m_dwSetup[10];
	WORD m_wLen = 10;
	WORD m_wType;
	char *msg;


    error = PcoCheckError(__LINE__, __FILE__, PCO_GetCameraSetup(m_handle, &m_wType, &m_dwSetup[0], &m_wLen));
	msg = "PCO_GetCameraSetup";
	PCO_PRINT_ERR(error, msg); 	if(error) return FALSE;

	return (m_dwSetup[0] == PCO_EDGE_SETUP_ROLLING_SHUTTER);

}
//=================================================================================================
//=================================================================================================
bool Camera::_isValid_pixelRate(DWORD dwPixelRate){
		
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	
	if(dwPixelRate > 0) 
		for(int i = 0; i < 4; i++) {			
			if(dwPixelRate == m_pcoData->stcPcoDescription.dwPixelRateDESC[i]) return TRUE;
		}

	return FALSE;
}



//=================================================================================================
//=================================================================================================
bool Camera::_isValid_Roi(const Roi &new_roi, Roi &fixed_roi){
		
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	int diffx0, diffx1, diffy0, diffy1 ;
	bool fixed;
	int xFix0, xFix1, yFix0, yFix1;
	int x0, x1, y0, y1;

	int xMax = m_pcoData->stcPcoDescription.wMaxHorzResStdDESC;
	int yMax = m_pcoData->stcPcoDescription.wMaxVertResStdDESC;
	int xSteps = m_pcoData->stcPcoDescription.wRoiHorStepsDESC;
	int ySteps = m_pcoData->stcPcoDescription.wRoiVertStepsDESC;

	xFix0 = x0 = new_roi.getTopLeft().x+1;
	xFix1 = x1 = new_roi.getBottomRight().x+1;
	yFix0 = y0 = new_roi.getTopLeft().y+1;
	yFix1 = y1 = new_roi.getBottomRight().y+1;

	// pco roi [1,2048]

	fixed = false;
	if (xFix0 < 1) {xFix0 = 1; fixed = true;}
	if (xFix1 > xMax) {xFix1 = xMax; fixed = true;}
	if (xFix0 > xFix1) {xFix0 = 1; xFix1 = xMax; fixed = true;}

	if ((diffx0 = ((xFix0 - 1) % xSteps)  ) != 0 ) {xFix0 -= diffx0; fixed = true;}
	if ((diffx1 = ((xFix1) % xSteps)) != 0 ) {xFix1 += xSteps - diffx1; fixed = true;}

	if (yFix0 < 1) {yFix0 = 1; fixed = true;}
	if (yFix1 > yMax) {yFix1 = yMax; fixed = true;}
	if (yFix0 > yFix1) {yFix0 = 1; yFix1 = yMax; fixed = true;}

	if ((diffy0 = ((yFix0 - 1) % ySteps)) != 0 ) {yFix0 -= diffy0; fixed = true;}
	if ((diffy1 = ((yFix1) % ySteps)) != 0 ) {yFix1 += ySteps - diffy1; fixed = true;;}


	fixed_roi.setTopLeft(Point(xFix0-1, yFix0-1));
	fixed_roi.setSize(Size(xFix1 -xFix0+1, yFix1-yFix0+1));

	if(_getDebug(DBG_ROI)) {
		DEB_ALWAYS()  << "REQUESTED roiX " << DEB_VAR4(x0, x1, xSteps, xMax)   << " roiY " 
			<< DEB_VAR4(y0, y1, ySteps, yMax) ;
		if(fixed) {
			DEB_ALWAYS()  << "FIXED roiXY "  << DEB_VAR4(xFix0, xFix1, yFix0, yFix1)    << " DIFF " 
				<< DEB_VAR4(diffx0, diffx1, diffy0, diffy1) ;
		}
	}

	return !fixed ;

}


//=================================================================================================
//=================================================================================================
void Camera::_set_Roi(const Roi &new_roi, int &error){
	
	Size roi_size;

	DEB_MEMBER_FUNCT();
	DEF_FNID;

	Roi fixed_roi;

	if(!_isValid_Roi(new_roi, fixed_roi)){
		error = -1;
		return;
	}

	    // pco roi [1,max] ---- lima Roi [0, max-1]

		m_roi.x[0] = fixed_roi.getTopLeft().x+1;
		m_roi.x[1] = fixed_roi.getBottomRight().x+1;
		m_roi.y[0] = fixed_roi.getTopLeft().y+1;
		m_roi.y[1] = fixed_roi.getBottomRight().y+1;
		m_roi.changed = Changed;

		m_RoiLima = fixed_roi;

	if(_getDebug(DBG_ROI)) {
		DEB_ALWAYS() << DEB_VAR1(m_RoiLima);
	}	
		
	error = 0;
	return ;
}



//=================================================================================================
//=================================================================================================

void Camera::_get_Roi(Roi &roi){
		
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	roi = m_RoiLima;

	roi.setTopLeft(Point(m_roi.x[0]-1, m_roi.y[0]-1));
	roi.setSize(Size(m_roi.x[1]-m_roi.x[0]+1, m_roi.y[1]-m_roi.y[0]+1));

	
}

void Camera::_get_MaxRoi(Roi &roi){
		
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	roi.setTopLeft(Point(0, 0));
	roi.setSize(Size(m_pcoData->maxWidth, m_pcoData->maxHeight));
}


//=========================================================================================================
//=========================================================================================================
void Camera::_get_RoiSize(Size& roi_size)
{
	int error, width, height;

	roi_size = m_RoiLima.getSize();

	width = m_roi.x[1] - m_roi.x[0] +1;
	height = m_roi.y[1] - m_roi.y[0] +1;

	roi_size = Size(int(width),int(height));

	error = 0;
}

//=========================================================================================================
//=========================================================================================================
void Camera::_get_ImageType(ImageType& image_type)
{
  unsigned int pixbytes;
  getBytesPerPixel(pixbytes);
  image_type = (pixbytes == 2) ? Bpp16 : Bpp8;
}

//=================================================================================================
//=================================================================================================
void Camera::_get_PixelSize(double& x_size,double &y_size)
{  
    // ---- TODO
	// pixel size in micrometer (???)

	if( _isCameraType(Pco4k)) {
		x_size = y_size = 9.0;		
		return;
	}
  x_size = y_size = -1.;		// @todo don't know

}

//=================================================================================================
//=================================================================================================
void Camera::_set_ImageType(ImageType curr_image_type)
{
    // ---- DONE
	// only check if it valid, BUT don't set it ????
  switch(curr_image_type)
    {
    case Bpp16:
    case Bpp8:
      break;

    default:
      throw LIMA_HW_EXC(InvalidValue,"This image type is not Managed");
    }

}
//=========================================================================================================
//=========================================================================================================
void Camera::_get_DetectorType(std::string& det_type)
{
    // ---- DONE
   det_type = "Pco";
}

//=========================================================================================================
//=========================================================================================================
void Camera::_get_MaxImageSize(Size& max_image_size)
{

  // ---- DONE
  DWORD width,height;

  getMaxWidthHeight(width,height);
  max_image_size = Size(int(width),int(height));

}

//=================================================================================================
//=================================================================================================
bool Camera::_isCameraType(int tp){
		
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	switch(_getCameraType()) {
		case CAMERATYPE_PCO_DIMAX_STD: 
			return !!(tp & Dimax) ;
		
		case CAMERATYPE_PCO_EDGE_GL:
			return !!(tp & (EdgeGL | Edge));

		case CAMERATYPE_PCO_EDGE:
			return !!(tp & (EdgeRolling | Edge));

		case CAMERATYPE_PCO2000:
			return !!(tp & Pco2k) ;

		case CAMERATYPE_PCO4000:
			return !!(tp & Pco4k) ;


		default:
			return FALSE;

	}
		
}

//=================================================================================================
//=================================================================================================
void Camera::_pco_GetPixelRate(DWORD &pixRate, int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;
#if 0
		if(!_isCameraType(Edge)) {
			pixRate = 0;
			error = -1;
			return;
		}
#endif

		error = PcoCheckError(__LINE__, __FILE__, PCO_GetPixelRate(m_handle, &m_pcoData->dwPixelRate));
	    PCO_THROW_OR_TRACE(error, "PCO_GetPixelRate") ;

		pixRate = m_pcoData->dwPixelRate;
}



//=================================================================================================
//=================================================================================================
void Camera::_pco_GetHWIOSignal(int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	int i, imax;
		error = 0;

		if(!( _isCameraType(Dimax | Edge | Pco2k | Pco4k))  ) {
			error = -1;
			return;
		}

		error |= PcoCheckError(__LINE__, __FILE__, PCO_GetHWIOSignalCount(m_handle, &m_pcoData->wNrPcoHWIOSignal0));

		imax = m_pcoData->wNrPcoHWIOSignal = 
			(m_pcoData->wNrPcoHWIOSignal0 <= SIZEARR_stcPcoHWIOSignal) ? m_pcoData->wNrPcoHWIOSignal0 : SIZEARR_stcPcoHWIOSignal;

		//DEB_ALWAYS()  << "--- size" << DEB_VAR3(imax, m_pcoData->wNrPcoHWIOSignal0 , m_pcoData->wNrPcoHWIOSignal ) ;

		for(i=0; i< imax; i++) {
			//DEB_ALWAYS()  << "---  descriptor" << DEB_VAR2(i, m_pcoData->stcPcoHWIOSignalDesc[i].wSize) ;
			error |= PcoCheckError(__LINE__, __FILE__, PCO_GetHWIOSignalDescriptor(m_handle, i, &m_pcoData->stcPcoHWIOSignalDesc[i]));
			//DEB_ALWAYS()  << "---  signal" << DEB_VAR2(i, m_pcoData->stcPcoHWIOSignal[i].wSize) ;
			error |= PcoCheckError(__LINE__, __FILE__, PCO_GetHWIOSignal(m_handle, i, &m_pcoData->stcPcoHWIOSignal[i]));
		}

}


//=================================================================================================
//=================================================================================================

/**************************************************************************************************
	name[Acquire Enable] idx[0] num[0]
	-def:     def[0x1] type[0xf] pol[0x3] filt[0x7]
	-sig:    enab[0x1] type[0x1] pol[0x1] filt[0x1] sel[0x0]

	name[Exposure Trigger] idx[1] num[1]
	-def:     def[0x1] type[0xf] pol[0xc] filt[0x7]
	-sig:    enab[0x1] type[0x1] pol[0x4] filt[0x1] sel[0x0]

	name[Status Expos] idx[2] num[2]
	-def:     def[0x3] type[0x1] pol[0x3] filt[0x0]
	-sig:    enab[0x1] type[0x1] pol[0x1] filt[0x0] sel[0x0]

	name[Ready Status] idx[3] num[3]
	-def:     def[0x3] type[0x1] pol[0x3] filt[0x0]
	-sig:    enab[0x1] type[0x1] pol[0x1] filt[0x0] sel[0x0]

	name[Set Ready] idx[4] num[4]
	-def:     def[0x1] type[0xf] pol[0x3] filt[0x7]
	-sig:    enab[0x1] type[0x1] pol[0x1] filt[0x1] sel[0x0]
**************************************************************************************************/


void Camera::_pco_initHWIOSignal(int mode, int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	int  _err, idx;
	error = 0;
	char *name;
	WORD val;


	if(!( _isCameraType(Dimax |Edge))  ) {
		error = -1;
		return;
	}

	_pco_GetHWIOSignal(_err); error |= _err;

	//	name[Acquire Enable] idx[0] num[0]
	idx = 0; val = 2;
	name = m_pcoData->stcPcoHWIOSignalDesc[idx].strSignalName[0];
	m_pcoData->stcPcoHWIOSignal[idx].wPolarity = 2;

	_pco_SetHWIOSignal(idx, _err); error |= _err;

	DEB_ALWAYS() << "set PcoHWIOSignal polarity "  << DEB_VAR3(name, idx, val) ;


}



//=================================================================================================
//=================================================================================================
void Camera::_pco_SetHWIOSignal(int sigNum, int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;


		if(!( _isCameraType(Dimax |Edge))  || 
			(sigNum < 0) || (sigNum >= m_pcoData->wNrPcoHWIOSignal) ) {
			error = -1;
			return;
		}

		error = PcoCheckError(__LINE__, __FILE__, PCO_SetHWIOSignal(m_handle, sigNum, &m_pcoData->stcPcoHWIOSignal[sigNum]));

}


//=================================================================================================
//=================================================================================================
void Camera::_get_XYsteps(Point &xy_steps){
	DEB_MEMBER_FUNCT();
	DEF_FNID;

		xy_steps.x = m_pcoData->maxwidth_step;
		xy_steps.y = m_pcoData->maxheight_step;
}

//=================================================================================================
//=================================================================================================
void Camera::_presetPixelRate(DWORD &pixRate, int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;
		if(!_isCameraType(Edge) || !_isValid_pixelRate(pixRate)) {
			pixRate = 0;
			error = -1;
			return;
		}

		m_pcoData->dwPixelRateRequested = pixRate;
		error = 0;
}


//=================================================================================================
//=================================================================================================
void Camera::msgLog(char *s) {
	m_msgLog->add(s); 
}



