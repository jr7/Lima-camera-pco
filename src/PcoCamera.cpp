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

//#define BUFF_INFO_SIZE 5000

#define DWORD_MAX ULONG_MAX 

#include <cstdlib>
#include <process.h>

#include <sys/timeb.h>
#include <time.h>

#include "Exceptions.h"

#include "PcoCamera.h"
#include "PcoSyncCtrlObj.h"
#include "PcoBufferCtrlObj.h"
#include "PcoCameraUtils.h"

using namespace lima;
using namespace lima::Pco;

static char *timebaseUnits[] = {"ns", "us", "ms"};


void _pco_acq_thread_dimax(void *argin);
void _pco_acq_thread_dimax_fifo(void *argin);
void _pco_acq_thread_edge(void *argin);
void _pco_shutter_thread_edge(void *argin);

//=========================================================================================================
char* _timestamp_pcocamera() {return ID_TIMESTAMP ;}
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
char * _timestamp_pcosyncctrlobj();
char * _timestamp_pcointerface();
char * _timestamp_pcobufferctrlobj();
char * _timestamp_pcodetinfoctrlobj();
char * _timestamp_pcocamerautils();

stcPcoData::stcPcoData(){

	char *ptr, *ptrMax;

	ptr = version; *ptr = 0;
	ptrMax = ptr + sizeof(version) - 1;

	ptr += sprintf_s(ptr, ptrMax - ptr, "\n");
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcocamera());
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcosyncctrlobj());
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcointerface());
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcobufferctrlobj());
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcodetinfoctrlobj());
	ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcocamerautils());
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

	m_pcoData =new(stcPcoData);
	if(m_pcoData == NULL)
		throw LIMA_HW_EXC(Error, "creation error");
	//memset((char *)m_pcoData, 0, sizeof(stcPcoData));
    DEB_ALWAYS()  << DEB_VAR1(m_pcoData->version) ;

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
	error = PcoCheckError(PCO_OpenCamera(&m_handle, 0));
	PCO_THROW_OR_TRACE(error, "PCO_OpenCamera") ;

	errMsg = _pcoGet_Camera_Type(error);
	PCO_THROW_OR_TRACE(error, errMsg) ;

		// -- Initialise adc, size, bin, roi
	m_pcoData->nr_adc= 1;
	m_pcoData->max_adc = m_pcoData->pcoInfo.wNumADCsDESC;

	m_pcoData->maxWidth = (unsigned int) m_pcoData->pcoInfo.wMaxHorzResStdDESC; // ds->ccd.size.xmax,
	m_pcoData->maxHeight= (unsigned int) m_pcoData->pcoInfo.wMaxVertResStdDESC; // ds->ccd.size.ymax,
	m_pcoData->bitsPerPix = (unsigned int) m_pcoData->pcoInfo.wDynResDESC; // ds->ccd.size.bits
	m_pcoData->bytesPerPix = (m_pcoData->bitsPerPix <= 8)?1:2; // nr de bytes por pixel  12 bits -> 2 bytes


	m_pcoData->maxwidth_step= (unsigned int) m_pcoData->pcoInfo.wRoiHorStepsDESC;   // ds->ccd.roi.xstep
	m_pcoData->maxheight_step= (unsigned int) m_pcoData->pcoInfo.wRoiVertStepsDESC; // ds->ccd.roi.ystep,

	m_roi.x[0] = m_roi.y[0] = 1;
	m_roi.x[1] = m_pcoData->maxWidth;
	m_roi.y[1] = m_pcoData->maxHeight;
	m_roi.changed = Changed;

	sprintf_s(msg, MSG_SIZE, "* CCD Size = X[%d] x Y[%d] (%d bits)\n", m_pcoData->maxWidth, m_pcoData->maxHeight, m_pcoData->bitsPerPix);
	DEB_TRACE() <<   msg;
	m_log.append(msg);
	
	sprintf_s(msg, MSG_SIZE, "* ROI Steps = x:%d, y:%d\n", m_pcoData->maxwidth_step, m_pcoData->maxheight_step);
	DEB_TRACE() <<   msg;
	m_log.append(msg);

	errMsg = _pcoGet_TemperatureInfo(error);
	PCO_THROW_OR_TRACE(error, errMsg) ;

	_pcoSet_RecordingState(0, error);

	if(_isCameraType(Dimax)) _init_dimax();
	else if(_isCameraType(Edge)) _init_edge();
	else {
		char msg[MSG_SIZE+1];
		sprintf_s(msg, MSG_SIZE, "Camera type not supported! [x%04x]", _getCameraType());
		throw LIMA_HW_EXC(Error, msg);
	}


	m_cam_connected = true;
	error = 0;

	if(!m_cam_connected)
		throw LIMA_HW_EXC(Error, "Camera not found!");


  DEB_TRACE() << m_log;
  DEB_TRACE() << "END OF CAMERA";

}

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
		
		error = PcoCheckError(PCO_GetCameraRamSize(m_handle, &ramSize, &pageSize));
		PCO_THROW_OR_TRACE(error, "PCO_GetCameraRamSize") ;

		m_pcoData->dwRamSize = ramSize;     // nr of pages of the ram
		m_pcoData->wPixPerPage = pageSize;    // nr of pixels of the page

		sprintf_s(msg, MSG_SIZE, "* RAM number of pages [%ld]  PAGE number of pixels [%d]\n",  
				m_pcoData->dwRamSize, m_pcoData->wPixPerPage);
		DEB_TRACE() <<   msg;
		m_log.append(msg);


		// ----------------- get initial seg Size - images & print

		// ---- get the size in pages of each of the 4 segments

		DWORD   segSize[4];
		error = PcoCheckError(PCO_GetCameraRamSegmentSize(m_handle, segSize));
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

			error = PcoCheckError(PCO_GetNumberOfImagesInSegment(m_handle, segmentPco, &_dwValidImageCnt, &_dwMaxImageCnt));
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


		error = PcoCheckError(PCO_SetCameraRamSegmentSize(m_handle, &m_pcoData->dwSegmentSize[0]));
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

		error = PcoCheckError(PCO_GetActiveRamSegment(m_handle, &m_pcoData->activeRamSegment));
		PCO_THROW_OR_TRACE(error, "PCO_GetActiveRamSegment") ;

		error = PcoCheckError(PCO_GetNumberOfImagesInSegment(m_handle, m_pcoData->activeRamSegment, &_dwValidImageCnt, &_dwMaxImageCnt));
		PCO_THROW_OR_TRACE(error, "PCO_GetNumberOfImagesInSegment") ;



	DEB_TRACE() <<  "original DONE";


}


void Camera::_init_edge() {

	m_pcoData->fTransferRateMHzMax = 550.;



}



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
	error = PcoCheckError(PCO_CloseCamera(m_handle));
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
        error = PcoCheckError(PCO_SetBinning(m_handle, wBinHorz, wBinVert));
        PCO_THROW_OR_TRACE(error, "PCO_SetBinning") ;
        m_bin.changed= Valid;
    }

    error = PcoCheckError(PCO_GetBinning(m_handle, &wBinHorz, &wBinVert));
    PCO_THROW_OR_TRACE(error, "PCO_GetBinning") ;
	DEB_TRACE() << DEB_VAR2(wBinHorz, wBinVert);

    //------------------------------------------------- set roi if needed
    WORD wRoiX0, wRoiY0; // Roi upper left x y
    WORD wRoiX1, wRoiY1; // Roi lower right x y

    if(m_roi.changed == Valid) m_roi.changed = Changed;    //+++++++++ TEST / FORCE WRITE ROI
    if (m_roi.changed == Changed) {
        wRoiX0 = (WORD)m_roi.x[0]; wRoiX1 = (WORD)m_roi.x[1];
        wRoiY0 = (WORD)m_roi.y[0]; wRoiY1 = (WORD)m_roi.y[1];

		DEB_TRACE() << DEB_VAR4(wRoiX0, wRoiY0, wRoiX1, wRoiY1);

        error = PcoCheckError(PCO_SetROI(m_handle, wRoiX0, wRoiY0, wRoiX1, wRoiY1));
        PCO_THROW_OR_TRACE(error, "PCO_SetROI") ;

        m_roi.changed= Valid;
    }

	error = PcoCheckError(PCO_GetROI(m_handle, &wRoiX0, &wRoiY0, &wRoiX1, &wRoiY1));
    PCO_THROW_OR_TRACE(error, "PCO_GetROI") ;
	DEB_TRACE() << DEB_VAR4(wRoiX0, wRoiY0, wRoiX1, wRoiY1);


	//------------------------------------------------- triggering mode 
    //------------------------------------- acquire mode : ignore or not ext. signal
	msg = _pcoSet_Trig_Acq_Mode(error);
    PCO_THROW_OR_TRACE(error, msg) ;

    // ----------------------------------------- storage mode (recorder + sequence)
    if(_isCameraType(Dimax)) {
		
		enumPcoStorageMode mode = (iRequestedFrames > 0) ? RecSeq : Fifo;

		msg = _pcoSet_Storage_subRecord_Mode(mode, error);
		PCO_THROW_OR_TRACE(error, msg) ;
	}

	//----------------------------------- set exposure time & delay time
	msg = _pcoSet_Exposure_Delay_Time(error,0);
	PCO_THROW_OR_TRACE(error, msg) ;


    //------------------------------------------------- check recording state
    error = PcoCheckError(PCO_GetRecordingState(m_handle, &state));
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
    error = PcoCheckError(PCO_ArmCamera(m_handle)); PCO_THROW_OR_TRACE(error, "PCO_ArmCamera") ;

		if(_isCameraType(Edge)) {
			error = PcoCheckError(PCO_GetPixelRate(m_handle, &m_pcoData->dwPixelRate));
		    PCO_THROW_OR_TRACE(error, "PCO_GetPixelRate") ;

			if(_isValid_pixelRate(m_pcoData->dwPixelRateRequested) && 
					(m_pcoData->dwPixelRate != m_pcoData->dwPixelRateRequested)) {

				error = PcoCheckError(PCO_SetPixelRate(m_handle, m_pcoData->dwPixelRateRequested));
			    PCO_THROW_OR_TRACE(error, "PCO_SetPixelRate") ;

				error = PcoCheckError(PCO_GetPixelRate(m_handle, &m_pcoData->dwPixelRate));
			    PCO_THROW_OR_TRACE(error, "PCO_GetPixelRate") ;

				error = PcoCheckError(PCO_ArmCamera(m_handle));
			    PCO_THROW_OR_TRACE(error, "PCO_ArmCamera") ;
			}
			m_pcoData->dwPixelRateRequested = 0;
		}

	//====================================== get the coc runtime 
    //---- only valid if it was used PCO_SetDelayExposureTime
    //---- and AFTER armed the cam
	msg = _get_coc_runtime(error); PCO_THROW_OR_TRACE(error, msg) ;


    //--------------------------- PREPARE / getSizes, pixelRate, clXferParam, LUT, setImgParam, Arm

	error = PcoCheckError(PCO_GetSizes(m_handle, &m_pcoData->wXResActual, &m_pcoData->wYResActual, &m_pcoData->wXResMax, &m_pcoData->wYResMax));
    PCO_THROW_OR_TRACE(error, "PCO_GetSizes") ;
	msg = _prepare_cameralink_interface(error); PCO_THROW_OR_TRACE(error, msg) ;
	msg = _pcoSet_Exposure_Delay_Time(error,1); PCO_THROW_OR_TRACE(error, msg) ;
	error = PcoCheckError(PCO_ArmCamera(m_handle)); PCO_THROW_OR_TRACE(error, "PCO_ArmCamera") ;
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

	if(_isCameraType(Dimax)){
		_pcoSet_RecordingState(1, error);
		if(iRequestedFrames > 0 ) {
			_beginthread( _pco_acq_thread_dimax, 0, (void*) this);
		} else {
			_beginthread( _pco_acq_thread_dimax_fifo, 0, (void*) this);
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

	printf("=== %s> ENTRY\n", fnId);

	Camera* m_cam = (Camera *) argin;
	SyncCtrlObj* m_sync = m_cam->_getSyncCtrlObj();
	BufferCtrlObj* m_buffer = m_sync->_getBufferCtrlObj();

	struct stcPcoData *m_pcoData = m_cam->_getPcoData();
	
	char *msg;
	struct __timeb64 tStart;
	msElapsedTimeSet(tStart);

	long timeout, timeout0, msNow, msRec, msXfer;
	int nb_acq_frames;
	bool requestStop = false;

	HANDLE m_handle = m_cam->getHandle();

	
	WORD wSegment = m_cam->pcoGetActiveRamSegment(); 
	DWORD dwMsSleep = (DWORD) (m_cam->pcoGetCocRunTime() * 1000.);
	if(dwMsSleep == 0) dwMsSleep = 1;

	int nb_frames; 	m_sync->getNbFrames(nb_frames);
	m_sync->setAcqFrames(0);

	timeout = timeout0 = (long) (dwMsSleep * (nb_frames * 1.1));
	if(timeout < TOUT_MIN_DIMAX) timeout = TOUT_MIN_DIMAX;
	m_pcoData->msAcqTout = timeout;
	_dwValidImageCnt = 0;

	m_sync->setExposing(pcoAcqRecordStart);

	while(_dwValidImageCnt <  (DWORD) nb_frames) {
		Sleep(dwMsSleep);	
		msg = m_cam->_PcoCheckError(PCO_GetNumberOfImagesInSegment(m_handle, wSegment, &_dwValidImageCnt, &_dwMaxImageCnt), error);
		if(error) {
			printf("=== %s [%d]> ERROR %s\n", fnId, __LINE__, msg);
			throw LIMA_HW_EXC(Error, "PCO_GetNumberOfImagesInSegment");
		}

		m_pcoData->msAcqTnow = msNow = msElapsedTime(tStart);
		if(timeout < msNow) { 
			m_sync->setExposing(pcoAcqRecordTimeout);
			printf("=== %s [%d]> TIMEOUT!!! tout0[%ld] tout[%ld] ms[%ld]\n", fnId, __LINE__, timeout0, timeout, msNow);
			break;
		}
	
	
		if(requestStop = m_buffer->_getRequestStop()) {
			m_sync->setExposing(pcoAcqStop);
			printf("=== %s> STOP REQUESTED\n", fnId);
			break;
		}
	}

	msg = m_cam->_pcoSet_RecordingState(0, error);
	if(error) {
		printf("=== %s [%d]> ERROR %s\n", fnId, __LINE__, msg);
		throw LIMA_HW_EXC(Error, "_pcoSet_RecordingState");
	}

	if(m_sync->getExposing() == pcoAcqRecordStart) m_sync->setExposing(pcoAcqRecordEnd);

	msg = m_cam->_PcoCheckError(PCO_GetNumberOfImagesInSegment(m_handle, wSegment, &_dwValidImageCnt, &_dwMaxImageCnt), error);
	if(error) {
		printf("=== %s [%d]> ERROR %s\n", fnId, __LINE__, msg);
		throw LIMA_HW_EXC(Error, "PCO_GetNumberOfImagesInSegment");
	}

	m_pcoData->dwValidImageCnt[wSegment-1] = _dwValidImageCnt;
	m_pcoData->dwMaxImageCnt[wSegment-1] = _dwMaxImageCnt;

	nb_acq_frames = (_dwValidImageCnt < (DWORD) nb_frames) ? _dwValidImageCnt : nb_frames;
	//m_sync->setAcqFrames(nb_acq_frames);


	m_pcoData->msAcqRec = msRec = msElapsedTime(tStart);
	msElapsedTimeSet(tStart);
	if(m_buffer->_getRequestStop()) {
		m_sync->setExposing(pcoAcqStop);
	} else {
			pcoAcqStatus status = (pcoAcqStatus) m_buffer->_xferImag();
			m_sync->setExposing(status);

			if(!m_buffer->_getRequestStop()) m_sync->stopAcq();
	}
	//m_sync->setExposing(status);
	m_pcoData->msAcqXfer = msXfer = msElapsedTime(tStart);
	printf("=== %s> EXIT tnow[%ld] tout[%ld] tout0[%ld] rec[%ld] xfer[%ld] (ms)\n", 
			fnId, msNow, timeout, timeout0, msRec, msXfer);
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

	printf("=== %s> ENTRY\n", fnId);
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
	bool requestStop = false;

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

void _pco_acq_thread_dimax_fifo(void *argin) {
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
	bool requestStop = false;

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
void Camera::reset()
{
  DEB_MEMBER_FUNCT();
  //@todo maybe something to do!
}


//=========================================================================================================
//=========================================================================================================
int Camera::PcoCheckError(int err) {
	static char lastErrorMsg[500];
	if (err != 0) {
		DWORD dwErr = err;
		m_pcoData->pcoError = err;
		PCO_GetErrorText(dwErr, m_pcoData->pcoErrorMsg, ERR_SIZE-14);
		printf("===> [x%08x][%s]\n", dwErr, m_pcoData->pcoErrorMsg);
		return (err);
	}
	return (err);
}

//=========================================================================================================
//=========================================================================================================
char* Camera::_PcoCheckError(int err, int &error) {
	static char lastErrorMsg[ERR_SIZE];
	error = m_pcoData->pcoError = err;
	if (err != 0) {
		PCO_GetErrorText(err, lastErrorMsg, ERR_SIZE-14);
		strncpy_s(m_pcoData->pcoErrorMsg, ERR_SIZE, lastErrorMsg, _TRUNCATE); 

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


		if(!_isCameraType(Dimax)) {
			printf("=== %s> unknow camera type [%d]\n", fnId, _getCameraType());
			return -1;
		}

		if((segmentPco <1) ||(segmentPco > PCO_MAXSEGMENTS)) {
			printf("=== %s> ERROR segmentPco[%d]\n", fnId, segmentPco);
			return -1;
		}

		xroisize = m_roi.x[1] - m_roi.x[0] + 1;
		yroisize = m_roi.y[1] - m_roi.y[0] + 1;

		pixPerFrame = (unsigned long long)xroisize * (unsigned long long)xroisize;

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
	WORD trigmode = m_sync->getPcoTrigMode();
    error = PcoCheckError(PCO_SetTriggerMode(m_handle, trigmode));
	if(error) return "PCO_SetTriggerMode";
	//PCO_THROW_OR_TRACE(error, "PCO_SetTriggerMode") ;
	//DEB_TRACE() << DEB_VAR1(trigmode);

    //------------------------------------- acquire mode : ignore or not ext. signal

	WORD acqmode = m_sync->getPcoAcqMode();
	error = PcoCheckError(PCO_SetAcquireMode(m_handle, acqmode));
	if(error) return "PCO_SetAcquireMode";
   //PCO_THROW_OR_TRACE(error, "PCO_SetAcquireMode") ;
	//DEB_TRACE() << DEB_VAR1(acqmode);
	return fnId;
}



//=================================================================================================
//=================================================================================================
char * Camera::_pcoSet_Storage_subRecord_Mode(enumPcoStorageMode mode, int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	    // ----------------------------------------- storage mode (recorder + sequence)
		// current storage mode
		// - 0x0000 = [recorder] mode
		// - 0x0001 = [FIFO buffer] mode
		// current recorder submode:
		// - 0x0000 = [sequence]
		// - 0x0001 = [ring buffer].

	switch(mode) {
		case RecSeq:  m_pcoData->storage_mode = 0; m_pcoData->recorder_submode = 0; break;
		case RecRing: m_pcoData->storage_mode = 0; m_pcoData->recorder_submode = 1; break;
		case Fifo:    m_pcoData->storage_mode = 1; m_pcoData->recorder_submode = 0; break;
		default: 
			throw LIMA_HW_EXC(Error,"FATAL - invalid storage mode!" );
	}

    error = PcoCheckError(PCO_SetStorageMode(m_handle, m_pcoData->storage_mode));
	if(error) return "PCO_SetStorageMode";
    //PCO_THROW_OR_TRACE(error, "PCO_SetStorageMode") ;

    error = PcoCheckError(PCO_SetRecorderSubmode(m_handle, m_pcoData->recorder_submode));
	if(error) return "PCO_SetRecorderSubmode";
    //PCO_THROW_OR_TRACE(error, "PCO_SetRecorderSubmode") ;

	return fnId;
}


//=================================================================================================
//=================================================================================================

// 4294967295.0 = pow(2., 32) - 1.
#define DWORD_MAX_FLOAT 4294967295.0

char* Camera::_pcoSet_Exposure_Delay_Time(int &error, int ph){
	DEB_MEMBER_FUNCT();
	DEF_FNID;
    float factor;
	bool doIt;

    DWORD dwExposure, dwDelay;
	WORD wExposure_base, wDelay_base;
    double _exposure, _delay, val;
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
			//if (period > _delay + _exposure) {
			//	_delay = period  - _exposure;
			//	printf("--- %s> delay forced [%g] -> [%g]\n", fnId, _delay0, _delay);
			//}
		}
	}

	if(!doIt) return fnId;

		// exp/lat time is saved in s. PCO requires it expressed in ms(=2), us(=1), ns(=0)
	// test time expressed in ns(=0), us(=1), ms(=2) up not overflow max precision in 32 bits
    for (wExposure_base = 0; wExposure_base < 3; wExposure_base++) {  // base 0(ns), 1(us), 2(ms)
        factor = pow((float)10, (int) (wExposure_base * 3 - 9));		// factor 10E-9, 10E-6, 10E-3
        if ( (val = (_exposure / factor)) <= DWORD_MAX_FLOAT) {		// multiply by 10E9, 10E6, 10E3
            dwExposure = (DWORD) val;			// exposure max precision in 32 bits, exposure base 0(ns)  1(us)  2(ms)
            break;
        }
    }
    //====================================== TODO set/get the value of ccd.delay now is 0 
    for (wDelay_base = 0; wDelay_base < 3; wDelay_base++) {
        factor = pow((float) 10, (int) (wDelay_base * 3 - 9));
        if ( (val = (_delay / factor)) <= DWORD_MAX_FLOAT) {
            dwDelay = (DWORD) val;
            break;
        }
    }

	error = PcoCheckError(PCO_SetDelayExposureTime(m_handle, dwDelay, dwExposure, wDelay_base, wExposure_base));
	if(error) {
		DEB_TRACE() << DEB_VAR2(_exposure, _delay);	
		DEB_TRACE() << DEB_VAR4(dwDelay, dwExposure, wDelay_base, wExposure_base);	
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

	switch (m_pcoData->stcCamType.wInterfaceType) {
        case INTERFACE_CAMERALINK:

            error = PcoCheckError(PCO_GetTransferParameter(m_handle, &m_pcoData->clTransferParam, sizeof(m_pcoData->clTransferParam)));
			if(error) return "PCO_GetTransferParameter";
            //PCO_THROW_OR_TRACE(error, "PCO_GetTransferParameter") ;


            if((m_pcoData->clTransferParam.baudrate != 115200) || (m_pcoData->clTransferParam.DataFormat != 2)) {
                m_pcoData->clTransferParam.baudrate=115200;
                m_pcoData->clTransferParam.DataFormat=2;

                error = PcoCheckError(PCO_SetTransferParameter(m_handle, &m_pcoData->clTransferParam, sizeof(m_pcoData->clTransferParam)));
				if(error) return "PCO_SetTransferParameter";
 				//PCO_THROW_OR_TRACE(error, "PCO_SetTransferParameter") ;
            }

            // ---- no break
            
        case INTERFACE_ETHERNET:
		    WORD wXres, wYres;

            wXres= m_pcoData->wXResActual;
            wYres= m_pcoData->wYResActual;
			error = PcoCheckError(PCO_CamLinkSetImageParameters(m_handle, wXres, wYres));
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

	
	//error = PcoCheckError(PCO_GetTransferParameter(m_handle, &m_pcoData->clTransferParam, sizeof(m_pcoData->clTransferParam)));
	error = PcoCheckError(PCO_GetTransferParameter(m_handle, &m_pcoData->clTransferParam, sizeof(PCO_SC2_CL_TRANSFER_PARAM)));
    PCO_THROW_OR_TRACE(error, "PCO_GetTransferParameter") ;
	memcpy(&_pcoData.clTransferParam, &m_pcoData->clTransferParam,sizeof(PCO_SC2_CL_TRANSFER_PARAM));
	
	m_pcoData->clTransferParam.baudrate = PCO_CL_BAUDRATE_115K2;

	if(_isCameraType(Dimax)){
			//m_pcoData->clTransferParam.Transmit = 1;
			//_pcoData.clTransferParam.Transmit = m_pcoData->clTransferParam.Transmit;
			m_pcoData->clTransferParam.DataFormat=PCO_CL_DATAFORMAT_2x12; //=2
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
		error = PcoCheckError(PCO_SetTransferParameter(m_handle, &m_pcoData->clTransferParam, sizeof(m_pcoData->clTransferParam)));
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
		error = PcoCheckError(
			PCO_GetActiveLookupTable(m_handle, &_wLUT_Identifier, &_wLUT_Parameter));
	    PCO_THROW_OR_TRACE(error, "PCO_GetActiveLookupTable") ;

		if(_wLUT_Identifier != m_pcoData->wLUT_Identifier) {
			bDoArm = TRUE;
			error = PcoCheckError(
				PCO_SetActiveLookupTable(m_handle, &m_pcoData->wLUT_Identifier, &m_pcoData->wLUT_Parameter));
				PCO_THROW_OR_TRACE(error, "PCO_SetActiveLookupTable") ;

			error = PcoCheckError(
				PCO_GetActiveLookupTable(m_handle, &m_pcoData->wLUT_Identifier, &m_pcoData->wLUT_Parameter));
				PCO_THROW_OR_TRACE(error, "PCO_GetActiveLookupTable") ;
		}
	}

	if(bDoArm) {
		error = PcoCheckError(PCO_ArmCamera(m_handle));
		PCO_THROW_OR_TRACE(error, "PCO_ArmCamera") ;
	}

	error = PcoCheckError(PCO_CamLinkSetImageParameters(m_handle, m_pcoData->wXResActual, m_pcoData->wYResActual));
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
		m_pcoData->stcCamType.wSize= sizeof(m_pcoData->stcCamType);
		error = PcoCheckError(PCO_GetCameraType(m_handle, &m_pcoData->stcCamType));
		msg = "PCO_GetCameraType";
		PCO_PRINT_ERR(error, msg); 	if(error) return msg;

		ptr = xlatPcoCode2Str(_getCameraType(), ModelType, error);
		strcpy_s(m_pcoData->model, MODEL_TYPE_SIZE, ptr);
		if(error) return m_pcoData->model;
		//DEB_TRACE() <<   "m_pcoData->model " << m_pcoData->model;
		//if(error) throw LIMA_HW_EXC(Error, "Unknow model");
		
		ptr = xlatPcoCode2Str(m_pcoData->stcCamType.wInterfaceType, InterfaceType, error);
		strcpy_s(m_pcoData->iface, INTERFACE_TYPE_SIZE, ptr);
		if(error) return m_pcoData->iface;

		//DEB_TRACE() <<   "m_pcoData->iface " << m_pcoData->iface;
		//if(error) throw LIMA_HW_EXC(Error, "Unknow interface");

		sprintf_s(m_pcoData->camera_name, CAMERA_NAME_SIZE, "%s %s", m_pcoData->model, m_pcoData->iface);
	}

	// -- Reset to default settings


	error = PcoCheckError(PCO_SetRecordingState(m_handle, 0));
	msg = "PCO_SetRecordingState";
	PCO_PRINT_ERR(error, msg); 	if(error) return msg;

	error = PcoCheckError(PCO_ResetSettingsToDefault(m_handle));
	msg = "PCO_ResetSettingsToDefault";
	PCO_PRINT_ERR(error, msg); 	if(error) return msg;
	

	// -- Get camera description
	m_pcoData->pcoInfo.wSize= sizeof(m_pcoData->pcoInfo);

	error = PcoCheckError(PCO_GetCameraDescription(m_handle, &m_pcoData->pcoInfo));
	msg = "PCO_GetCameraDescription";
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
	error = PcoCheckError(PCO_GetTemperature(m_handle, &m_pcoData->temperature.wCcd, &m_pcoData->temperature.wCam, &m_pcoData->temperature.wPower));
	if(error) return "PCO_GetTemperature";
	//PCO_THROW_OR_TRACE(error, "PCO_GetTemperature") ;

	sprintf_s(msg, MSG_SIZE, "* temperature: CCD[%.1f]  CAM[%d]  PS[%d]\n", m_pcoData->temperature.wCcd/10., m_pcoData->temperature.wCam, m_pcoData->temperature.wPower);
	//DEB_TRACE() <<   msg;
	m_log.append(msg);


	m_pcoData->temperature.wMinCoolSet = m_pcoData->pcoInfo.sMinCoolSetDESC;
	m_pcoData->temperature.wMaxCoolSet = m_pcoData->pcoInfo.sMaxCoolSetDESC;

	sprintf_s(msg, MSG_SIZE, "* cooling temperature: MIN [%d]  Max [%d]\n",  m_pcoData->temperature.wMinCoolSet, m_pcoData->temperature.wMaxCoolSet);
	//DEB_TRACE() <<   msg;
	m_log.append(msg);

	// -- Set/Get cooling temperature
	if (m_pcoData->temperature.wSetpoint != -1) {
		if (m_pcoData->temperature.wSetpoint < m_pcoData->temperature.wMinCoolSet)	m_pcoData->temperature.wSetpoint = m_pcoData->temperature.wMinCoolSet;
		if (m_pcoData->temperature.wSetpoint > m_pcoData->temperature.wMaxCoolSet)	m_pcoData->temperature.wSetpoint= m_pcoData->temperature.wMaxCoolSet;
	} else {
		error = PcoCheckError(PCO_GetCoolingSetpointTemperature(m_handle, &m_pcoData->temperature.wSetpoint));
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

	error = PcoCheckError(PCO_GetRecordingState(m_handle, &wRecState_actual));
	msg = "PCO_GetRecordingState";
	PCO_PRINT_ERR(error, msg); 	if(error) return msg;

	//if(wRecState_new == wRecState_actual) {error = 0; return fnId; }

	error = PcoCheckError(PCO_SetRecordingState(m_handle, wRecState_new));
	msg = "PCO_SetRecordingState";
	PCO_PRINT_ERR(error, msg); 	if(error) return msg;

	if(wRecState_new == 0) {
		error = PcoCheckError(PCO_CancelImages(m_handle));
		msg = "PCO_CancelImages";
		PCO_PRINT_ERR(error, msg); 	if(error) return msg;
	}

	return fnId;

}

//=================================================================================================
//=================================================================================================
char *Camera::_get_coc_runtime(int &error){
		
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	//====================================== get the coc runtime 
    //---- only valid if it was used PCO_SetDelayExposureTime
    //---- and AFTER armed the cam

	DWORD dwTime_s, dwTime_ns;
    double runTime;

    error = PcoCheckError(PCO_GetCOCRuntime(m_handle, &dwTime_s, &dwTime_ns));
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
		error = PcoCheckError(
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

    error = PcoCheckError(PCO_GetCameraSetup(m_handle, &m_wType, &m_dwSetup[0], &m_wLen));
	msg = "PCO_GetCameraSetup";
	PCO_PRINT_ERR(error, msg); 	if(error) return;

	if(m_dwSetup[0] == _dwSetup) { m_config = FALSE;return;}

	m_dwSetup[0] = _dwSetup;

    error = PcoCheckError(PCO_SetTimeouts(m_handle, &ts[0], sizeof(ts)));
	msg = "PCO_SetTimeouts";
	PCO_PRINT_ERR(error, msg); 	if(error) return;

    error = PcoCheckError(PCO_SetCameraSetup(m_handle, m_wType, &m_dwSetup[0], m_wLen));
	msg = "PCO_SetCameraSetup";
	PCO_PRINT_ERR(error, msg); 	if(error) return;

    error = PcoCheckError(PCO_RebootCamera(m_handle));
	msg = "PCO_RebootCamera";
	PCO_PRINT_ERR(error, msg); 	if(error) return;

	m_sync->_getBufferCtrlObj()->_pcoAllocBuffersFree();

    error = PcoCheckError(PCO_CloseCamera(m_handle));
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


    error = PcoCheckError(PCO_GetCameraSetup(m_handle, &m_wType, &m_dwSetup[0], &m_wLen));
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
			if(dwPixelRate == m_pcoData->pcoInfo.dwPixelRateDESC[i]) return TRUE;
		}

	return FALSE;
}



//=================================================================================================
//=================================================================================================
bool Camera::_isValid_Roi(struct stcRoi *new_roi){
		
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	int xMax = m_pcoData->pcoInfo.wMaxHorzResStdDESC;
	int yMax = m_pcoData->pcoInfo.wMaxVertResStdDESC;
	int xSteps = m_pcoData->pcoInfo.wRoiHorStepsDESC;
	int ySteps = m_pcoData->pcoInfo.wRoiVertStepsDESC;
	int x0 = new_roi->x[0];
	int x1 = new_roi->x[1];
	int y0 = new_roi->y[0];
	int y1 = new_roi->y[1];

	if(
		(x0 < 1) || (x0 > x1) || (x1 > xMax) ||
		(y0 < 1) || (y0 > y1) || (y1 > yMax) ||
		(((x0 - 1) % xSteps) != 0) ||((x1 % xSteps) != 0) ||
		(((y0 - 1) % ySteps) != 0) ||((y1 % ySteps) != 0) ) return FALSE;
		
	return TRUE;
}


//=================================================================================================
//=================================================================================================
bool Camera::_isCameraType(enum enumPcoFamily tp){
		
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	switch(_getCameraType()) {
		case CAMERATYPE_PCO_DIMAX_STD: 
			return tp == Dimax ;
		
		case CAMERATYPE_PCO_EDGE_GL:
			return((tp == EdgeGL) || (tp == Edge));

		case CAMERATYPE_PCO_EDGE:
			return((tp == EdgeRolling) || (tp == Edge));

		default:
			return FALSE;

	}
		
}

//=================================================================================================
//=================================================================================================
void Camera::_pco_GetPixelRate(DWORD &pixRate, int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;
		if(!_isCameraType(Edge)) {
			pixRate = 0;
			error = -1;
			return;
		}

		error = PcoCheckError(PCO_GetPixelRate(m_handle, &m_pcoData->dwPixelRate));
	    PCO_THROW_OR_TRACE(error, "PCO_GetPixelRate") ;

		pixRate = m_pcoData->dwPixelRate;
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
