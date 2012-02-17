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


#define BUFF_INFO_SIZE 5000

#define DWORD_MAX ULONG_MAX 

#include <cstdlib>
#include <process.h>

#include <sys/timeb.h>
#include <time.h>

#include "Exceptions.h"

#include "PcoCamera.h"
#include "PcoSyncCtrlObj.h"
#include "PcoBufferCtrlObj.h"

using namespace lima;
using namespace lima::Pco;

static char *timebaseUnits[] = {"ns", "us", "ms"};

void _pco_acq_thread_dimax(void *argin);
void _pco_acq_thread_edge(void *argin);



char * _timestamp_pcosyncctrlobj();
char * _timestamp_pcointerface();
char * _timestamp_pcobufferctrlobj();
char * _timestamp_pcodetinfoctrlobj();
	
//=========================================================================================================
char* _timestamp_pcocamera() {return __TIMESTAMP__ " (" __FILE__ ")";}
//=========================================================================================================

//====================================================================
//====================================================================

char *str_trim_left(char *s) {
	char c;
	if(s == NULL) return NULL;
	while((c = *s) != 0) {
		if(!isspace(c)) break;
		s++;		
	}
	return s;
}

char *str_trim_right(char *s) {
	char *ptr;
	if(s == NULL) return NULL;
	ptr = s + strlen(s) - 1;
	while(s <= ptr) {
		if(!isspace(*ptr)) break;
		*ptr-- = 0;
	}
	return s;
}
char *str_trim(char *s) {
	return str_trim_left(str_trim_right(s));
}
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
enum timestampFmt {Iso, FnFull, FnDate};

static char *getTimestamp(timestampFmt fmtIdx) {
   static char timeline[128];
   errno_t err;
	time_t ltime;
	struct tm today;
	char *fmt;

  switch(fmtIdx) {
    case Iso: fmt = "%Y/%m/%d %H:%M:%S"; break;
    case FnDate: fmt = "%Y-%m-%d"; break;

    default:
    case FnFull: fmt = "%Y-%m-%d-%H%M%S"; break;
  }

	time( &ltime );
	err = localtime_s( &today, &ltime );
	strftime(timeline, 128, fmt, &today );
      
	return timeline;
}

//=========================================================================================================
//=========================================================================================================
Camera::Camera(const char *camPar) :
	m_cam_connected(false),
	m_acq_frame_nb(1),
	m_sync(NULL)
{
	DEB_CONSTRUCTOR();
	char msg[MSG_SIZE + 1];
	int error=0;
	char *errMsg;

	m_pcoData =new(stcPcoData);
	if(m_pcoData == NULL)
		throw LIMA_HW_EXC(Error, "creation error");
	memset((char *)m_pcoData, 0, sizeof(m_pcoData));

	m_bin.changed = Invalid;
	m_roi.changed = Invalid;

	m_log.clear();
	m_log.append("\n");

	DebParams::checkInit();

	// --- Open Camera
	error = PcoCheckError(PCO_OpenCamera(&m_handle, 0));
	PCO_TRACE("PCO_OpenCamera") ;
  
	errMsg = _pcoGet_Camera_Type(error);
	PCO_TRACE(errMsg) ;

		// -- Initialise adc, size, bin, roi
	m_pcoData->nr_adc= 1;
	m_pcoData->max_adc = m_pcoData->pcoInfo.wNumADCsDESC;

	m_pcoData->maxWidth = (unsigned int) m_pcoData->pcoInfo.wMaxHorzResStdDESC; // ds->ccd.size.xmax,
	m_pcoData->maxHeight= (unsigned int) m_pcoData->pcoInfo.wMaxVertResStdDESC; // ds->ccd.size.ymax,
	m_pcoData->bitsPerPix = (unsigned int) m_pcoData->pcoInfo.wDynResDESC; // ds->ccd.size.bits
	m_pcoData->bytesPerPix = (m_pcoData->bitsPerPix <= 8)?1:2; // nr de bytes por pixel  12 bits -> 2 bytes

	//m_allocatedBufferSizeMax = m_pcoData->maxWidth * m_pcoData->maxHeight * m_pcoData->bytesPerPix;

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
	PCO_TRACE(errMsg) ;

	_pcoSet_RecordingState(0, error);

	switch(_getCameraType()) {
		case CAMERATYPE_PCO_DIMAX_STD: _init_dimax(); break;
		case CAMERATYPE_PCO_EDGE: _init_edge(); break;
		default: throw LIMA_HW_EXC(Error, "Camera type not supported!");
			break;
	}


	m_cam_connected = true;
	error = 0;

	if(!m_cam_connected)
		throw LIMA_HW_EXC(Error, "Camera not found!");


  DEB_TRACE() << m_log;
  DEB_TRACE() << "END OF CAMERA";

}

int Camera::_init_dimax() {

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
		PCO_TRACE("PCO_GetCameraRamSize") ;

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
		PCO_TRACE("PCO_GetCameraRamSegmentSize") ;

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
			PCO_TRACE("PCO_GetNumberOfImagesInSegment") ;

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
		PCO_TRACE("PCO_SetCameraRamSegmentSize") ;
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
		PCO_TRACE("PCO_GetActiveRamSegment") ;

		error = PcoCheckError(PCO_GetNumberOfImagesInSegment(m_handle, m_pcoData->activeRamSegment, &_dwValidImageCnt, &_dwMaxImageCnt));
		PCO_TRACE("PCO_GetNumberOfImagesInSegment") ;



	DEB_TRACE() <<  "original DONE";


	return 0;
}


int Camera::_init_edge() {


	return 0;

}



Camera::~Camera()
{
  DEB_DESTRUCTOR();
	int error;
	char *sErr;

  DEB_TRACE() << "~Camera";

  if(m_cam_connected)
    {
	m_sync->_getBufferCtrlObj()->_pcoAllocBuffersFree();

	sErr = _PcoCheckError(PCO_CloseCamera(m_handle), error);
	if(error) {
    	DEB_TRACE() << sErr;
		THROW_HW_ERROR(NotSupported) << sErr;
	}
	m_cam_connected = false;
  }
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


//=====================================================================
	DEF_FNID;
    WORD state;
    HANDLE hEvent= NULL;

	int error;
	char *msg;

    //------------------------------------------------- set binning if needed
    WORD wBinHorz, wBinVert;
    if (m_bin.changed == Changed) {
		wBinHorz = (WORD)m_bin.x;
		wBinVert = (WORD)m_bin.y;
        error = PcoCheckError(PCO_SetBinning(m_handle, wBinHorz, wBinVert));
        PCO_TRACE("PCO_SetBinning") ;
        m_bin.changed= Valid;
    }

    error = PcoCheckError(PCO_GetBinning(m_handle, &wBinHorz, &wBinVert));
    PCO_TRACE("PCO_GetBinning") ;
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
        PCO_TRACE("PCO_SetROI") ;

        m_roi.changed= Valid;
    }

	error = PcoCheckError(PCO_GetROI(m_handle, &wRoiX0, &wRoiY0, &wRoiX1, &wRoiY1));
    PCO_TRACE("PCO_GetROI") ;
	DEB_TRACE() << DEB_VAR4(wRoiX0, wRoiY0, wRoiX1, wRoiY1);


	//------------------------------------------------- triggering mode 
    //------------------------------------- acquire mode : ignore or not ext. signal
	msg = _pcoSet_Trig_Acq_Mode(error);
    PCO_TRACE(msg) ;

    // ----------------------------------------- storage mode (recorder + sequence)
    if(_getCameraType() == CAMERATYPE_PCO_DIMAX_STD) {
		msg = _pcoSet_Storage_subRecord_Mode(error);
		PCO_TRACE(msg) ;
	}
	//----------------------------------- set exposure time & delay time
	msg = _pcoSet_Exposure_Delay_Time(error);
	PCO_TRACE(msg) ;


    //------------------------------------------------- check recording state
    error = PcoCheckError(PCO_GetRecordingState(m_handle, &state));
    PCO_TRACE("PCO_GetRecordingState") ;

    if (state>0) {
        DEB_TRACE() << "Force recording state to 0x0000" ;

		_pcoSet_RecordingState(0, error);
        PCO_TRACE("PCO_SetRecordingState") ;
	}

//-----------------------------------------------------------------------------------------------
//	5. Arm the camera.
//	6. Get the sizes and allocate a buffer:
//		PCO_GETSIZES(hCam, &actualsizex, &actualsizey, &ccdsizex, &ccdsizey)
//		PCO_ALLOCATEBUFFER(hCam, &bufferNr, actualsizex * actualsizey * sizeof(WORD), &data, &hEvent)
//		In case of CamLink and GigE interface: PCO_CamLinkSetImageParameters(actualsizex, actualsizey)
//		PCO_ArmCamera(hCam)
//-----------------------------------------------------------------------------------------------
	
	msg = _set_metadata_mode(0, error); PCO_TRACE(msg) ;

	// ------------------------------------------------- arm camera
    error = PcoCheckError(PCO_ArmCamera(m_handle));
    PCO_TRACE("PCO_ArmCamera") ;


    //====================================== get the coc runtime 
    //---- only valid if it was used PCO_SetDelayExposureTime
    //---- and AFTER armed the cam
	msg = _get_coc_runtime(error); PCO_TRACE(msg) ;
	



#if 0
	//------------------------------------------------ get size
	error = PcoCheckError(PCO_GetSizes(m_handle, &m_pcoData->wXResActual, &m_pcoData->wYResActual, &m_pcoData->wXResMax, &m_pcoData->wYResMax));
    PCO_TRACE("PCO_GetSizes") ;
	error = PcoCheckError(PCO_GetPixelRate(m_handle, &m_pcoData->dwPixelRate));
    PCO_TRACE("PCO_GetPixelRate") ;
	msg = _pcoSet_Cameralink_GigE_Parameters(error);
	PCO_TRACE(msg) ;
#endif


    //--------------------------- PREPARE / getSizes, pixelRate, clXferParam, LUT, setImgParam, Arm
	msg = _prepare_cameralink_interface(error); PCO_TRACE(msg) ;


    //------------------------------------------------- checking nr of frames
    {
        unsigned long framesMax;
        int iFrames;

        m_sync->getNbFrames(iFrames);
        framesMax = pcoGetFramesMax(m_pcoData->activeRamSegment);

        if ((((unsigned long) iFrames) > framesMax)) {
            throw LIMA_HW_EXC(Error, "frames OUT OF RANGE");
        }
    } 

	//------------------------------------------------- start acquisition

	m_sync->setStarted(true);
	m_sync->setExposing(pcoAcqRecordStart);

	switch(_getCameraType()) {
		case CAMERATYPE_PCO_EDGE:
			_beginthread( _pco_acq_thread_edge, 0, (void*) this);
			break;

		case CAMERATYPE_PCO_DIMAX_STD:
			_pcoSet_RecordingState(1, error);
			_beginthread( _pco_acq_thread_dimax, 0, (void*) this);
			break;

		default:
			throw LIMA_HW_EXC(Error, "unkown camera type");

	}



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

	long timeout, msNow, msRec, msXfer;
	int nb_acq_frames;
	bool requestStop = false;

	HANDLE m_handle = m_cam->getHandle();

	
	WORD wSegment = m_cam->pcoGetActiveRamSegment(); 
	DWORD dwMsSleep = (DWORD) (m_cam->pcoGetCocRunTime() * 1000.);
	if(dwMsSleep == 0) dwMsSleep = 1;

	int nb_frames; 	m_sync->getNbFrames(nb_frames);
	m_sync->setAcqFrames(0);

	m_pcoData->msAcqTout = timeout = (long) (dwMsSleep * (nb_frames * 1.1));
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
			printf("=== %s [%d]> TIMEOUT!!! timeout[%ld] ms[%ld]\n", fnId, __LINE__, timeout, msNow);
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
	m_sync->setAcqFrames(nb_acq_frames);


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
	printf("=== %s> EXIT tnow[%ld] tout[%ld] rec[%ld] xfer[%ld] (ms)\n", 
			fnId, msNow, timeout, msRec, msXfer);
	_endthread();
}

//==========================================================================================================
//==========================================================================================================

void _pco_acq_thread_edge(void *argin) {
	DEF_FNID;


	printf("=== %s> ENTRY\n", fnId);

	Camera* m_cam = (Camera *) argin;
	SyncCtrlObj* m_sync = m_cam->_getSyncCtrlObj();
	BufferCtrlObj* m_buffer = m_sync->_getBufferCtrlObj();

	struct stcPcoData *m_pcoData = m_cam->_getPcoData();

	struct __timeb64 tStart;
	msElapsedTimeSet(tStart);

	long msXfer;
	bool requestStop = false;

	HANDLE m_handle = m_cam->getHandle();

	
	pcoAcqStatus status = (pcoAcqStatus) m_buffer->_xferImag();
	m_sync->setExposing(status);

	//m_sync->setExposing(status);

	m_pcoData->msAcqXfer = msXfer = msElapsedTime(tStart);
	printf("=== %s> EXIT xfer[%ld] (ms)\n", 
			fnId, msXfer);
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
	m_pcoData->pcoError = err;
	if (err != 0) {
		PCO_GetErrorText(err, m_pcoData->pcoErrorMsg, ERR_SIZE-14);

		return (1);
	}
	return (0);
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

		if(_getCameraType() == CAMERATYPE_PCO_EDGE) {
			return LONG_MAX;
		}


		if(_getCameraType() != CAMERATYPE_PCO_DIMAX_STD) {
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

//=========================================================================================================
//=========================================================================================================

char *Camera::talk(char *cmd){
	static char buff[BUFF_INFO_SIZE +1];
	return _talk(cmd, buff, BUFF_INFO_SIZE);
}

#define NRTOK 5
#define NRCMDS 50
char *Camera::_talk(char *_cmd, char *output, int lg){
	DEB_MEMBER_FUNCT();
		char cmdBuff[BUFF_INFO_SIZE +1];
		char *cmd, *key, *keys[NRCMDS];
		int ikey = 0;
		char *tok[NRTOK];
		int tokNr;
		char *ptr, *ptrMax;
		int segmentPco = m_pcoData->activeRamSegment;
		int segmentArr = segmentPco -1;
		
		ptr = output; *ptr = 0;
		ptrMax = ptr + lg;

		int width = +20;
		
		strncpy_s(cmdBuff, BUFF_INFO_SIZE, _cmd, BUFF_INFO_SIZE);
		cmd = str_trim(cmdBuff);

		if(*cmd){
			char *tokContext;
			for(int i=0; i < NRTOK; i++) {
				if( (tok[i] = strtok_s(cmd, " ", &tokContext)) == NULL) break;
				cmd = NULL;
				tokNr = i;
			}
			cmd = tok[0];
		}

		if(*cmd == 0) {
			ptr += sprintf_s(ptr, ptrMax - ptr,"**** %s [begin]\n", __FUNCTION__);

			ptr += sprintf_s(ptr, ptrMax - ptr,"**** PCO log\n");
			ptr += sprintf_s(ptr, ptrMax - ptr,"%s\n", m_log.c_str());

			ptr += sprintf_s(ptr, ptrMax - ptr,"**** PCO Info\n");
			ptr += sprintf_s(ptr, ptrMax - ptr, "* timestamp[%s]\n", getTimestamp(Iso));
			ptr += sprintf_s(ptr, ptrMax - ptr, "* cam_name[%s]\n", m_pcoData->camera_name);

			ptr += sprintf_s(ptr, ptrMax - ptr, "* roi X(%d,%d) Y(%d,%d) size(%d,%d)\n",  
					m_roi.x[0], m_roi.x[1],
					m_roi.y[0], m_roi.y[1],
					m_roi.x[1] - m_roi.x[0] + 1, m_roi.y[1] - m_roi.y[0] + 1);

			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_cocRunTime=[%g s]\n",  m_pcoData->cocRunTime);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_frameRate=[%g fps]\n", m_pcoData->frameRate);
			
			ptr += sprintf_s(ptr, ptrMax - ptr, "* Acq (ms) Tnow=[%ld] Tout=[%ld] Rec=[%ld] Xfer=[%ld]\n", 
						m_pcoData->msAcqTnow, m_pcoData->msAcqTout, m_pcoData->msAcqRec, m_pcoData->msAcqXfer);

			double _exposure, _delay;
			m_sync->getExpTime(_exposure);
			m_sync->getLatTime(_delay);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->storage_mode=[%d]\n", m_pcoData->storage_mode);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->recorder_submode=[%d]\n", m_pcoData->recorder_submode);
			
			ptr += sprintf_s(ptr, ptrMax - ptr, "* _exposure=[%g s]\n", _exposure);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* _delay=[%g s]\n", _delay);
			

			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->wXResActual=[%d] .wYResActual=[%d] \n",  m_pcoData->wXResActual,  m_pcoData->wYResActual);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->wXResMax=[%d] .wYResMax=[%d] \n",  m_pcoData->wXResMax,  m_pcoData->wYResMax);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->wMetaDataSize=[%d] .wMetaDataVersion=[%d] \n",  m_pcoData->wMetaDataSize,  m_pcoData->wMetaDataVersion);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->dwPixelRate=[l%d]\n",  m_pcoData->dwPixelRate);


			int iFrames;
			m_sync->getNbFrames(iFrames);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_sync->getNbFrames=[%d frames]\n", iFrames);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* PcoActiveSegment=[%d]\n", segmentArr+1);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->dwMaxFramesInSegment[%d]=[%d frames]\n", segmentArr, m_pcoData->dwMaxFramesInSegment[segmentArr]);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->dwSegmentSize[%d]=[%d pages]\n", segmentArr, m_pcoData->dwSegmentSize[segmentArr]);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->wPixPerPage[%d pix]\n", m_pcoData->wPixPerPage);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->dwValidImageCnt[%d]=[%ld]\n", segmentArr, m_pcoData->dwValidImageCnt[segmentArr]);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->dwMaxImageCnt[%d]=[%ld]\n", segmentArr, m_pcoData->dwMaxImageCnt[segmentArr]);

			ptr += sprintf_s(ptr, ptrMax - ptr,"**** %s [end]\n", __FUNCTION__);
			return output;
		}
		
		key = keys[ikey++] = "cocRunTime";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "%g",  m_pcoData->cocRunTime);
			return output;
		}

		key = keys[ikey++] = "frameRate";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "%g", m_pcoData->frameRate);
			return output;
		}

		key = keys[ikey++] = "timestamp";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcocamera());
			ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcosyncctrlobj());
			ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcointerface());
			ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcobufferctrlobj());
			ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcodetinfoctrlobj());
			
			return output;
		}


		key = keys[ikey++] = "clTransferParam";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "      baudrate=[%u] %g Kbps\n", m_pcoData->clTransferParam.baudrate, m_pcoData->clTransferParam.baudrate/1000.);
			ptr += sprintf_s(ptr, ptrMax - ptr, "ClockFrequency=[%u] %g MHz\n", m_pcoData->clTransferParam.ClockFrequency, m_pcoData->clTransferParam.ClockFrequency/1000000.);
			ptr += sprintf_s(ptr, ptrMax - ptr, "        CCline=[%u]\n", m_pcoData->clTransferParam.CCline);
			ptr += sprintf_s(ptr, ptrMax - ptr, "    DataFormat=[%u]\n", m_pcoData->clTransferParam.DataFormat);
			ptr += sprintf_s(ptr, ptrMax - ptr, "      Transmit=[%u]\n", m_pcoData->clTransferParam.Transmit);
			
			return output;
		}

		key = keys[ikey++] = "maxNbImages";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "%ld", m_pcoData->dwMaxImageCnt[segmentArr]);
			return output;
		}

		key = keys[ikey++] = "acqTime";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "* Acq (ms) Tnow=[%ld] Tout=[%ld] Rec=[%ld] Xfer=[%ld]\n", 
						m_pcoData->msAcqTnow, m_pcoData->msAcqTout, m_pcoData->msAcqRec, m_pcoData->msAcqXfer);
			return output;
		}

		key = keys[ikey++] = "allocatedBuffer";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "AllocatedBuffer: Done=[%d] Nr=[%d] Size=[%ld][%g MB]\n", 
				m_pcoData->bAllocatedBufferDone, 
				m_pcoData->iAllocatedBufferNumber, 
				m_pcoData->dwAllocatedBufferSize, m_pcoData->dwAllocatedBufferSize/1000000.);
			
			return output;
		}

		key = keys[ikey++] = "testCmd";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){

			if((tokNr == 2) &&  (_stricmp(tok[1], "time")==0)){
				ptr += sprintf_s(ptr, ptrMax - ptr, "sleeping\n"); 
				::Sleep(atoi(tok[2])*1000);
				ptr += sprintf_s(ptr, ptrMax - ptr, "sleeping\n"); 
			}

			if(tokNr == 0) {
				ptr += sprintf_s(ptr, ptrMax - ptr, "tokNr [%d] cmd [%s] No parameters", tokNr, cmd); 
			} else {
				ptr += sprintf_s(ptr, ptrMax - ptr, "tokNr [%d] cmd [%s]\n", tokNr, cmd); 
				for(int i = 1; i<= tokNr; i++) {
					ptr += sprintf_s(ptr, ptrMax - ptr, "tok [%d] [%s]\n", i, tok[i]); 
				}
			}
			return output;
		}


		key = keys[ikey++] = "rollingShutter";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			DWORD dwSetup; int error;

			if(_getCameraType() != CAMERATYPE_PCO_EDGE) {
				ptr += sprintf_s(ptr, ptrMax - ptr, "invalid cmd / only for EDGE");
				return output;
			}
			
			if(tokNr == 0) {
				_pco_GetCameraSetup(dwSetup, error);
				ptr += sprintf_s(ptr, ptrMax - ptr, "%d", dwSetup);
				return output;
			}

			if((tokNr != 1) || ((strcmp(tok[1],"0") != 0) && (strcmp(tok[1],"1") != 0))){
				ptr += sprintf_s(ptr, ptrMax - ptr, "syntax ERROR - %s <0 | 1>", cmd);
				return output;
			}
			
			dwSetup = atoi(tok[1]);

			_pco_SetCameraSetup(dwSetup, error);
			ptr += sprintf_s(ptr, ptrMax - ptr, "%d", dwSetup);
			return output;
		}


		key = keys[ikey++] = "cameraType";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "sn[%ld] type: cam[%x][%x]if[%x] ver: hw[%lx]fw[%lx]\n", 
				m_pcoData->stcCamType.dwSerialNumber, 
				m_pcoData->stcCamType.wCamType, 
				m_pcoData->stcCamType.wCamSubType, 
				m_pcoData->stcCamType.wInterfaceType,
				m_pcoData->stcCamType.dwHWVersion, 
				m_pcoData->stcCamType.dwFWVersion

				);
			
			return output;
		}



		key = keys[ikey++] = "?";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			for(int i = 0; i < ikey; i++) {
				ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", keys[i]);
			}
			ptr += sprintf_s(ptr, ptrMax - ptr, "--- nrCmds[%d][%d]\n", ikey, NRCMDS);
			return output;
		}


		sprintf_s(ptr, ptrMax - ptr, "ERROR unknown cmd [%s]", cmd);
		return output;
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
	//PCO_TRACE("PCO_SetTriggerMode") ;
	//DEB_TRACE() << DEB_VAR1(trigmode);

    //------------------------------------- acquire mode : ignore or not ext. signal

	WORD acqmode = m_sync->getPcoAcqMode();
	error = PcoCheckError(PCO_SetAcquireMode(m_handle, acqmode));
	if(error) return "PCO_SetAcquireMode";
   //PCO_TRACE("PCO_SetAcquireMode") ;
	//DEB_TRACE() << DEB_VAR1(acqmode);
	return fnId;
}



//=================================================================================================
//=================================================================================================
char * Camera::_pcoSet_Storage_subRecord_Mode(int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	    // ----------------------------------------- storage mode (recorder + sequence)
		// current storage mode
		// - 0x0000 = [recorder] mode
		// - 0x0001 = [FIFO buffer] mode
    m_pcoData->storage_mode = 0;
		// current recorder submode:
		// - 0x0000 = [sequence]
		// - 0x0001 = [ring buffer].
    m_pcoData->recorder_submode = 0;

    error = PcoCheckError(PCO_SetStorageMode(m_handle, m_pcoData->storage_mode));
	if(error) return "PCO_SetStorageMode";
    //PCO_TRACE("PCO_SetStorageMode") ;

    error = PcoCheckError(PCO_SetRecorderSubmode(m_handle, m_pcoData->recorder_submode));
	if(error) return "PCO_SetRecorderSubmode";
    //PCO_TRACE("PCO_SetRecorderSubmode") ;

	return fnId;
}


//=================================================================================================
//=================================================================================================
char* Camera::_pcoSet_Exposure_Delay_Time(int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;
    float factor;

	    DWORD dwExposure, dwDelay;
		WORD wExposure_base, wDelay_base;
        double _exposure, _delay, val;
		double maxDw = pow(2., 32) - 1.;
        m_sync->getExpTime(_exposure);
        m_sync->getLatTime(_delay);

		// exp/lat time is saved in s. PCO requires it expressed in ms(=2), us(=1), ns(=0)
		// test time expressed in ns(=0), us(=1), ms(=2) up not overflow max precision in 32 bits
        for (wExposure_base = 0; wExposure_base < 3; wExposure_base++) {  // base 0(ns), 1(us), 2(ms)
            factor = pow((float)10, (int) (wExposure_base * 3 - 9));		// factor 10E-9, 10E-6, 10E-3
            if ( (val = (_exposure / factor)) <= maxDw) {		// multiply by 10E9, 10E6, 10E3
                dwExposure = (DWORD) val;			// exposure max precision in 32 bits, exposure base 0(ns)  1(us)  2(ms)
                break;
            }
        }
        //====================================== TODO set/get the value of ccd.delay now is 0 
        for (wDelay_base = 0; wDelay_base < 3; wDelay_base++) {
            factor = pow((float) 10, (int) (wDelay_base * 3 - 9));
            if ( (val = (_delay / factor)) <= maxDw) {
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

//	switch (m_pcoData->interface_type) {
	switch (m_pcoData->stcCamType.wInterfaceType) {
        case INTERFACE_CAMERALINK:

            error = PcoCheckError(PCO_GetTransferParameter(m_handle, &m_pcoData->clTransferParam, sizeof(m_pcoData->clTransferParam)));
			if(error) return "PCO_GetTransferParameter";
            //PCO_TRACE("PCO_GetTransferParameter") ;


            if((m_pcoData->clTransferParam.baudrate != 115200) || (m_pcoData->clTransferParam.DataFormat != 2)) {
                m_pcoData->clTransferParam.baudrate=115200;
                m_pcoData->clTransferParam.DataFormat=2;

                error = PcoCheckError(PCO_SetTransferParameter(m_handle, &m_pcoData->clTransferParam, sizeof(m_pcoData->clTransferParam)));
				if(error) return "PCO_SetTransferParameter";
 				//PCO_TRACE("PCO_SetTransferParameter") ;
            }

            // ---- no break
            
// SC2_SDK_FUNC int WINAPI PCO_CamLinkSetImageParameters(HANDLE ph, WORD wxres, WORD wyres);
// Neccessary while using a CamLink interface
// If there is a change in buffer size (ROI, binning) this function has to be called 
// with the new x and y resolution. Additionally this function has to be called, if you
// switch to another camRAM segment and like to get images.
// In: HANDLE ph -> Handle to a previously opened camera.
//     WORD wxres -> X Resolution of the images to be transferred
//     WORD wyres -> Y Resolution of the images to be transferred
// Out: int -> Error message.
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

	error = PcoCheckError(PCO_GetSizes(m_handle, &m_pcoData->wXResActual, &m_pcoData->wYResActual, &m_pcoData->wXResMax, &m_pcoData->wYResMax));
    PCO_TRACE("PCO_GetSizes") ;

	error = PcoCheckError(PCO_GetPixelRate(m_handle, &m_pcoData->dwPixelRate));
    PCO_TRACE("PCO_GetPixelRate") ;

    error = PcoCheckError(PCO_GetTransferParameter(m_handle, &m_pcoData->clTransferParam, sizeof(m_pcoData->clTransferParam)));
	if(error) return "PCO_GetTransferParameter";


    m_pcoData->clTransferParam.baudrate = PCO_CL_BAUDRATE_115K2;

	switch(_getCameraType()) {
		case CAMERATYPE_PCO_DIMAX_STD:
		    m_pcoData->clTransferParam.baudrate = PCO_CL_BAUDRATE_115K2;
			//m_pcoData->clTransferParam.Transmit = 1;

			m_pcoData->clTransferParam.DataFormat=PCO_CL_DATAFORMAT_2x12; //=2
			break;

		case CAMERATYPE_PCO_EDGE:
		    m_pcoData->clTransferParam.baudrate = PCO_CL_BAUDRATE_115K2;
			m_pcoData->clTransferParam.Transmit = 1;

			if(m_pcoData->dwPixelRate == PCO_CL_PIXELCLOCK_95MHZ) {
				m_pcoData->clTransferParam.DataFormat=PCO_CL_DATAFORMAT_5x16 | SCCMOS_FORMAT_TOP_CENTER_BOTTOM_CENTER;
			m_pcoData->wLUT_Identifier = 0; // Switch LUT->of
				break;
			}
			
			if(m_pcoData->dwPixelRate != PCO_CL_PIXELCLOCK_286MHZ) {

				break;
			}

			m_pcoData->clTransferParam.DataFormat = PCO_CL_DATAFORMAT_5x12L | SCCMOS_FORMAT_TOP_CENTER_BOTTOM_CENTER;
			m_pcoData->wLUT_Identifier = 0x1612; //Switch LUT->sqrt

			break;

		default:

			break;
			

	}

    error = PcoCheckError(PCO_SetTransferParameter(m_handle, &m_pcoData->clTransferParam, sizeof(m_pcoData->clTransferParam)));
	if(error) return "PCO_SetTransferParameter";

	//*****************************************************************************************
	//SC2_SDK_FUNC int WINAPI PCO_SetActiveLookupTable(HANDLE ph,
	//	WORD        *wIdentifier,            // define LUT to be activated, 0x0000 for no LUT
	//  WORD        *wParameter);            // optional parameter
	//*****************************************************************************************

	if(_getCameraType() == CAMERATYPE_PCO_EDGE) {
		m_pcoData->wLUT_Parameter = 0; 
		error = PcoCheckError(PCO_SetActiveLookupTable(m_handle, &m_pcoData->wLUT_Identifier, &m_pcoData->wLUT_Parameter));
		if(error) return "PCO_SetActiveLookupTabler";
	}


	//*****************************************************************************************
	// SC2_SDK_FUNC int WINAPI PCO_CamLinkSetImageParameters(HANDLE ph, WORD wxres, WORD wyres);
	// Neccessary while using a CamLink interface
	// If there is a change in buffer size (ROI, binning) this function has to be called 
	// with the new x and y resolution. Additionally this function has to be called, if you
	// switch to another camRAM segment and like to get images.
	// In: HANDLE ph -> Handle to a previously opened camera.
	//     WORD wxres -> X Resolution of the images to be transferred
	//     WORD wyres -> Y Resolution of the images to be transferred
	// Out: int -> Error message.
	//*****************************************************************************************

	error = PcoCheckError(PCO_CamLinkSetImageParameters(m_handle, m_pcoData->wXResActual, m_pcoData->wYResActual));
	if(error) return "PCO_CamLinkSetImageParameters";






	return fnId;
}


//=================================================================================================
//=================================================================================================
char *Camera::_pcoGet_Camera_Type(int &error){
	DEB_MEMBER_FUNCT();
	DEF_FNID;
	m_pcoData->frames_per_buffer = 1; // for PCO DIMAX

	// --- Get camera type
	{
		char *ptr;
		m_pcoData->stcCamType.wSize= sizeof(m_pcoData->stcCamType);
		error = PcoCheckError(PCO_GetCameraType(m_handle, &m_pcoData->stcCamType));
		if(error) return "PCO_GetCameraType";
		//PCO_TRACE("PCO_GetCameraType") ;

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

		//DEB_TRACE() <<   "m_pcoData->camera_name " << m_pcoData->camera_name ;	
	}

	// -- Reset to default settings
	error = PcoCheckError(PCO_ResetSettingsToDefault(m_handle));
	if(error) return "PCO_ResetSettingsToDefault";
	//PCO_TRACE("PCO_ResetSettingsToDefault") ;

	// -- Get camera description
	m_pcoData->pcoInfo.wSize= sizeof(m_pcoData->pcoInfo);

	error = PcoCheckError(PCO_GetCameraDescription(m_handle, &m_pcoData->pcoInfo));
	if(error) return "PCO_GetCameraDescription";
	//PCO_TRACE("PCO_GetCameraDescription") ;


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
	//PCO_TRACE("PCO_GetTemperature") ;

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
		//PCO_TRACE("PCO_GetCoolingSetpointTemperature") ;
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

	WORD wRecState_new, wRecState_actual;

	wRecState_new = state ? 0x0001 : 0x0000 ; // 0x0001 => START acquisition

	/************************************************************************************************
	SC2_SDK_FUNC int WINAPI PCO_SetRecordingState(HANDLE ph, WORD wRecState)
		· WORD wRecState: WORD to set the active recording state.
			- 0x0001 = camera is running, in recording status = [run]
			- 0x0000 = camera is idle or [stop]’ped, not ready to take images
	**************************************************************************************************/
	error = PcoCheckError(PCO_GetRecordingState(m_handle, &wRecState_actual));
	if(error) return "PCO_GetRecordingState";

	if(wRecState_new == wRecState_actual) {
		error = 0;
		return fnId;
	}

	/************************************************************************************************
	SC2_SDK_FUNC int WINAPI PCO_SetRecordingState(HANDLE ph, WORD wRecState)
		· WORD wRecState: WORD to set the active recording state.
			- 0x0001 = camera is running, in recording status = [run]
			- 0x0000 = camera is idle or [stop]’ped, not ready to take images
	**************************************************************************************************/

	error = PcoCheckError(PCO_SetRecordingState(m_handle, wRecState_new));
	if(error) return "PCO_SetRecordingState";

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
	if(error) return "PCO_GetCOCRuntime";

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

	//***************************************************************************************************
	// SC2_SDK_FUNC int WINAPI PCO_SetMetaDataMode(HANDLE ph, WORD wMetaDataMode, WORD* wMetaDataSize,
	//                                            WORD* wMetaDataVersion);
	// This option is only available with pco.dimax
	// In: HANDLE ph -> Handle to a previously opened camera.
	//     WORD  wMetaDataMode -> WORD variable to set the meta data mode.
	//     WORD* wMetaDataSize -> Pointer to a WORD variable receiving the meta data size.
	//     WORD* wMetaDataVersion -> Pointer to a WORD variable receiving the meta data version.
	// Out: int -> Error message.
	//***************************************************************************************************
	
	m_pcoData->wMetaDataMode = wMetaDataMode;
    error = PcoCheckError(PCO_SetMetaDataMode(m_handle, wMetaDataMode, &m_pcoData->wMetaDataSize, &m_pcoData->wMetaDataVersion));
	if(error) return "PCO_SetMetaDataMode";

	return fnId;

}


//=================================================================================================
//=================================================================================================
char *Camera::_pco_SetCameraSetup(DWORD dwSetup, int &error){
		
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	DWORD m_dwSetup[10];
	WORD m_wLen = 10;
	WORD m_wType;
	int ts[3] = { 2000, 3000, 250}; // command, image, channel timeout

	//***************************************************************************************************
	// SC2_SDK_FUNC int WINAPI PCO_GetCameraSetup(HANDLE ph, WORD *wType, DWORD *dwSetup, WORD *wLen);
	// Gets the camera setup structure (see camera specific structures)
	// Not applicable to all cameras.
	// See sc2_defs.h for valid flags: -- Defines for Get / Set Camera Setup
	// In: HANDLE ph -> Handle to a previously opened camera.
	//     WORD* wType -> Pointer to a word to get the actual type (Can be NULL to query wLen).
	//     DWORD* dwSetup -> Pointer to a dword array (Can be NULL to query wLen)
	//     WORD *wLen -> WORD Pointer to get the length of the array
	// Out: int -> Error message.
	//***************************************************************************************************

	//***************************************************************************************************
	// SC2_SDK_FUNC int WINAPI PCO_SetCameraSetup(HANDLE ph, WORD wType, DWORD *dwSetup, WORD wLen);
	// Sets the camera setup structure (see camera specific structures)
	// Camera must be switched off do activate new setup
	// Not applicable to all cameras.
	// See sc2_defs.h for valid flags: -- Defines for Get / Set Camera Setup
	// In: HANDLE ph -> Handle to a previously opened camera.
	//     WORD wType -> Word to set the actual type
	//     DWORD* dwSetup -> Pointer to a dword array
	//     WORD wLen -> WORD to set the length of the array
	// Out: int -> Error message.
	//***************************************************************************************************

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


    error = PcoCheckError(PCO_GetCameraSetup(m_handle, &m_wType, &m_dwSetup[0], &m_wLen));
	if(error) return "PCO_GetCameraSetup";

	if(m_dwSetup[0] == dwSetup) return fnId;

	m_dwSetup[0] = dwSetup;

    error = PcoCheckError(PCO_SetTimeouts(m_handle, &ts[0], sizeof(ts)));
	if(error) return "PCO_SetTimeouts";

    error = PcoCheckError(PCO_SetCameraSetup(m_handle, m_wType, &m_dwSetup[0], m_wLen));
	if(error) return "PCO_SetCameraSetup";

    error = PcoCheckError(PCO_RebootCamera(m_handle));
	if(error) return "PCO_RebootCamera";

    error = PcoCheckError(PCO_CloseCamera(m_handle));
	if(error) return "PCO_CloseCamera";


	return fnId;

}

char *Camera::_pco_GetCameraSetup(DWORD &dwSetup, int &error){
		
	DEB_MEMBER_FUNCT();
	DEF_FNID;

	DWORD m_dwSetup[10];
	WORD m_wLen = 10;
	WORD m_wType;

	//***************************************************************************************************
	// SC2_SDK_FUNC int WINAPI PCO_GetCameraSetup(HANDLE ph, WORD *wType, DWORD *dwSetup, WORD *wLen);
	// Gets the camera setup structure (see camera specific structures)
	// Not applicable to all cameras.
	// See sc2_defs.h for valid flags: -- Defines for Get / Set Camera Setup
	// In: HANDLE ph -> Handle to a previously opened camera.
	//     WORD* wType -> Pointer to a word to get the actual type (Can be NULL to query wLen).
	//     DWORD* dwSetup -> Pointer to a dword array (Can be NULL to query wLen)
	//     WORD *wLen -> WORD Pointer to get the length of the array
	// Out: int -> Error message.
	//***************************************************************************************************


    error = PcoCheckError(PCO_GetCameraSetup(m_handle, &m_wType, &m_dwSetup[0], &m_wLen));
	if(error) return "PCO_GetCameraSetup";

	dwSetup = m_dwSetup[0];
	return fnId;

}

