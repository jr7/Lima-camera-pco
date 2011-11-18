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

#include <cstdlib>
#include <process.h>

#include <sys/timeb.h>
#include <time.h>


#include "Exceptions.h"

#include "PcoCamera.h"
#include "PcoSyncCtrlObj.h"

using namespace lima;
using namespace lima::Pco;




static char *timebaseUnits[] = {"ns", "us", "ms"};



void pco_acq_thread(void *argin);
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
		sprintf_s(buff, BUFF_XLAT_SIZE, "UNKNOWN INTERFACE [0x%04x]", code);
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
	//err = ctime_s(timebuf, 26, &ltime);
	
	err = localtime_s( &today, &ltime );

	strftime(timeline, 128, fmt, &today );
      
	return timeline;
}

//=========================================================================================================
//=========================================================================================================
Camera::Camera(const char *ip_addr) :
  m_cam_connected(false),
  m_sync(NULL)
  //m_video(NULL)
{
  DEB_CONSTRUCTOR();
	char msg[MSG_SIZE + 1];
  int error=0;
  DWORD _dwValidImageCnt, _dwMaxImageCnt;

  // Init Frames
  //m_frame[0].ImageBuffer = NULL;
  //m_frame[0].Context[0] = this;
  //m_frame[1].ImageBuffer = NULL;
  //m_frame[1].Context[0] = this;
  
  m_pcoData.camera_name[0] = m_pcoData.sensor_type[0] = '\0';

  //m_acq_mode = 0;
  m_pcoData.storage_mode = 0;
  m_pcoData.recorder_submode = 0;
  m_bin.changed = Invalid;
	m_roi.changed = Invalid;

	log.clear();
	log.append("\n");




  m_pcoData.cocRunTime = 0;		/* cam operation code - delay & exposure time & readout in s*/
	m_pcoData.frameRate = 0;


	// block of creating the events passed to constructor of BufferCtrlObj
		//m_allocatedBufferNr[i] = -1;
		//m_allocatedBufferPtr[i]	= NULL;
    //m_allocatedBufferEvent[i] = CreateEvent( 


  DebParams::checkInit();

	// --- Open Camera
	error = PcoCheckError(PCO_OpenCamera(&m_handle, 0));
	PCO_TRACE("PCO_OpenCamera") ;
  

	// --- Get camera type
	{
		char *ptr;
		m_pcoData.stcCamType.wSize= sizeof(m_pcoData.stcCamType);
		error = PcoCheckError(PCO_GetCameraType(m_handle, &m_pcoData.stcCamType));
		PCO_TRACE("PCO_GetCameraType") ;

		ptr = xlatPcoCode2Str(m_pcoData.stcCamType.wCamType, ModelType, error);
		strcpy_s(m_pcoData.model, MODEL_TYPE_SIZE, ptr);
		DEB_TRACE() <<   "m_pcoData.model " << m_pcoData.model;
		if(error) throw LIMA_HW_EXC(Error, "Unknow model");
		
		ptr = xlatPcoCode2Str(m_pcoData.stcCamType.wInterfaceType, InterfaceType, error);
		strcpy_s(m_pcoData.iface, INTERFACE_TYPE_SIZE, ptr);
		DEB_TRACE() <<   "m_pcoData.iface " << m_pcoData.iface;
		if(error) throw LIMA_HW_EXC(Error, "Unknow interface");

		sprintf_s(m_pcoData.camera_name, CAMERA_NAME_SIZE, "%s %s", m_pcoData.model, m_pcoData.iface);

		DEB_TRACE() <<   "m_pcoData.camera_name " << m_pcoData.camera_name ;	
	}

	// -- Reset to default settings
	error = PcoCheckError(PCO_ResetSettingsToDefault(m_handle));
	PCO_TRACE("PCO_ResetSettingsToDefault") ;

	// -- Get camera description
	m_pcoData.pcoInfo.wSize= sizeof(m_pcoData.pcoInfo);

	error = PcoCheckError(PCO_GetCameraDescription(m_handle, &m_pcoData.pcoInfo));
	PCO_TRACE("PCO_GetCameraDescription") ;



  // PvAttrUint32Get(m_handle, "SensorWidth", &m_maxwidth);
  // PvAttrUint32Get(m_handle, "SensorHeight", &m_maxheight);

  //DEB_TRACE() << DEB_VAR2(m_maxwidth,m_maxheight);


		// -- Initialise adc, size, bin, roi
	m_pcoData.nr_adc= 1;
	m_pcoData.max_adc = m_pcoData.pcoInfo.wNumADCsDESC;

	m_size.maxwidth = (unsigned int) m_pcoData.pcoInfo.wMaxHorzResStdDESC; // ds->ccd.size.xmax,
	m_size.maxheight= (unsigned int) m_pcoData.pcoInfo.wMaxVertResStdDESC; // ds->ccd.size.ymax,
	m_size.pixbits = (unsigned int) m_pcoData.pcoInfo.wDynResDESC; // ds->ccd.size.bits
	m_size.pixbytes = (m_size.pixbits <= 8)?1:2; // nr de bytes por pixel  12 bits -> 2 bytes

	m_allocatedBufferSizeMax = m_size.maxwidth * m_size.maxheight * m_size.pixbytes;

	m_pcoData.maxwidth_step= (unsigned int) m_pcoData.pcoInfo.wRoiHorStepsDESC;   // ds->ccd.roi.xstep
	m_pcoData.maxheight_step= (unsigned int) m_pcoData.pcoInfo.wRoiVertStepsDESC; // ds->ccd.roi.ystep,

	m_roi.x[0] = m_roi.y[0] = 1;
	m_roi.x[1] = m_size.maxwidth;
	m_roi.y[1] = m_size.maxheight;
	m_roi.changed = Changed;

	sprintf_s(msg, MSG_SIZE, "* CCD Size = X[%d] x Y[%d] (%d bits)\n", m_size.maxwidth, m_size.maxheight, m_size.pixbits);
	DEB_TRACE() <<   msg;
	log.append(msg);
	
	sprintf_s(msg, MSG_SIZE, "* ROI Steps = x:%d, y:%d\n", m_pcoData.maxwidth_step, m_pcoData.maxheight_step);
	DEB_TRACE() <<   msg;
	log.append(msg);


	// -- Print out current temperatures
	error = PcoCheckError(PCO_GetTemperature(m_handle, &m_pcoData.temperature.wCcd, &m_pcoData.temperature.wCam, &m_pcoData.temperature.wPower));
	PCO_TRACE("PCO_GetTemperature") ;

	//dprintf("<%s> * CCD temperature = %.1f", fnId, ccdTemp/10.);
	sprintf_s(msg, MSG_SIZE, "* temperature: CCD[%.1f]  CAM[%d]  PS[%d]\n", m_pcoData.temperature.wCcd/10., m_pcoData.temperature.wCam, m_pcoData.temperature.wPower);
	DEB_TRACE() <<   msg;
	log.append(msg);


	m_pcoData.temperature.wMinCoolSet = m_pcoData.pcoInfo.sMinCoolSetDESC;
	m_pcoData.temperature.wMaxCoolSet = m_pcoData.pcoInfo.sMaxCoolSetDESC;

	sprintf_s(msg, MSG_SIZE, "* cooling temperature: MIN [%d]  Max [%d]\n",  m_pcoData.temperature.wMinCoolSet, m_pcoData.temperature.wMaxCoolSet);
	DEB_TRACE() <<   msg;
	log.append(msg);

	// -- Set/Get cooling temperature
	if (m_pcoData.temperature.wSetpoint != -1) {
		if (m_pcoData.temperature.wSetpoint < m_pcoData.temperature.wMinCoolSet)	m_pcoData.temperature.wSetpoint = m_pcoData.temperature.wMinCoolSet;
		if (m_pcoData.temperature.wSetpoint > m_pcoData.temperature.wMaxCoolSet)	m_pcoData.temperature.wSetpoint= m_pcoData.temperature.wMaxCoolSet;
	} else {
		error = PcoCheckError(PCO_GetCoolingSetpointTemperature(m_handle, &m_pcoData.temperature.wSetpoint));
		PCO_TRACE("PCO_GetCoolingSetpointTemperature") ;
	}
	sprintf_s(msg, MSG_SIZE, "* Cooling Setpoint = %d\n", m_pcoData.temperature.wSetpoint);
	DEB_TRACE() <<   msg;
	log.append(msg);



	DEB_TRACE() <<  "end block 0";

// block #1 -- Get RAM size
	{
		int segmentPco, segmentArr;

		DWORD ramSize;
		WORD pageSize;
		
		error = PcoCheckError(PCO_GetCameraRamSize(m_handle, &ramSize, &pageSize));
		PCO_TRACE("PCO_GetCameraRamSize") ;

		m_pcoData.dwRamSize = ramSize;     // nr of pages of the ram
		m_pcoData.wPixPerPage = pageSize;    // nr of pixels of the page

	sprintf_s(msg, MSG_SIZE, "* RAM number of pages [%ld]  PAGE number of pixels [%d]\n",  m_pcoData.dwRamSize, m_pcoData.wPixPerPage);
	DEB_TRACE() <<   msg;
	log.append(msg);


		// ----------------- get initial seg Size - images & print

		// ---- get the size in pages of each of the 4 segments
		error = PcoCheckError(PCO_GetCameraRamSegmentSize(m_handle, segSize));
		PCO_TRACE("PCO_GetCameraRamSegmentSize") ;

		for(segmentArr=0; segmentArr < PCO_MAXSEGMENTS ; segmentArr++) {
			segmentPco = segmentArr +1;		// PCO segment (1 ... 4)
			m_pcoData.dwSegmentSize[segmentArr] = segSize[segmentArr];

			sprintf_s(msg, MSG_SIZE, "* segment[%d] number of pages[%ld]\n", segmentPco,m_pcoData.dwSegmentSize[segmentArr]);
			DEB_TRACE() <<   msg;
			log.append(msg);

		}

		//---- get nr de images in each segment & nr max of img on each segmente
		for(segmentArr=0;  segmentArr< PCO_MAXSEGMENTS ; segmentArr++) {
			segmentPco = segmentArr +1;

			error = PcoCheckError(PCO_GetNumberOfImagesInSegment(m_handle, segmentPco, &_dwValidImageCnt, &_dwMaxImageCnt));
			PCO_TRACE("PCO_GetNumberOfImagesInSegment") ;

			dwValidImageCnt[segmentArr] = _dwValidImageCnt;
			dwMaxImageCnt[segmentArr] = _dwMaxImageCnt;

			sprintf_s(msg, MSG_SIZE, "* segment[%d] nr images [%ld]  max imag [%ld]\n", segmentPco, _dwValidImageCnt,  _dwMaxImageCnt);
			DEB_TRACE() <<   msg;
			log.append(msg);

		} // for	


		// set the first segment to the max ram size, the others = 0
		// This function will result in all segments being cleared. 
		// All previously recorded images will be lost!

		//m_pcoData.dwSegmentSize[0] = m_pcoData.dwRamSize;
		for(segmentArr=1;  segmentArr < PCO_MAXSEGMENTS ; segmentArr++) {
			m_pcoData.dwSegmentSize[0] += m_pcoData.dwSegmentSize[segmentArr];
			m_pcoData.dwSegmentSize[segmentArr] = 0;
		}
		sprintf_s(msg, MSG_SIZE, "* m_pcoData.dwSegmentSize0 [%ld]  m_pcoData.dwRamSize [%ld]\n", m_pcoData.dwSegmentSize[0], m_pcoData.dwRamSize);
		DEB_TRACE() <<   msg;
		log.append(msg);


		error = PcoCheckError(PCO_SetCameraRamSegmentSize(m_handle, &m_pcoData.dwSegmentSize[0]));
		PCO_TRACE("PCO_SetCameraRamSegmentSize") ;
	}  // block #1 

	DEB_TRACE() <<  "end block 1 / get initial seg Size - images";


	{
		int segmentPco, segmentArr;
		DWORD pages_per_image = m_size.maxwidth * m_size.maxheight / m_pcoData.wPixPerPage;

		///------------------------------------------------------------------------TODO ?????
		for(segmentArr=0; segmentArr < PCO_MAXSEGMENTS ; segmentArr++) {
			segmentPco = segmentArr +1;
			if(dwMaxImageCnt[segmentArr] == 0){
				dwMaxImageCnt[segmentArr] = m_pcoData.dwSegmentSize[segmentArr] / pages_per_image;
				if(dwMaxImageCnt[segmentArr] > 4) dwMaxImageCnt[segmentArr] -= 2;
			}

			//dprintf("<%s> * segment[%d] nr images [%ld]  max imag [%ld]", fnId, segmentPco, dwValidImageCnt, dwMaxImageCnt);

		}	

		for(segmentArr=0; segmentArr < PCO_MAXSEGMENTS ; segmentArr++) {
			segmentPco = segmentArr +1;
			//dprintf("<%s> ** segment[%d] pages [%ld] nr images [%ld]  max imag [%ld]", fnId, segmentPco, ds->ccd.dwSegmentSize[segmentArr],ss->ccd.dwValidImageCnt[segmentArr], ds->ccd.dwMaxImageCnt[segmentArr]);
		}	

	} // block





	// -- Get Active RAM segment 

		error = PcoCheckError(PCO_GetActiveRamSegment(m_handle, &m_pcoData.activeRamSegment));
		PCO_TRACE("PCO_GetActiveRamSegment") ;


	//dprintf("<%s> * Active RAM segment = %d", fnId, ds->ccd.activeRamSegment);

	//getNrImagesSegment(ds, ds->ccd.activeRamSegment, &dwValidImageCnt, &dwMaxImageCnt, error);

		error = PcoCheckError(PCO_GetNumberOfImagesInSegment(m_handle, m_pcoData.activeRamSegment, &_dwValidImageCnt, &_dwMaxImageCnt));
		PCO_TRACE("PCO_GetNumberOfImagesInSegment") ;



		//dprintf("<%s> * segment[%d] nr images [%ld]  max imag [%ld]", fnId, wSegment, *dwValidImageCnt, *dwMaxImageCnt);







	//dprintf("<%s> DONE", fnId);
	//return (DS_OK);


	DEB_TRACE() <<  "original DONE";


	m_cam_connected = 1;
	error = 0;

  // m_cam_connected = !PvCameraOpenByAddr(ip,ePvAccessMaster,&m_handle);
  if(!m_cam_connected)
    throw LIMA_HW_EXC(Error, "Camera not found!");

  // unsigned long psize;
  // PvAttrStringGet(m_handle, "CameraName", m_pcoData.camera_name, 128, &psize);
  // PvAttrUint32Get(m_handle, "UniqueId", &m_uid);
  // PvAttrUint32Get(m_handle, "FirmwareVerMajor", &m_ufirmware_maj);
  // PvAttrUint32Get(m_handle, "FirmwareVerMinor", &m_ufirmware_min);
  // PvAttrEnumGet(m_handle, "SensorType", m_pcoData.sensor_type, 
//		sizeof(m_pcoData.sensor_type), &psize);

  DEB_TRACE() << DEB_VAR1(m_pcoData.camera_name);
  DEB_TRACE() << DEB_VAR2(m_size.maxwidth, m_size.maxheight);


  // error = PvAttrUint32Set(m_handle,"Width",m_maxwidth);
  if(error)
    throw LIMA_HW_EXC(Error,"Can't set image width");
  
  // error = PvAttrUint32Set(m_handle,"Height",m_maxheight);
  if(error)
    throw LIMA_HW_EXC(Error,"Can't set image height");
  
	// m_sync = new SyncCtrlObj(this, NULL); NO!
  
  // error = PvAttrEnumSet(m_handle, "AcquisitionMode", "Continuous");
  if(error)
    throw LIMA_HW_EXC(Error,"Can't set acquisition mode to continuous");

  DEB_TRACE() << log;
  DEB_TRACE() << "END OF CAMERA";

}



Camera::~Camera()
{
  DEB_DESTRUCTOR();


  DEB_TRACE() << "~Camera";

#ifndef BYPASS
  if(m_cam_connected)
    {
      PvCommandRun(m_handle,"AcquisitionStop");
      PvCaptureEnd(m_handle);
      PvCameraClose(m_handle);
    }
  PvUnInitialize();
  if(m_frame[0].ImageBuffer)
    free(m_frame[0].ImageBuffer);
  if(m_frame[1].ImageBuffer)
    free(m_frame[1].ImageBuffer);
#endif
}

/** @brief test if the camera is monochrome
 */



void Camera::getCameraName(std::string& name)
{
  DEB_MEMBER_FUNCT();
  DEB_RETURN() << DEB_VAR1(m_pcoData.camera_name);

  name = m_pcoData.camera_name;
}


#ifndef BYPASS
void Camera::_allocBuffer()
{
  DEB_MEMBER_FUNCT();

  tPvUint32 imageSize;
  tPvErr error = PvAttrUint32Get(m_handle, "TotalBytesPerFrame", &imageSize);
  if(error)
    throw LIMA_HW_EXC(Error,"Can't get camera image size");

  DEB_TRACE() << DEB_VAR1(imageSize);
  //realloc
  if(!m_frame[0].ImageBuffer || m_frame[0].ImageBufferSize < imageSize)
    {
      //Frame 0
      m_frame[0].ImageBuffer = realloc(m_frame[0].ImageBuffer,
				       imageSize);
      m_frame[0].ImageBufferSize = imageSize;

      //Frame 1
      m_frame[1].ImageBuffer = realloc(m_frame[1].ImageBuffer,
				       imageSize);

      m_frame[1].ImageBufferSize = imageSize;
    }
}
/** @brief start the acquisition.
    must have m_video != NULL and previously call _allocBuffer
*/
#endif

//=================================================================================================
//=================================================================================================
void Camera::startAcq()
{
    DEB_MEMBER_FUNCT();

	m_acq_frame_nb = 0;

#ifndef BYPASS
  m_continue_acq = true;
  m_acq_frame_nb = 0;
  tPvErr error = PvCaptureQueueFrame(m_handle,&m_frame[0],_newFrameCBK);

  int requested_nb_frames;
  m_sync->getNbFrames(requested_nb_frames);
  bool isLive;
  m_video->getLive(isLive);

  if(!requested_nb_frames || requested_nb_frames > 1 || isLive)
    error = PvCaptureQueueFrame(m_handle,&m_frame[1],_newFrameCBK);
#endif

#define DWORD_MAX 0xffffffff 
//=====================================================================
    char *fnId = "StartAcq";
    unsigned long size;
    WORD state;
    HANDLE hEvent= NULL;
    //float factor;
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
    msg = _pcoSet_Storage_subRecord_Mode(error);
    PCO_TRACE(msg) ;

	//----------------------------------- set exposure time & delay time
	msg = _pcoSet_Exposure_Delay_Time(error);
	PCO_TRACE(msg) ;


    //------------------------------------------------- check recording state

    error = PcoCheckError(PCO_GetRecordingState(m_handle, &state));
    PCO_TRACE("PCO_GetRecordingState") ;

    if (state>0) {
        DEB_TRACE() << "Force recording state to 0x0000" ;

        error = PcoCheckError(PCO_SetRecordingState(m_handle, 0x0000));
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

    // ------------------------------------------------- arm camera
    error = PcoCheckError(PCO_ArmCamera(m_handle));
    PCO_TRACE("PCO_ArmCamera") ;


        //====================================== get the coc runtime 
        //---- only valid if it was used PCO_SetDelayExposureTime
        //---- and AFTER armed the cam
    {
        DWORD dwTime_s, dwTime_ns;
        double runTime;

        error = PcoCheckError(PCO_GetCOCRuntime(m_handle, &dwTime_s, &dwTime_ns));
        PCO_TRACE("PCO_GetCOCRuntime") ;

        m_pcoData.cocRunTime = runTime = ((double) dwTime_ns * 1.0E-9) + (double) dwTime_s;
        m_pcoData.frameRate = (dwTime_ns | dwTime_s) ? 1.0 / runTime : 0.0;

        DEB_TRACE() << DEB_VAR2(m_pcoData.frameRate, m_pcoData.cocRunTime);
    } // block


    //====================================== get exp time

    {
        DWORD exposure, delay;
        WORD wExposure_base, wDelay_base;

        error = PcoCheckError(PCO_GetDelayExposureTime(m_handle, &delay, &exposure, &wDelay_base, &wExposure_base));
        PCO_TRACE("PCO_GetDelayExposureTime") ;

	    DEB_TRACE() << DEB_VAR2(delay, timebaseUnits[wDelay_base]);
		DEB_TRACE() << DEB_VAR2(exposure, timebaseUnits[wExposure_base]);
    }


    //------------------------------------------------ get size
	//   int err = PCO_GetSizes(hCamera, &wXResActual, &wYResActual, &wXResMax, &wYResMax);

    WORD xmax, ymax;
	
	error = PcoCheckError(PCO_GetSizes(m_handle, &m_size.armwidth, &m_size.armheight, &xmax, &ymax));
    PCO_TRACE("PCO_GetSizes") ;

    //m_size.armwidth= xactualsize;
    //m_size.armheight= yactualsize;
    size= m_size.armwidth * m_size.armheight * m_size.pixbytes;

    m_allocatedBufferSize= m_allocatedBufferSizeMax;

	//------------------------------------------------- allocate buffer if not yet done, or size changed
	//if ((m_allocatedBufferNr[0]==-1) || (size != m_imgsizeBytes)) {
  // ---------------- data is useful??
    m_imgsizeBytes= size;
    m_imgsizePixels= m_size.armwidth * m_size.armheight;
    m_imgsizePages= m_imgsizePixels / m_pcoData.wPixPerPage;
    if(m_imgsizePixels % m_pcoData.wPixPerPage) m_imgsizePages++;
    m_imgsizePages++;
    m_imgsizeBuffer = m_imgsizePages * m_pcoData.wPixPerPage * m_size.pixbytes;
    m_allocatedBufferSize= m_allocatedBufferSizeMax;

    //------------------------------------------------- set image size for CamLink and GigE
	msg = _pcoSet_Cameralink_GigE_Parameters(error);
	PCO_TRACE(msg) ;

    //------------------------------------------------- checking nr of frames
    {
        int segmentPco, segmentArr;
        unsigned long frames, framesMax;
        int iFrames;

        segmentPco = m_pcoData.activeRamSegment;
        segmentArr = segmentPco-1;

        m_sync->getNbFrames(iFrames);

        //frames = m_frame.nb;
        frames = iFrames;
        framesMax = _getFramesMax(segmentPco);

        //dprintf2("<%s> --- checking frames frames[%ld] framesMax[%ld]", fnId, frames, framesMax);

        if ((frames > framesMax)) {
            throw LIMA_HW_EXC(Error, "frames OUT OF RANGE");

            //*error= DevErr_ValueOutOfBounds;
            //dprintf2("<%s> ERROR --- frames OUT OF RANGE  frames[%ld] framesMax[%ld]", fnId, frames, framesMax);
            //return (DS_NOTOK);
        }
    } // block


	//------------------------------------------------- start acquisition
	/************************************************************************************************
	SC2_SDK_FUNC int WINAPI PCO_SetRecordingState(HANDLE ph, WORD wRecState)
		· WORD wRecState: WORD to set the active recording state.
			- 0x0001 = camera is running, in recording status = [run]
			- 0x0000 = camera is idle or [stop]’ped, not ready to take images
	**************************************************************************************************/
	{
		WORD wRecState = 0x0000;   // 0x0001 => START acquisition  (0 for tests)     
		error = PcoCheckError(PCO_SetRecordingState(m_handle, wRecState));
		PCO_TRACE("PCO_SetRecordingState") ;
	}

    m_frame.done= 0;

	_beginthread( pco_acq_thread, 0, (void*) this);
	return;
}

//==========================================================================================================
//==========================================================================================================

long msElapsedTime(struct __timeb64 &t0) {
	struct __timeb64 tNow;
	_ftime64_s(&tNow);

	return (long)((tNow.time - t0.time)*1000) + (tNow.millitm - t0.millitm);
}

void pco_acq_thread(void *argin) {
	Camera* m_cam = (Camera *) argin;
	SyncCtrlObj* m_sync = m_cam->_getSyncCtrlObj();
	BufferCtrlObj* m_buffer = m_sync->_getBufferCtrlObj();

	struct stcPcoData *m_pcoData = m_cam->_getPcoData();
	
	char *msg;
	struct __timeb64 tStart;
	long timeout;
	int nb_acq_frames;

	HANDLE m_handle = m_cam->getHandle();
	
	int nb_frames; 	m_sync->getNbFrames(nb_frames);
	DWORD dwValidImageCnt, dwMaxImageCnt;
	WORD wSegment =m_pcoData->activeRamSegment; 
	DWORD dwMsSleep = (DWORD) (m_pcoData->cocRunTime * 1000.);
	if(dwMsSleep == 0) dwMsSleep = 1;

	timeout = (long) (dwMsSleep * (nb_frames * 1.1));
	dwValidImageCnt = 0;
	while(dwValidImageCnt <  (DWORD) nb_frames) {
		Sleep(dwMsSleep);	
		msg = _PcoCheckError(PCO_GetNumberOfImagesInSegment(m_handle, wSegment, &dwValidImageCnt, &dwMaxImageCnt));

		if(timeout < msElapsedTime(tStart)) { break; }
	}

	{
		WORD wRecState = 0x0000;   // 0x0000 => STOP acquisition  (0 for tests)     
		msg = _PcoCheckError(PCO_SetRecordingState(m_handle, wRecState));
		//PCO_TRACE("PCO_SetRecordingState") ;
	}


	msg = _PcoCheckError(PCO_GetNumberOfImagesInSegment(m_handle, wSegment, &dwValidImageCnt, &dwMaxImageCnt));

	nb_acq_frames = (dwValidImageCnt < (DWORD) nb_frames) ? dwValidImageCnt : nb_frames;
	m_sync->setAcqFrames(nb_acq_frames);

	m_buffer->_xferImag();

	_endthread();
}


//=====================================================================
//=====================================================================
void Camera::reset()
{
  DEB_MEMBER_FUNCT();
  //@todo maybe something to do!
}


#ifndef BYPASS
void Camera::_newFrameCBK(tPvFrame* aFrame)
{
  DEB_STATIC_FUNCT();
  Camera *aCamera = (Camera*)aFrame->Context[0];
  aCamera->_newFrame(aFrame);
}
#endif

#ifndef BYPASS
void Camera::_newFrame(tPvFrame* aFrame)
{
  DEB_MEMBER_FUNCT();

  if(!m_continue_acq) return;

  if(aFrame->Status != ePvErrSuccess)
    {
      if(aFrame->Status != ePvErrCancelled)
	{
	  DEB_WARNING() << DEB_VAR1(aFrame->Status);
	  PvCaptureQueueFrame(m_handle,aFrame,_newFrameCBK);
	}
      return;
    }
  
  int requested_nb_frames;
  m_sync->getNbFrames(requested_nb_frames);
  bool isLive;
  m_video->getLive(isLive);
  ++m_acq_frame_nb;

  bool stopAcq = false;
  if(isLive || !requested_nb_frames || m_acq_frame_nb < (requested_nb_frames - 1))
    {
      if(isLive || requested_nb_frames ||
	 m_acq_frame_nb < (requested_nb_frames - 2))
	tPvErr error = PvCaptureQueueFrame(m_handle,aFrame,_newFrameCBK);
    }
  else
    stopAcq = true;
  
  VideoMode mode;
  switch(aFrame->Format)
    {
    case ePvFmtMono8: 	mode = Y8;		break;
    case ePvFmtMono16: 	mode = Y16;		break;
    case ePvFmtBayer8: 	mode = BAYER_RG8;	break;
    case ePvFmtBayer16: mode = BAYER_RG16;	break;
    default:
      DEB_ERROR() << "Format not supported: " << DEB_VAR1(aFrame->Format);
      m_sync->stopAcq();
      return;
    }

  m_continue_acq =  m_video->callNewImage((char*)aFrame->ImageBuffer,
					  aFrame->Width,
					  aFrame->Height,
					  mode);
  if(stopAcq || !m_continue_acq)
    m_sync->stopAcq(false);
}
#endif
int Camera::PcoCheckError(int err) {
	static char lastErrorMsg[500];
	if (err != 0) {
		PCO_GetErrorText(err, pcoErrorMsg, ERR_SIZE-14);
		//sprintf(lastErrorMsg, "<PCO ERROR> %s", pcoErrorMsg);
		//dprintf(lastErrorMsg);


		return (1);
	}
	return (0);
}


char* _PcoCheckError(int err) {
	static char lastErrorMsg[ERR_SIZE];
	if (err != 0) {
		PCO_GetErrorText(err, lastErrorMsg, ERR_SIZE-14);
		//sprintf(lastErrorMsg, "<PCO ERROR> %s", pcoErrorMsg);
		//dprintf(lastErrorMsg);


		return lastErrorMsg;
	}
	return NULL;
}


unsigned long Camera::_getFramesMax(int segmentPco){
		char *fnId = "_getFramesMax";

		int segmentArr = segmentPco-1;
		unsigned long framesMax;
		unsigned long xroisize,yroisize;
		unsigned long long pixPerFrame, pagesPerFrame;

		printf("%s> segmentPco[%d]\n", fnId, segmentPco);
		if((segmentPco <1) ||(segmentPco > PCO_MAXSEGMENTS)) {
			printf("%s> ERROR segmentPco[%d]\n", fnId, segmentPco);
			return -1;
		}

		xroisize = m_roi.x[1] - m_roi.x[0] + 1;
		yroisize = m_roi.y[1] - m_roi.y[0] + 1;

		pixPerFrame = (unsigned long long)xroisize * (unsigned long long)yroisize;

		printf("%s> pixPerFrame[%lld]\n", fnId, pixPerFrame);
		if(pixPerFrame <0) {
			printf("%s> ERROR pixPerFrame[%lld]\n", fnId, pixPerFrame);
			return -1;
		}

		printf("%s> m_pcoData.wPixPerPage[%d]\n", fnId, m_pcoData.wPixPerPage);
		if(m_pcoData.wPixPerPage < 1) {
			printf("%s> ERROR m_pcoData.wPixPerPage[%d]\n", fnId, m_pcoData.wPixPerPage);
			return -1;
		}
		pagesPerFrame = (pixPerFrame / m_pcoData.wPixPerPage) + 1;
		if(pixPerFrame % m_pcoData.wPixPerPage) pagesPerFrame++;

		framesMax = m_pcoData.dwMaxFramesInSegment[segmentArr] = (unsigned long)(((long long) m_pcoData.dwSegmentSize[segmentArr] ) / pagesPerFrame);

		//dprintf2("<%s> framesMax[%ld] segment[%d] ramPages[%ld] pageSize[%ld] roi[%ld / %ld]", fnId, framesMax, segmentPco, 
			//ds->ccd.dwSegmentSize[segmentArr], ds->ccd.wPageSize, xroisize, yroisize);

		

		printf("%s> framesMax[%ld]\n", fnId, framesMax);
		return framesMax;
	}




char *Camera::getInfo(char *output, int lg){
		char *ptr, *ptrMax;
		int segmentPco = m_pcoData.activeRamSegment;
		int segmentArr = segmentPco -1;

		ptr = output; *ptr = 0;
		ptrMax = ptr + lg;

		int width = +20;

		ptr += sprintf_s(ptr, ptrMax - ptr,"**** %s [begin]\n", __FUNCTION__);

		ptr += sprintf_s(ptr, ptrMax - ptr,"**** PCO log\n");
		ptr += sprintf_s(ptr, ptrMax - ptr,"%s\n", log.c_str());

		ptr += sprintf_s(ptr, ptrMax - ptr,"**** PCO Info\n");
		ptr += sprintf_s(ptr, ptrMax - ptr, "* timestamp[%s]\n", getTimestamp(Iso));

		ptr += sprintf_s(ptr, ptrMax - ptr, "* roi X(%d,%d) Y(%d,%d) size(%d,%d)\n",  
				m_roi.x[0], m_roi.x[1],
				m_roi.y[0], m_roi.y[1],
				m_roi.x[1] - m_roi.x[0] + 1, m_roi.y[1] - m_roi.y[0] + 1);

		ptr += sprintf_s(ptr, ptrMax - ptr, "* m_cocRunTime[%g s] m_frameRate[%g fps]\n",  m_pcoData.cocRunTime, m_pcoData.frameRate);

		double _exposure, _delay;
        m_sync->getExpTime(_exposure);
        m_sync->getLatTime(_delay);
		ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData.storage_mode[%d] m_pcoData.recorder_submode[%d]\n", m_pcoData.storage_mode, m_pcoData.recorder_submode);
		ptr += sprintf_s(ptr, ptrMax - ptr, "* _exposure[%g s] _delay[%g s]\n", _exposure, _delay);
		
		

		int iFrames;
		m_sync->getNbFrames(iFrames);
		ptr += sprintf_s(ptr, ptrMax - ptr, "* m_sync->getNbFrames[%d frames] m_pcoData.dwMaxFramesInSegment[][%d frames]\n", iFrames, m_pcoData.dwMaxFramesInSegment[segmentArr]);
		ptr += sprintf_s(ptr, ptrMax - ptr, "* PcoActiveSegment[%d] m_pcoData.dwSegmentSize[][%d pages] m_pcoData.wPixPerPage[%d pix]\n", segmentArr+1, m_pcoData.dwSegmentSize[segmentArr], m_pcoData.wPixPerPage);


/***
		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = [%s]\n", width,"fileDir", ds->ccd.file.image_dir);
		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = [%s]\n", width, "filePrefix", ds->ccd.file.image_prefix);
		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = [%s]\n", width, "fileSuffix", ds->ccd.file.image_suffix);
		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = [%s]\n", width, "fileNoFmt", ds->ccd.file.image_no_fmt);
		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = [%d]\n", width, "fileNo", ds->ccd.file.image_no);

		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = [%s]\n", width, "sinogram fileDir", ds->ccd.sinogram.fileDir);
		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = [%d][%d]\n", width, "sinogram cols", ds->ccd.sinogram.col_beg, ds->ccd.sinogram.col_end);
		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = [%d][%d]\n", width, "sinogram rows", ds->ccd.sinogram.row_beg, ds->ccd.sinogram.row_end);
		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = [%d]\n", width, "sinogram frames", ds->ccd.sinogram.nr_frames);
		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = [%d]\n", width, "sinogram saving", ds->ccd.sinogram.saving);

****/
		ptr += sprintf_s(ptr, ptrMax - ptr,"**** %s [end]\n", __FUNCTION__);
		return output;
}

//===================================================================================================================
//===================================================================================================================
// passed to bufferctrlobj  void Camera::_assignImage2Buffer(DWORD &dwFrameFirst, DWORD &dwFrameLast, DWORD dwRequestedFrames, int bufIdx) {
/*---------------------------------------------------------------------------
        Purpose:  Write file to disk in a separate thread
		Arguments:	argin = ccd ds struct
        Returns:    nothing
*---------------------------------------------------------------------------*/
// passed to bufferctrlobj void Camera::_xferImag()



//=================================================================================================
//=================================================================================================
char * Camera::_pcoSet_Trig_Acq_Mode(int &error){
	static char *fnId = "_pcoSet_Trig_Acq_Mode";
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
	static char *fnId = "_pcoSet_Storage_subRecord_Mode";

	    // ----------------------------------------- storage mode (recorder + sequence)
		// current storage mode
		// - 0x0000 = [recorder] mode
		// - 0x0001 = [FIFO buffer] mode
    m_pcoData.storage_mode = 0;
		// current recorder submode:
		// - 0x0000 = [sequence]
		// - 0x0001 = [ring buffer].
    m_pcoData.recorder_submode = 0;

    error = PcoCheckError(PCO_SetStorageMode(m_handle, m_pcoData.storage_mode));
	if(error) return "PCO_SetStorageMode";
    //PCO_TRACE("PCO_SetStorageMode") ;

    error = PcoCheckError(PCO_SetRecorderSubmode(m_handle, m_pcoData.recorder_submode));
	if(error) return "PCO_SetRecorderSubmode";
    //PCO_TRACE("PCO_SetRecorderSubmode") ;

	return fnId;
}


//=================================================================================================
//=================================================================================================
char* Camera::_pcoSet_Exposure_Delay_Time(int &error){
	static char *fnId = "_pcoSet_Exposure_Delay_Time";
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
		//DEB_TRACE() << DEB_VAR2(dwDelay, timebaseUnits[wDelay_base]);
		//DEB_TRACE() << DEB_VAR2(dwExposure, timebaseUnits[wExposure_base]);

		error = PcoCheckError(PCO_SetDelayExposureTime(m_handle, dwDelay, dwExposure, wDelay_base, wExposure_base));
		if(error) return "PCO_SetDelayExposureTime";
		//PCO_TRACE("PCO_SetDelayExposureTime") ;


	return fnId;
}

//=================================================================================================
//=================================================================================================
char *Camera::_pcoSet_Cameralink_GigE_Parameters(int &error){
	static char *fnId = "_pcoSet_Cameralink_GigE_Parameters";

	//------------------------------------------------- set image size for CamLink and GigE

//	switch (m_pcoData.interface_type) {
	switch (m_pcoData.stcCamType.wInterfaceType) {
        case INTERFACE_CAMERALINK:

            error = PcoCheckError(PCO_GetTransferParameter(m_handle, &m_pcoData.clTransferParam, sizeof(m_pcoData.clTransferParam)));
			if(error) return "PCO_GetTransferParameter";
            //PCO_TRACE("PCO_GetTransferParameter") ;

            //dprintf2("<%s> PCO_GetTransferParameter  baudrate[%d] ClockFrequency[%d] CCline[%d] DataFormat[%ld] Transmit[%d]", fnId, 
            //ds->ccd.clTransferParam.baudrate, ds->ccd.clTransferParam.ClockFrequency, ds->ccd.clTransferParam.CCline,
            //ds->ccd.clTransferParam.DataFormat, ds->ccd.clTransferParam.Transmit);

            if((m_pcoData.clTransferParam.baudrate != 115200) || (m_pcoData.clTransferParam.DataFormat != 2)) {
                //dprintf("<%s> ALERT - PCO_GetTransferParameter  baudrate[%d] DataFormat[%ld]", fnId, 
                //m_pcoData.clTransferParam.baudrate, m_pcoData.clTransferParam.DataFormat);
                m_pcoData.clTransferParam.baudrate=115200;
                m_pcoData.clTransferParam.DataFormat=2;

                error = PcoCheckError(PCO_SetTransferParameter(m_handle, &m_pcoData.clTransferParam, sizeof(m_pcoData.clTransferParam)));
				if(error) return "PCO_SetTransferParameter";
 				//PCO_TRACE("PCO_SetTransferParameter") ;
            }

            // ---- no break
            
        case INTERFACE_ETHERNET:
		    WORD xsent, ysent;

            xsent= m_size.armwidth;
            //ysent= (WORD) yactualsize * (WORD) ds->ccd.frames_per_buffer;
            ysent= m_size.armheight;

            //dprintf2("<%s> PCO_CamLinkSetImageParameters xact[%ld] xsent[%ld] yact[%ld] ysent[%ld] frames/buf[%d]", fnId, 
            //xactualsize, xsent, yactualsize, ysent, ds->ccd.frames_per_buffer);

            error = PcoCheckError(PCO_CamLinkSetImageParameters(m_handle, xsent, ysent));
			if(error) return "PCO_CamLinkSetImageParameters";

            break;

        default: break;
    } // case



	return fnId;
}