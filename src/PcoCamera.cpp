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



#include "Exceptions.h"

#include "PcoCamera.h"
#include "PcoSyncCtrlObj.h"

using namespace lima;
using namespace lima::Pco;

#define THROW_LIMA_HW_EXC(e, x)  { \
	printf("========*** LIMA_HW_EXC %s\n", x ); \
			throw LIMA_HW_EXC(e, x); \
} 


#define PCO_TRACE(x)  \
{ \
		if(error){ \
			DEB_TRACE() << "*** " <<  x << " PCO ERROR " << pcoErrorMsg; \
			throw LIMA_HW_EXC(Error, x); \
		} \
		DEB_TRACE() << "*** " <<  x << " OK" ; \
}



static char *timebaseUnits[] = {"ns", "us", "ms"};

struct stcXlatI2A modelType[] = {
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

struct stcXlatI2A interfaceType[] = {
	{INTERFACE_FIREWIRE, "FIREWIRE"},
	{INTERFACE_CAMERALINK, "CAMERALINK"},
	{INTERFACE_USB, "USB"},
	{INTERFACE_ETHERNET, "ETHERNET"},
	{INTERFACE_SERIAL, "SERIAL"},
	{0, NULL}
};

char *xlatI2A(int code, struct stcXlatI2A *stc) {

	char *type;

	while( (type = stc->str) != NULL) {
		if(stc->code == code) return type;
		stc++;
	}

	return NULL;

}

static char *getTimestamp(int fmtIdx) {
   static char timeline[128];
   errno_t err;
	time_t ltime;
	struct tm today;
	char *fmt[] = {"%Y/%m/%d %H:%M:%S", "%Y-%m-%d-%H%M%S" , "%Y-%m-%d"};


	time( &ltime );
	//err = ctime_s(timebuf, 26, &ltime);
	
	err = localtime_s( &today, &ltime );

	strftime(timeline, 128, fmt[fmtIdx], &today );
      
	return timeline;
}
Camera::Camera(const char *ip_addr) :
  m_cam_connected(false),
  m_sync(NULL)
  //m_video(NULL)
{
  DEB_CONSTRUCTOR();
	char msg[MSG_SIZE + 1];
  int error=0;
  char *ptr;
  DWORD _dwValidImageCnt, _dwMaxImageCnt;

  // Init Frames
  //m_frame[0].ImageBuffer = NULL;
  //m_frame[0].Context[0] = this;
  //m_frame[1].ImageBuffer = NULL;
  //m_frame[1].Context[0] = this;
  
  m_camera_name[0] = m_sensor_type[0] = '\0';

  m_acq_mode = 0;
  m_storage_mode = 0;
  m_recorder_submode = 0;
  m_bin.changed = Invalid;
	m_roi.changed = Invalid;



  m_cocRunTime = 0;		/* cam operation code - delay & exposure time & readout in s*/
	m_frameRate = 0;


	for(int i=0; i < 8; i++) {
		m_bufferNrM[i] = -1;
		m_bufferM[i]	= NULL;


	 // Create two event objects
        m_bufferM_events[i] = CreateEvent( 
            NULL,   // default security attributes
            FALSE,  // auto-reset event object
            FALSE,  // initial state is nonsignaled
            NULL);  // unnamed object

        if (m_bufferM_events[i] == NULL) 
        { 
            THROW_LIMA_HW_EXC(Error, "CreateEvent error")
        } 
    } 


  DebParams::checkInit();
  
  std::string name = "TEST";
  DEB_TRACE() <<   " TRACE 1 " << name.c_str(); //DebParams::getModuleName(lima::DebModCamera) ;

    ConstStr s, s1, s2;
    s =  DebParams::getFormatName(lima::DebFmtFunct) ;
    s1 =  DebParams::getModuleName(lima::DebModCamera) ;
    s2 =  DebParams::getTypeName(lima::DebTypeTrace) ;
  DEB_TRACE() <<   " TRACE 1a " << s;

  // tPvErr error = PvInitialize();
	// --- Open Camera
	error = PcoCheckError(PCO_OpenCamera(&m_handle, 0));

	PCO_TRACE("PCO_OpenCamera") ;



  

	// --- Get camera type
	m_stcCamType.wSize= sizeof(m_stcCamType);
	error = PcoCheckError(PCO_GetCameraType(m_handle, &m_stcCamType));
	PCO_TRACE("PCO_GetCameraType") ;

	if((ptr = xlatI2A(m_stcCamType.wCamType, modelType)) != NULL) {
		strcpy_s(m_model, MODEL_SIZE, ptr);	error= 0;
	} else {
		sprintf_s(m_model, MODEL_SIZE, "UNKNOWN [0x%04x]", m_stcCamType.wCamType); error= 1;
	}

	DEB_TRACE() <<   "m_model " << m_model;

	if(error) throw LIMA_HW_EXC(Error, "Unknow model");
	
	if((ptr = xlatI2A((m_interface_type = m_stcCamType.wInterfaceType), interfaceType)) != NULL) {
		strcpy_s(m_iface, MODEL_SIZE, ptr);	error= 0;
	} else {
		sprintf_s(m_iface, MODEL_SIZE, "UNKNOWN [0x%04x]", m_stcCamType.wInterfaceType); error= 1;
	}

	DEB_TRACE() <<   "m_iface " << m_iface;

	if(error) throw LIMA_HW_EXC(Error, "Unknow interface");

	sprintf_s(m_camera_name, CAMERA_SIZE, "%s %s", m_model, m_iface);

	DEB_TRACE() <<   "m_camera_name " << m_camera_name ;



	// -- Reset to default settings
	error = PcoCheckError(PCO_ResetSettingsToDefault(m_handle));
	PCO_TRACE("PCO_ResetSettingsToDefault") ;



	// -- Get camera description
	m_pcoInfo.wSize= sizeof(m_pcoInfo);

	error = PcoCheckError(PCO_GetCameraDescription(m_handle, &m_pcoInfo));
	PCO_TRACE("PCO_GetCameraDescription") ;



	  // PvAttrUint32Get(m_handle, "SensorWidth", &m_maxwidth);
  // PvAttrUint32Get(m_handle, "SensorHeight", &m_maxheight);

  //DEB_TRACE() << DEB_VAR2(m_maxwidth,m_maxheight);


		// -- Initialise adc, size, bin, roi
	m_nradc= 1;
	m_maxadc = m_pcoInfo.wNumADCsDESC;

	m_size.maxwidth = (unsigned int) m_pcoInfo.wMaxHorzResStdDESC; // ds->ccd.size.xmax,
	m_size.maxheight= (unsigned int) m_pcoInfo.wMaxVertResStdDESC; // ds->ccd.size.ymax,
	m_size.pixbits = (unsigned int) m_pcoInfo.wDynResDESC; // ds->ccd.size.bits
	m_size.pixbytes = (m_size.pixbits <= 8)?1:2; // nr de bytes por pixel  12 bits -> 2 bytes

	m_bufsize_max = m_size.maxwidth * m_size.maxheight * m_size.pixbytes;

	m_maxwidth_step= (unsigned int) m_pcoInfo.wRoiHorStepsDESC;   // ds->ccd.roi.xstep
	m_maxheight_step= (unsigned int) m_pcoInfo.wRoiVertStepsDESC; // ds->ccd.roi.ystep,

	m_roi.x[0] = m_roi.y[0] = 1;
	m_roi.x[1] = m_size.maxwidth;
	m_roi.y[1] = m_size.maxheight;
	m_roi.changed = Changed;


	sprintf_s(msg, MSG_SIZE, "* CCD Size = X[%d] x Y[%d] (%d bits)", m_size.maxwidth, m_size.maxheight, m_size.pixbits);
	DEB_TRACE() <<   msg;

	sprintf_s(msg, MSG_SIZE, "* ROI Steps = x:%d, y:%d", m_maxwidth_step,m_maxheight_step);
	DEB_TRACE() <<   msg;

	//dprintf("<%s> * CCD Size = %dx%d (%d bits)", fnId, ds->ccd.size.xmax, ds->ccd.size.ymax, ds->ccd.size.bits);
	//dprintf("<%s> * ROI Steps = x:%d, y:%d", fnId, ds->ccd.roi.xstep, ds->ccd.roi.ystep);



	// -- Print out current temperatures
	error = PcoCheckError(PCO_GetTemperature(m_handle, &m_temperature.ccd, &m_temperature.cam, &m_temperature.power));
	PCO_TRACE("PCO_GetTemperature") ;

	sprintf_s(msg, MSG_SIZE, "* temperature: CCD[%.1f]  CAM[%d]  PS[%d]", m_temperature.ccd, m_temperature.cam, m_temperature.power);
	DEB_TRACE() <<   msg;

	//dprintf("<%s> * CCD temperature = %.1f", fnId, ccdTemp/10.);
	//dprintf("<%s> * Camera temperature = %d", fnId, camTemp);
	//dprintf("<%s> * PowerSupply temperature = %d", fnId, powTemp);

	m_temperature.minCoolSet = m_pcoInfo.sMinCoolSetDESC;
	m_temperature.maxCoolSet = m_pcoInfo.sMaxCoolSetDESC;

	sprintf_s(msg, MSG_SIZE, "* cooling temperature: MIN [%d]  Max [%d]",  m_temperature.minCoolSet, m_temperature.maxCoolSet);
	DEB_TRACE() <<   msg;
	// dprintf("<%s> * res Cooling temperature = %d [%d - %d]", fnId, ds->ccd.temperature, ds->ccd.pcoInfo.sMinCoolSetDESC, ds->ccd.pcoInfo.sMaxCoolSetDESC);

	// -- Set/Get cooling temperature
	if (m_temperature.setpoint != -1) {
		if (m_temperature.setpoint < m_temperature.minCoolSet)	m_temperature.setpoint = m_temperature.minCoolSet;
		if (m_temperature.setpoint > m_temperature.maxCoolSet)	m_temperature.setpoint= m_temperature.maxCoolSet;
	} else {
		error = PcoCheckError(PCO_GetCoolingSetpointTemperature(m_handle, &m_temperature.setpoint));
		PCO_TRACE("PCO_GetCoolingSetpointTemperature") ;
	}
	//dprintf("<%s> * Cooling Setpoint = %d", fnId, ds->ccd.temperature);


	DEB_TRACE() <<  "end block 0";



// block #1 -- Get RAM size
	{
		int segmentPco, segmentArr;

		DWORD ramSize;
		WORD pageSize;
		
		error = PcoCheckError(PCO_GetCameraRamSize(m_handle, &ramSize, &pageSize));
		PCO_TRACE("PCO_GetCameraRamSize") ;



		//dprintf("<%s> * RAM number of pages = %ld", fnId, ramSize);
		//dprintf("<%s> * PAGE number of pixels = %d", fnId, pageSize);
		m_dwRamSize = ramSize;     // nr of pages of the ram
		m_wPageSize = pageSize;    // nr of pixels of the page


	sprintf_s(msg, MSG_SIZE, "* RAM number of pages [%ld]  PAGE number of pixels [%d]",  m_dwRamSize,m_wPageSize);
	DEB_TRACE() <<   msg;

		// ----------------- get initial seg Size - images & print

		error = PcoCheckError(PCO_GetCameraRamSegmentSize(m_handle, segSize));
		PCO_TRACE("PCO_GetCameraRamSegmentSize") ;


		for(segmentArr=0; segmentArr <4 ; segmentArr++) {
			segmentPco = segmentArr +1;
			m_dwSegmentSize[segmentArr] = segSize[segmentArr];

			sprintf_s(msg, MSG_SIZE, "* segment[%d] number of pages[%ld]", segmentPco,m_dwSegmentSize[segmentArr]);
			DEB_TRACE() <<   msg;
			//dprintf("<%s> * segment[%d] number of pages = %ld", fnId, segmentPco, segSize[segmentArr]);
		}

		for(segmentArr=0;  segmentArr<4 ; segmentArr++) {
			segmentPco = segmentArr +1;


		error = PcoCheckError(PCO_GetNumberOfImagesInSegment(m_handle, segmentPco, &_dwValidImageCnt, &_dwMaxImageCnt));
		PCO_TRACE("PCO_GetNumberOfImagesInSegment") ;


		//dprintf("<%s> * segment[%d] nr images [%ld]  max imag [%ld]", fnId, segmentPco, dwValidImageCnt, dwMaxImageCnt);
			dwValidImageCnt[segmentArr] = _dwValidImageCnt;
			dwMaxImageCnt[segmentArr] = _dwMaxImageCnt;
		} // for	

		for(segmentArr=0;  segmentArr<4 ; segmentArr++) {
			segmentPco = segmentArr +1;
			//dprintf("<%s> ** segment[%d] pages [%ld] nr images [%ld]  max imag [%ld]", fnId, segmentPco, dwSegmentSize[segmentArr], dwValidImageCnt[segmentArr], dwMaxImageCnt[segmentArr]);
		}	

		// set the first segment to the max ram size, the others = 0
		segmentArr=0;
		m_dwSegmentSize[segmentArr] = m_dwRamSize;
		for(segmentArr=1; segmentArr <4 ; segmentArr++) {m_dwSegmentSize[segmentArr] = 0; }
	

		// This function will result in all segments being cleared. All previously recorded images
		// will be lost!
		segmentArr=0;

		error = PcoCheckError(PCO_SetCameraRamSegmentSize(m_handle, &m_dwSegmentSize[segmentArr]));
		PCO_TRACE("PCO_SetCameraRamSegmentSize") ;




	}  // block #1 

	DEB_TRACE() <<  "end block 1";


		// block #2 -- Get RAM size 	-- 2nd - get  seg Size - images & print
	{
		int segmentPco, segmentArr;

		error = PcoCheckError(PCO_GetCameraRamSegmentSize(m_handle, segSize));
		PCO_TRACE("PCO_GetCameraRamSegmentSize") ;

		for(segmentArr=0; segmentArr <4 ; segmentArr++) {
			segmentPco = segmentArr +1;
			m_dwSegmentSize[segmentArr] = segSize[segmentArr];
			//dprintf("<%s> * segment[%d] number of pages = %ld", fnId, segmentPco, segSize[segmentArr]);
		}


		for(segmentArr=0;  segmentArr<4 ; segmentArr++) {
			segmentPco = segmentArr +1;

			error = PcoCheckError(PCO_GetNumberOfImagesInSegment(m_handle, segmentPco, &_dwValidImageCnt, &_dwMaxImageCnt));
			PCO_TRACE("PCO_GetNumberOfImagesInSegment") ;

			//dprintf("<%s> * segment[%d] nr images [%ld]  max imag [%ld]", fnId, segmentPco, dwValidImageCnt, dwMaxImageCnt);
			dwValidImageCnt[segmentArr] = _dwValidImageCnt;
			dwMaxImageCnt[segmentArr] = _dwMaxImageCnt;
		} // for	


		for(segmentArr=0;  segmentArr<4 ; segmentArr++) {
			segmentPco = segmentArr +1;
			//dprintf("<%s> ** segment[%d] pages [%ld] nr images [%ld]  max imag [%ld]", fnId, segmentPco, dwSegmentSize[segmentArr], dwValidImageCnt[segmentArr], dwMaxImageCnt[segmentArr]);
		}	
		

	} // block #2


	DEB_TRACE() <<  "end block 2";


	{
		int segmentPco, segmentArr;
		DWORD pages_per_image = m_size.maxwidth * m_size.maxheight / m_wPageSize;

		///------------------------------------------------------------------------TODO ?????
		for(segmentArr=0; segmentArr <4 ; segmentArr++) {
			segmentPco = segmentArr +1;
			if(dwMaxImageCnt[segmentArr] == 0){
				dwMaxImageCnt[segmentArr] = m_dwSegmentSize[segmentArr] / pages_per_image;
				if(dwMaxImageCnt[segmentArr] > 4) dwMaxImageCnt[segmentArr] -= 2;
			}

			//dprintf("<%s> * segment[%d] nr images [%ld]  max imag [%ld]", fnId, segmentPco, dwValidImageCnt, dwMaxImageCnt);

		}	

		for(segmentArr=0; segmentArr <4 ; segmentArr++) {
			segmentPco = segmentArr +1;
			//dprintf("<%s> ** segment[%d] pages [%ld] nr images [%ld]  max imag [%ld]", fnId, segmentPco, ds->ccd.dwSegmentSize[segmentArr],ss->ccd.dwValidImageCnt[segmentArr], ds->ccd.dwMaxImageCnt[segmentArr]);
		}	

	} // block





	// -- Get Active RAM segment 

		error = PcoCheckError(PCO_GetActiveRamSegment(m_handle, &m_activeRamSegment));
		PCO_TRACE("PCO_GetActiveRamSegment") ;


	//dprintf("<%s> * Active RAM segment = %d", fnId, ds->ccd.activeRamSegment);

	//getNrImagesSegment(ds, ds->ccd.activeRamSegment, &dwValidImageCnt, &dwMaxImageCnt, error);

		error = PcoCheckError(PCO_GetNumberOfImagesInSegment(m_handle, m_activeRamSegment, &_dwValidImageCnt, &_dwMaxImageCnt));
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
  // PvAttrStringGet(m_handle, "CameraName", m_camera_name, 128, &psize);
  // PvAttrUint32Get(m_handle, "UniqueId", &m_uid);
  // PvAttrUint32Get(m_handle, "FirmwareVerMajor", &m_ufirmware_maj);
  // PvAttrUint32Get(m_handle, "FirmwareVerMinor", &m_ufirmware_min);
  // PvAttrEnumGet(m_handle, "SensorType", m_sensor_type, 
//		sizeof(m_sensor_type), &psize);

  DEB_TRACE() << DEB_VAR1(m_camera_name);
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
  DEB_RETURN() << DEB_VAR1(m_camera_name);

  name = m_camera_name;
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

//=====================================================================
//static long StartAcq(Ccd ds, DevVoid *argin, DevVoid *argout, long *error){
    char *fnId = "StartAcq";
    unsigned long size;
    WORD state;
    HANDLE hEvent= NULL;
    int bufIdx;
    DWORD exposure, delay;
    WORD exposure_base, delay_base;
    float factor;
    int error;

    //*error= DevErrCcdController;


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
    WORD wRoiX0; // Roi upper left x
    WORD wRoiY0; // Roi upper left y
    WORD wRoiX1; // Roi lower right x
    WORD wRoiY1;// Roi lower right y

    if(m_roi.changed == Valid) m_roi.changed = Changed;    //+++++++++ TEST / FORCE WRITE ROI
    if (m_roi.changed == Changed) {
        wRoiX0 = (WORD)m_roi.x[0];
        wRoiX1 = (WORD)m_roi.x[1];
        wRoiY0 = (WORD)m_roi.y[0];
        wRoiY1 = (WORD)m_roi.y[1];

		DEB_TRACE() << DEB_VAR4(wRoiX0, wRoiY0, wRoiX1, wRoiY1);

        error = PcoCheckError(PCO_SetROI(m_handle, wRoiX0, wRoiY0, wRoiX1, wRoiY1));
        PCO_TRACE("PCO_SetROI") ;

        m_roi.changed= Valid;
    }

	error = PcoCheckError(PCO_GetROI(m_handle, &wRoiX0, &wRoiY0, &wRoiX1, &wRoiY1));
    PCO_TRACE("PCO_GetROI") ;
	DEB_TRACE() << DEB_VAR4(wRoiX0, wRoiY0, wRoiX1, wRoiY1);

    //------------------------------------------------- triggering mode 
	
	WORD trigmode = m_sync->getPcoTrigMode();
    error = PcoCheckError(PCO_SetTriggerMode(m_handle, trigmode));
    PCO_TRACE("PCO_SetTriggerMode") ;
	DEB_TRACE() << DEB_VAR1(trigmode);

    // -- acquire mode : ignore or not ext. signal

	WORD acqmode = m_sync->getPcoAcqMode();
	error = PcoCheckError(PCO_SetAcquireMode(m_handle, acqmode));
    PCO_TRACE("PCO_SetAcquireMode") ;
	DEB_TRACE() << DEB_VAR1(acqmode);

    // -- storage mode (recorder + sequence)
    m_storage_mode = 0;
    m_recorder_submode = 0;

    error = PcoCheckError(PCO_SetStorageMode(m_handle, m_storage_mode));
    PCO_TRACE("PCO_SetStorageMode") ;

    error = PcoCheckError(PCO_SetRecorderSubmode(m_handle, m_recorder_submode));
    PCO_TRACE("PCO_SetRecorderSubmode") ;

    {
        double _exposure, _delay;
        m_sync->getExpTime(_exposure);
        m_sync->getLatTime(_delay);

        //----------------------------------- set exposure time & delay time
        for (exposure_base = 0; exposure_base < 3; exposure_base++) {
            factor = pow((float)10, (int) (exposure_base * 3 - 9));			// factor 10E-9, 10E-6, 10E-3
            if ((_exposure / factor) <= (pow(2., 32) - 1)) {		// multiply by 10E9, 10E6, 10E3
                exposure = (DWORD) (_exposure / factor);			// exposure max precision in 32 bits, exposure base 0(ns)  1(us)  2(ms)
                break;
            }
        }
        //====================================== TODO set/get the valuo of ccd.delay now is 0 
        for (delay_base = 0; delay_base < 3; delay_base++) {
            factor = pow((float) 10, (int) (delay_base * 3 - 9));
            if ((_delay / factor) <= (pow(2., 32) - 1)) {
                delay = (DWORD) (_delay / factor);
                break;
            }
        }
    } // block

    DEB_TRACE() << DEB_VAR2(delay, timebaseUnits[delay_base]);
	DEB_TRACE() << DEB_VAR2(exposure, timebaseUnits[exposure_base]);

	error = PcoCheckError(PCO_SetDelayExposureTime(m_handle, delay, exposure, delay_base, exposure_base));
    PCO_TRACE("PCO_SetDelayExposureTime") ;

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

        m_cocRunTime = runTime = ((double) dwTime_ns * 1.0E-9) + (double) dwTime_s;
        m_frameRate = (dwTime_ns | dwTime_s) ? 1.0 / runTime : 0.0;

        DEB_TRACE() << DEB_VAR2(m_frameRate, m_cocRunTime);
    } // block


        //====================================== get exp time

    {
        DWORD exposure, delay;
        WORD exposure_base, delay_base;

        error = PcoCheckError(PCO_GetDelayExposureTime(m_handle, &delay, &exposure, &delay_base, &exposure_base));
        PCO_TRACE("PCO_GetDelayExposureTime") ;

	    DEB_TRACE() << DEB_VAR2(delay, timebaseUnits[delay_base]);
		DEB_TRACE() << DEB_VAR2(exposure, timebaseUnits[exposure_base]);
    }


    //------------------------------------------------ get size
	//   int err = PCO_GetSizes(hCamera, &wXResActual, &wYResActual, &wXResMax, &wYResMax);

    WORD xmax, ymax;
	
	error = PcoCheckError(PCO_GetSizes(m_handle, &m_size.armwidth, &m_size.armheight, &xmax, &ymax));
    PCO_TRACE("PCO_GetSizes") ;

    //m_size.armwidth= xactualsize;
    //m_size.armheight= yactualsize;
    size= m_size.armwidth * m_size.armheight * m_size.pixbytes;

    m_bufsize= m_bufsize_max;

	//------------------------------------------------- allocate buffer if not yet done, or size changed
	if ((m_bufferNrM[0]==-1) || (size != m_imgsizeBytes)) {
        m_imgsizeBytes= size;
        m_imgsizePixels= m_size.armwidth * m_size.armheight;
        m_imgsizePages= m_imgsizePixels / m_wPageSize;
        if(m_imgsizePixels % m_wPageSize) m_imgsizePages++;
        m_imgsizePages++;
        m_imgsizeBuffer = m_imgsizePages * m_wPageSize * m_size.pixbytes;

        //ds->ccd.bufsize= size;
        m_bufsize= m_bufsize_max;



        m_frames_per_buffer = (int) m_bufsize_max / m_imgsizeBuffer;
        //if(ds->ccd.frames_per_buffer > 2) ds->ccd.frames_per_buffer /= 2;
        m_frames_per_buffer = 1;

        //dprintf2("<%s> allocating buffer ... imgsizeBytes[%ld] imgsizeBuffer[%ld] bufMax[%ld] framesPerBuf[%ld]", fnId, ds->ccd.imgsizeBytes, ds->ccd.imgsizeBuffer, ds->ccd.bufsize_max, ds->ccd.frames_per_buffer);

        //+++++++		//ds->ccd.bufsize=2016 * 2016 * ds->ccd.size.depth;

        //dprintf2("<%s> PCO_FreeBuffers ...", fnId);
        for(bufIdx=0; bufIdx < 8; bufIdx++) {
            if(m_bufferNrM[bufIdx] != -1) {
                //dprintf2("<%s> PCO_FreeBuffers ... bufnrM[%d] = %d", fnId, bufIdx,  m_bufferNrM[bufIdx]);

                error = PcoCheckError(PCO_FreeBuffer(m_handle, m_bufferNrM[bufIdx]));
                PCO_TRACE("PCO_FreeBuffer") ;
                m_bufferNrM[bufIdx] = -1;
                m_bufferM[bufIdx] = NULL;
            }
        }

        //-------------- allocate 2 buffers (0,1) and received the handle, mem ptr, events
        for(bufIdx = 0; bufIdx <2 ; bufIdx ++) {
            //dprintf3("... PCO_AllocateBuffer[idx = %d]", bufIdx);

            error = PcoCheckError(PCO_AllocateBuffer(m_handle, &(m_bufferNrM[bufIdx]), (DWORD)m_bufsize, &(m_bufferM[bufIdx]), &(m_bufferM_events[bufIdx])));
            PCO_TRACE("PCO_AllocateBuffer") ;

            //dprintf2("<%s> Buffer #%d allocated (%d) (size=%d)", fnId, m_bufferNrM[bufIdx], bufIdx, ds->ccd.bufsize);
        }
	} // if((m_bufferNrM[0]==-1) 


    //------------------------------------------------- set image size for CamLink and GigE
    switch (m_interface_type) {
        case INTERFACE_CAMERALINK:

            error = PcoCheckError(PCO_GetTransferParameter(m_handle, &m_clTransferParam, sizeof(m_clTransferParam)));
            PCO_TRACE("PCO_GetTransferParameter") ;

            //dprintf2("<%s> PCO_GetTransferParameter  baudrate[%d] ClockFrequency[%d] CCline[%d] DataFormat[%ld] Transmit[%d]", fnId, 
            //ds->ccd.clTransferParam.baudrate, ds->ccd.clTransferParam.ClockFrequency, ds->ccd.clTransferParam.CCline,
            //ds->ccd.clTransferParam.DataFormat, ds->ccd.clTransferParam.Transmit);

            if((m_clTransferParam.baudrate != 115200) || (m_clTransferParam.DataFormat != 2)) {
                //dprintf("<%s> ALERT - PCO_GetTransferParameter  baudrate[%d] DataFormat[%ld]", fnId, 
                //m_clTransferParam.baudrate, m_clTransferParam.DataFormat);
                m_clTransferParam.baudrate=115200;
                m_clTransferParam.DataFormat=2;

                error = PcoCheckError(PCO_SetTransferParameter(m_handle, &m_clTransferParam, sizeof(m_clTransferParam)));
                PCO_TRACE("PCO_SetTransferParameter") ;
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
            PCO_TRACE("PCO_CamLinkSetImageParameters") ;

            break;

        default: break;
    } // case

    //------------------------------------------------- checking nr of frames
    {
        int segmentPco, segmentArr;
        unsigned long frames, framesMax;
        int iFrames;

        segmentPco = m_activeRamSegment;
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
    //dprintf3("... PCO_SetRecordingState");

	return;

    error = PcoCheckError(PCO_SetRecordingState(m_handle, 0x0001));
    PCO_TRACE("PCO_SetRecordingState") ;

    m_frame.done= 0;
    //dprintf("<%s> acquisition started.", fnId);
    //return (DS_OK);
	
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

unsigned long Camera::_getFramesMax(int segmentPco){
		char *fnId = "_getFramesMax";

		int segmentArr = segmentPco-1;
		unsigned long framesMax;
		unsigned long xroisize,yroisize;
		unsigned long long pixPerFrame, pagesPerFrame;

		xroisize = m_roi.x[1] - m_roi.x[0] + 1;
		yroisize = m_roi.y[1] - m_roi.y[0] + 1;

		pixPerFrame = (unsigned long long)xroisize * (unsigned long long)yroisize;
		pagesPerFrame = (pixPerFrame / m_wPageSize) + 1;
		if(pixPerFrame % m_wPageSize) pagesPerFrame++;

		framesMax = m_dwMaxFramesInSegment[segmentArr] = (unsigned long)(((long long) m_dwSegmentSize[segmentArr] ) / pagesPerFrame);

		//dprintf2("<%s> framesMax[%ld] segment[%d] ramPages[%ld] pageSize[%ld] roi[%ld / %ld]", fnId, framesMax, segmentPco, 
			//ds->ccd.dwSegmentSize[segmentArr], ds->ccd.wPageSize, xroisize, yroisize);

		

		return framesMax;
	}



char *Camera::getInfo(char *output, int lg){
		char *ptr, *ptrMax;
		int segmentPco = m_activeRamSegment;
		int segmentArr = segmentPco -1;

		ptr = output; *ptr = 0;
		ptrMax = ptr + lg;

		int width = +20;

		ptr += sprintf_s(ptr, ptrMax - ptr,"**** PCO Info\n");
		ptr += sprintf_s(ptr, ptrMax - ptr,"*%*s = %s\n", width, "timestamp", getTimestamp(0));

		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = %g fps\n", width, "frameRate", m_frameRate);
		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = X(%d,%d) Y(%d,%d) size(%d,%d)\n", width, "roi", 
				m_roi.x[0], m_roi.x[1],
				m_roi.y[0], m_roi.y[1],
				m_roi.x[1] - m_roi.x[0] + 1, m_roi.y[1] - m_roi.y[0] + 1);

		double _exposure;
		m_sync->getExpTime(_exposure);
		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = %g s\n", width, "expTime", _exposure);
		
		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = %d\n", width, "framesMax", m_dwMaxFramesInSegment[segmentArr]);

		int iFrames;
		m_sync->getNbFrames(iFrames);
		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = %ld\n", width,"nrFrames", iFrames);

		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = %d\n", width, "activeSegment", segmentPco);

		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = %ld pages\n", width, "segmentSize", m_dwSegmentSize[segmentArr]);

		ptr += sprintf_s(ptr, ptrMax - ptr, "*%*s = %d pix\n", width, "pixPerPage", m_wPageSize);

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
		ptr += sprintf_s(ptr, ptrMax - ptr,"****\n");
		return output;
}
