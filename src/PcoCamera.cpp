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

#define BYPASS

#include <cstdlib>


#ifndef BYPASS
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif


#include "Exceptions.h"

#include "PcoCamera.h"
#include "PcoSyncCtrlObj.h"
#include "PcoVideoCtrlObj.h"

using namespace lima;
using namespace lima::Pco;



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

Camera::Camera(const char *ip_addr) :
  m_cam_connected(false),
  m_sync(NULL),
  m_video(NULL)
{
  DEB_CONSTRUCTOR();

  int error;
  char *ptr;

  // Init Frames
  //m_frame[0].ImageBuffer = NULL;
  //m_frame[0].Context[0] = this;
  //m_frame[1].ImageBuffer = NULL;
  //m_frame[1].Context[0] = this;
  
  //m_camera_name[0] = m_sensor_type[0] = '\0';




	// tPvErr error = PvInitialize();
	// --- Open Camera
	error = PcoCheckError(PCO_OpenCamera(&hPco, 0));
	if(error)
		throw LIMA_HW_EXC(Error, "PCO_OpenCamera / could not open the camera");


	// --- Get camera type
	strCamType.wSize= sizeof(strCamType);
	error = PcoCheckError(PCO_GetCameraType(hPco, &strCamType));
	if(error)
		throw LIMA_HW_EXC(Error, "PCO_GetCameraType");



	if((ptr = xlatI2A(strCamType.wCamType, modelType)) != NULL) {
		strcpy_s(model, MODEL_SIZE, ptr);	error= 0;
	} else {
		sprintf_s(model, MODEL_SIZE, "UNKNOWN [0x%04x]", strCamType.wCamType); error= 1;
	}

	if(error)
		throw LIMA_HW_EXC(Error, "Unknow model");
	
	if((ptr = xlatI2A((interface_type = strCamType.wInterfaceType), interfaceType)) != NULL) {
		strcpy_s(iface, MODEL_SIZE, ptr);	error= 0;
	} else {
		sprintf_s(iface, MODEL_SIZE, "UNKNOWN [0x%04x]", strCamType.wInterfaceType); error= 1;
	}

	if(error)
		throw LIMA_HW_EXC(Error, "Unknow interface");

	

	// -- Reset to default settings
	error = PcoCheckError(PCO_ResetSettingsToDefault(hPco));
	if(error)
		throw LIMA_HW_EXC(Error, "PCO_ResetSettingsToDefault");

	error = PcoCheckError(PCO_ResetSettingsToDefault(hPco));
	if(error)
		throw LIMA_HW_EXC(Error, "PCO_ResetSettingsToDefault");

	// -- Get camera description
	pcoInfo.wSize= sizeof(pcoInfo);

	error = PcoCheckError(PCO_GetCameraDescription(hPco, &pcoInfo));
	if(error)
		throw LIMA_HW_EXC(Error, "PCO_GetCameraDescription");


	  // PvAttrUint32Get(m_handle, "SensorWidth", &m_maxwidth);
  // PvAttrUint32Get(m_handle, "SensorHeight", &m_maxheight);

  //DEB_TRACE() << DEB_VAR2(m_maxwidth,m_maxheight);


		// -- Initialise adc, size, bin, roi
	m_nradc= 1;
	m_maxadc = pcoInfo.wNumADCsDESC;

	m_maxwidth = (unsigned int) pcoInfo.wMaxHorzResStdDESC;
	m_maxheight= (unsigned int) pcoInfo.wMaxVertResStdDESC;
	m_pixbits = (unsigned int) pcoInfo.wDynResDESC;
	m_pixbytes = (m_pixbits <= 8)?1:2; // nr de bytes por pixel  12 bits -> 2 bytes

	m_max_buffsize = m_maxwidth * m_maxheight * m_pixbytes;

	m_maxwidth_step= (unsigned int) pcoInfo.wRoiHorStepsDESC;
	m_maxheight_step= (unsigned int) pcoInfo.wRoiVertStepsDESC;

	//dprintf("<%s> * CCD Size = %dx%d (%d bits)", fnId, ds->ccd.size.xmax, ds->ccd.size.ymax, ds->ccd.size.bits);
	//dprintf("<%s> * ROI Steps = x:%d, y:%d", fnId, ds->ccd.roi.xstep, ds->ccd.roi.ystep);



	// -- Print out current temperatures
	error = PcoCheckError(PCO_GetTemperature(hPco, &m_temperature.ccd, &m_temperature.cam, &m_temperature.power));
	if(error)
		throw LIMA_HW_EXC(Error, "PCO_GetTemperature");


	//dprintf("<%s> * CCD temperature = %.1f", fnId, ccdTemp/10.);
	//dprintf("<%s> * Camera temperature = %d", fnId, camTemp);
	//dprintf("<%s> * PowerSupply temperature = %d", fnId, powTemp);

	m_temperature.minCoolSet = pcoInfo.sMinCoolSetDESC;
	m_temperature.maxCoolSet = pcoInfo.sMaxCoolSetDESC;

	// dprintf("<%s> * res Cooling temperature = %d [%d - %d]", fnId, ds->ccd.temperature, ds->ccd.pcoInfo.sMinCoolSetDESC, ds->ccd.pcoInfo.sMaxCoolSetDESC);

	// -- Set/Get cooling temperature
	if (m_temperature.setpoint != -1) {
		if (m_temperature.setpoint < m_temperature.minCoolSet)	m_temperature.setpoint = m_temperature.minCoolSet;
		if (m_temperature.setpoint > m_temperature.maxCoolSet)	m_temperature.setpoint= m_temperature.maxCoolSet;
	} else {
		error = PcoCheckError(PCO_GetCoolingSetpointTemperature(hPco, &m_temperature.setpoint));
		if(error)
			throw LIMA_HW_EXC(Error, "PCO_GetCoolingSetpointTemperature");
	}
	//dprintf("<%s> * Cooling Setpoint = %d", fnId, ds->ccd.temperature);






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

 // DEB_TRACE() << DEB_VAR3(m_camera_name,m_sensor_type,m_uid);


  // error = PvAttrUint32Set(m_handle,"Width",m_maxwidth);
  if(error)
    throw LIMA_HW_EXC(Error,"Can't set image width");
  
  // error = PvAttrUint32Set(m_handle,"Height",m_maxheight);
  if(error)
    throw LIMA_HW_EXC(Error,"Can't set image height");
  
  VideoMode localVideoMode;
  if(isMonochrome())
    {
      //error = PvAttrEnumSet(m_handle, "PixelFormat", "Mono16");
      localVideoMode = Y16;
    }
  else
    {
      // error = PvAttrEnumSet(m_handle, "PixelFormat", "Bayer16");
      localVideoMode = BAYER_RG16;
    }

  if(error)
    throw LIMA_HW_EXC(Error,"Can't set image format");
  
  m_video_mode = localVideoMode;

  // error = PvAttrEnumSet(m_handle, "AcquisitionMode", "Continuous");
  if(error)
    throw LIMA_HW_EXC(Error,"Can't set acquisition mode to continuous");
}


  long ControllerInitialise(Ccd ds, long *error)
{
	char *fnId = "ControllerInitialise";

	short failed;
	char model[32], iface[32];
	short ccdTemp, camTemp, powTemp;
	PCO_CameraType	strCamType;
	DWORD dwValidImageCnt, dwMaxImageCnt;
	DWORD segSize[4];

	//------------------------ DONE




	// block #1 -- Get RAM size
	{
		int segmentPco, segmentArr;

		DWORD ramSize;
		WORD pageSize;
		
		if (PcoCheckError(PCO_GetCameraRamSize(ds->ccd.hPco, &ramSize, &pageSize))) {
			*error= DevErr_DeviceHardwareError;
			dprintf("<%s> Cannot get RAM size", fnId);
			return (DS_NOTOK);
		}
		dprintf("<%s> * RAM number of pages = %ld", fnId, ramSize);
		dprintf("<%s> * PAGE number of pixels = %d", fnId, pageSize);
		ds->ccd.dwRamSize = ramSize;     // nr of pages of the ram
		ds->ccd.wPageSize = pageSize;    // nr of pixels of the page

		// ----------------- get initial seg Size - images & print
		if (PcoCheckError(PCO_GetCameraRamSegmentSize(ds->ccd.hPco, segSize))) {
			*error= DevErr_DeviceHardwareError;
			dprintf("<%s> Cannot get seg size", fnId);
			return (DS_NOTOK);
		}
		
		for(segmentArr=0; segmentArr <4 ; segmentArr++) {
			segmentPco = segmentArr +1;
			ds->ccd.dwSegmentSize[segmentArr] = segSize[segmentArr];
			//dprintf("<%s> * segment[%d] number of pages = %ld", fnId, segmentPco, segSize[segmentArr]);
		}

		for(segmentArr=0;  segmentArr<4 ; segmentArr++) {
			segmentPco = segmentArr +1;

			if (PcoCheckError(PCO_GetNumberOfImagesInSegment(ds->ccd.hPco, segmentPco, &dwValidImageCnt, &dwMaxImageCnt))) {
				*error= DevErr_DeviceHardwareError;
				dprintf("<%s> Cannot get nr of images in the segment %d", fnId, segmentPco);
				return (DS_NOTOK);
			}
			//dprintf("<%s> * segment[%d] nr images [%ld]  max imag [%ld]", fnId, segmentPco, dwValidImageCnt, dwMaxImageCnt);
			ds->ccd.dwValidImageCnt[segmentArr] = dwValidImageCnt;
			ds->ccd.dwMaxImageCnt[segmentArr] = dwMaxImageCnt;
		}	

		for(segmentArr=0;  segmentArr<4 ; segmentArr++) {
			segmentPco = segmentArr +1;
			dprintf("<%s> ** segment[%d] pages [%ld] nr images [%ld]  max imag [%ld]", fnId, segmentPco, 
				ds->ccd.dwSegmentSize[segmentArr], ds->ccd.dwValidImageCnt[segmentArr], ds->ccd.dwMaxImageCnt[segmentArr]);
		}	

		// set the first segment to the max ram size, the others = 0
		segmentArr=0;
		ds->ccd.dwSegmentSize[segmentArr] = ds->ccd.dwRamSize;
		for(segmentArr=1; segmentArr <4 ; segmentArr++) {ds->ccd.dwSegmentSize[segmentArr] = 0; }
	

		// This function will result in all segments being cleared. All previously recorded images
		// will be lost!
		segmentArr=0;
		if (PcoCheckError(PCO_SetCameraRamSegmentSize(ds->ccd.hPco, &ds->ccd.dwSegmentSize[segmentArr]))) {
			*error= DevErr_DeviceHardwareError;
			dprintf("<%s> Cannot set seg size", fnId);
			return (DS_NOTOK);
		}

	}  // block #1 




		// block #2 -- Get RAM size 	-- 2nd - get  seg Size - images & print
	{
		int segmentPco, segmentArr;

		if (PcoCheckError(PCO_GetCameraRamSegmentSize(ds->ccd.hPco, segSize))) {
			*error= DevErr_DeviceHardwareError;
			dprintf("<%s> Cannot get seg size", fnId);
			return (DS_NOTOK);
		}
	
		for(segmentArr=0; segmentArr <4 ; segmentArr++) {
			segmentPco = segmentArr +1;
			ds->ccd.dwSegmentSize[segmentArr] = segSize[segmentArr];
			//dprintf("<%s> * segment[%d] number of pages = %ld", fnId, segmentPco, segSize[segmentArr]);
		}


		for(segmentArr=0;  segmentArr<4 ; segmentArr++) {
			segmentPco = segmentArr +1;

			if (PcoCheckError(PCO_GetNumberOfImagesInSegment(ds->ccd.hPco, segmentPco, &dwValidImageCnt, &dwMaxImageCnt))) {
				*error= DevErr_DeviceHardwareError;
				dprintf("<%s> Cannot get nr of images in the segment %d", fnId, segmentPco);
				return (DS_NOTOK);
			}
			//dprintf("<%s> * segment[%d] nr images [%ld]  max imag [%ld]", fnId, segmentPco, dwValidImageCnt, dwMaxImageCnt);
			ds->ccd.dwValidImageCnt[segmentArr] = dwValidImageCnt;
			ds->ccd.dwMaxImageCnt[segmentArr] = dwMaxImageCnt;
		}	
		

		for(segmentArr=0;  segmentArr<4 ; segmentArr++) {
			segmentPco = segmentArr +1;
			dprintf("<%s> ** segment[%d] pages [%ld] nr images [%ld]  max imag [%ld]", fnId, segmentPco, 
				ds->ccd.dwSegmentSize[segmentArr], ds->ccd.dwValidImageCnt[segmentArr], ds->ccd.dwMaxImageCnt[segmentArr]);
		}	

	} // block #2


	{
		int segmentPco, segmentArr;
		DWORD pages_per_image = ds->ccd.size.xmax * ds->ccd.size.ymax / ds->ccd.wPageSize;

		///------------------------------------------------------------------------TODO ?????
		for(segmentArr=0; segmentArr <4 ; segmentArr++) {
			segmentPco = segmentArr +1;
			if(ds->ccd.dwMaxImageCnt[segmentArr] == 0){
				ds->ccd.dwMaxImageCnt[segmentArr] = ds->ccd.dwSegmentSize[segmentArr] / pages_per_image;
				if(ds->ccd.dwMaxImageCnt[segmentArr] > 4) ds->ccd.dwMaxImageCnt[segmentArr] -= 2;
			}

			//dprintf("<%s> * segment[%d] nr images [%ld]  max imag [%ld]", fnId, segmentPco, dwValidImageCnt, dwMaxImageCnt);

		}	

		for(segmentArr=0; segmentArr <4 ; segmentArr++) {
			segmentPco = segmentArr +1;
			dprintf("<%s> ** segment[%d] pages [%ld] nr images [%ld]  max imag [%ld]", fnId, segmentPco, ds->ccd.dwSegmentSize[segmentArr], 
				ds->ccd.dwValidImageCnt[segmentArr], ds->ccd.dwMaxImageCnt[segmentArr]);
		}	

	} // block




	// -- Get Active RAM segment 
	if (PcoCheckError(PCO_GetActiveRamSegment(ds->ccd.hPco, &ds->ccd.activeRamSegment))) {
		*error= DevErr_DeviceHardwareError;
		dprintf("<%s> Cannot get active RAM segment", fnId);
		return (DS_NOTOK);
	}
	dprintf("<%s> * Active RAM segment = %d", fnId, ds->ccd.activeRamSegment);

	getNrImagesSegment(ds, ds->ccd.activeRamSegment, &dwValidImageCnt, &dwMaxImageCnt, error);



	dprintf("<%s> DONE", fnId);
	return (DS_OK);
}

#endif

#ifndef BYPASS
Camera::~Camera()
{
  DEB_DESTRUCTOR();

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
}

/** @brief test if the camera is monochrome
 */
#endif

#ifndef BYPASS
bool Camera::isMonochrome() const
{
  DEB_MEMBER_FUNCT();

  return !strcmp(m_sensor_type,"Mono");
}

#endif

#ifndef BYPASS
VideoMode Camera::getVideoMode() const
{
  DEB_MEMBER_FUNCT();
  DEB_RETURN() << DEB_VAR1(m_video_mode);

  return m_video_mode;
}

#endif

#ifndef BYPASS
void Camera::getCameraName(std::string& name)
{
  DEB_MEMBER_FUNCT();
  DEB_RETURN() << DEB_VAR1(m_camera_name);

  name = m_camera_name;
}
#endif

#ifndef BYPASS
void Camera::setVideoMode(VideoMode aMode)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(aMode);

  ImageType anImageType;
  tPvErr error;
  switch(aMode)
    {
    case Y8:
      error = PvAttrEnumSet(m_handle, "PixelFormat", "Mono8");
      anImageType = Bpp8;
      break;
    case Y16:
      error = PvAttrEnumSet(m_handle, "PixelFormat", "Mono16");
      anImageType = Bpp16;
      break;
    case BAYER_RG8:
      error = PvAttrEnumSet(m_handle, "PixelFormat", "Bayer8");
      anImageType = Bpp8;
      break;
    case BAYER_RG16:
      error = PvAttrEnumSet(m_handle, "PixelFormat", "Bayer16");
      anImageType = Bpp16;
      break;
    default:
      throw LIMA_HW_EXC(InvalidValue,"This video mode is not managed!");
    }
  
  if(error)
    throw LIMA_HW_EXC(Error,"Can't change video mode");
  
  m_video_mode = aMode;
  maxImageSizeChanged(Size(m_maxwidth,m_maxheight),anImageType);
}

#endif

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

#ifndef BYPASS
void Camera::startAcq()
{
  DEB_MEMBER_FUNCT();

  m_continue_acq = true;
  m_acq_frame_nb = 0;
  tPvErr error = PvCaptureQueueFrame(m_handle,&m_frame[0],_newFrameCBK);

  int requested_nb_frames;
  m_sync->getNbFrames(requested_nb_frames);
  bool isLive;
  m_video->getLive(isLive);

  if(!requested_nb_frames || requested_nb_frames > 1 || isLive)
    error = PvCaptureQueueFrame(m_handle,&m_frame[1],_newFrameCBK);
}

#endif

#ifndef BYPASS
void Camera::reset()
{
  DEB_MEMBER_FUNCT();
  //@todo maybe something to do!
}

#endif

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
	if (err != 0) {
		PCO_GetErrorText(err, pcoErrorMsg, ERR_SIZE-14);
		//sprintf(lastErrorMsg, "<PCO ERROR> %s", pcoErrorMsg);
		//dprintf(lastErrorMsg);
		return (1);
	}
	return (0);
}