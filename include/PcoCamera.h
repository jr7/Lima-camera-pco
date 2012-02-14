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
#ifndef PCOCAMERA_H
#define PCOCAMERA_H
#include "Pco.h"
#include "Pco_errt.h"
#include "Debug.h"
#include "Constants.h"
#include "HwMaxImageSizeCallback.h"
//#include "PcoBufferCtrlObj.h"

#define PCO_CL_PIXELCLOCK_95MHZ 95000000
#define PCO_CL_PIXELCLOCK_286MHZ 286000000
#define PCO_CL_BAUDRATE_115K2	115200

#define PCO_BUFFER_NREVENTS 4
struct stcXlatCode2Str {
		int code;
		char *str;
};


#define PCO_MAXSEGMENTS 4



struct stcFrame {
	BOOL	changed;
	unsigned int nb;
	unsigned int done;

	unsigned int ram_current;

	unsigned long next2read;


};


struct stcTemp {
	short wCcd, wCam, wPower;
	short wMinCoolSet, wMaxCoolSet;
	short wSetpoint;
};


struct stcPcoData {
	PCO_Description	pcoInfo;	/* camera description structure */
	PCO_CameraType	stcCamType;
	char model[MODEL_TYPE_SIZE+1], iface[INTERFACE_TYPE_SIZE+1];
	//int	interface_type;

	PCO_SC2_CL_TRANSFER_PARAM clTransferParam;
	int pcoError;
        char pcoErrorMsg[ERR_SIZE+1];

	double	cocRunTime;		/* cam operation code - delay & exposure time & readout in s*/
	double	frameRate;
    WORD    activeRamSegment;				/* active ram segment */

      	//WORD		m_acq_mode;
      	WORD		storage_mode;
      	WORD		recorder_submode;
		unsigned long	frames_per_buffer; 
        DWORD   dwRamSize;
        WORD    wPixPerPage;
        DWORD   dwMaxFramesInSegment[4];
        DWORD   dwSegmentSize[4];

        DWORD   dwValidImageCnt[4];
        DWORD   dwMaxImageCnt[4];

		char		camera_name[CAMERA_NAME_SIZE];
        char		sensor_type[64];

        
        unsigned int    nr_adc, max_adc;
        unsigned int    maxwidth_step, maxheight_step;

        struct stcTemp temperature;

		unsigned int bytesPerPix;     //unsigned int depth;
		unsigned int bitsPerPix;     //unsigned int bits;
		unsigned int maxWidth;		// unsigned int xmax;		/* Max size */
		unsigned int maxHeight;        //unsigned int ymax;

		WORD wMetaDataMode, wMetaDataSize, wMetaDataVersion;
		long msAcqRec, msAcqXfer, msAcqTout, msAcqTnow;
		DWORD dwPixelRate;

		WORD wXResActual, wYResActual, wXResMax, wYResMax;
		WORD wLUT_Identifier, wLUT_Parameter;

		DWORD dwAllocatedBufferSize;
		int iAllocatedBufferNumber;
		bool bAllocatedBufferDone;

};

enum enumChange {
	Invalid, Valid, Changed,
};

struct stcRoi {
	enumChange changed;	/* have values been changed ? */
	unsigned int x[2];	/* ROI min/max in x dir.(note starts at 1)*/
	unsigned int y[2];	/* ROI min/max in y dir.(note starts at 1)*/
	unsigned int xstep;	/* ROI granularity in x dir */
	unsigned int ystep;	/* ROI granularity in x dir */
};


struct stcBinning {
	enumChange	changed;		/* have values been changed ? */
	unsigned int x;			/* amount to bin/group x data.                 */
	unsigned int y;			/* amount to bin/group y data.                 */
};


namespace lima
{
  namespace Pco
  {
    class SyncCtrlObj;
    class VideoCtrlObj;
    class  DLL_EXPORT  Camera : public HwMaxImageSizeCallbackGen
    {
      friend class Interface;
      friend class SyncCtrlObj;
      DEB_CLASS_NAMESPC(DebModCamera,"Camera","Pco");
      public:
        Camera(const char *camPar);
        ~Camera();

        void 	startAcq();
        void	reset();

		HANDLE& getHandle() {return m_handle;}

        void getMaxWidthHeight(DWORD& width,DWORD& height){width = m_pcoData->maxWidth, height = m_pcoData->maxHeight;}
        void getArmWidthHeight(WORD& width,WORD& height){width = m_pcoData->wXResActual, height = m_pcoData->wYResActual;}
        void getBytesPerPixel(unsigned int& pixbytes){pixbytes = m_pcoData->bytesPerPix;}
		void getBitsPerPixel(WORD& pixbits){pixbits = (WORD) m_pcoData->bitsPerPix;}

		int getNbAcquiredFrames() const {return m_acq_frame_nb;}

        void getCameraName(std::string& name);

        char *getInfo(char *cmd, char *output, int lg);
        char *getInfo(char *cmd);

        unsigned long pcoGetFramesMax(int segmentPco);

		unsigned long	pcoGetFramesPerBuffer() { return m_pcoData->frames_per_buffer; }
		double pcoGetCocRunTime() { return m_pcoData->cocRunTime; }
		double pcoGetFrameRate() { return m_pcoData->frameRate; }

		WORD pcoGetActiveRamSegment() {return m_pcoData->activeRamSegment;}

		SyncCtrlObj*	_getSyncCtrlObj() { return m_sync;}
		struct stcPcoData * _getPcoData() {return  m_pcoData; }
		char* _PcoCheckError(int err, int&error) ;
		int pcoGetError() {return m_pcoData->pcoError;}

		char *_pcoSet_RecordingState(int state, int &error);
		WORD _getCameraType() {return m_pcoData->stcCamType.wCamType ; }

	private:
		SyncCtrlObj*	m_sync;

		std::string m_log;
        //char pcoErrorMsg[ERR_SIZE+1];

		struct stcPcoData *m_pcoData;

        HANDLE	m_handle;				/* handle of opened camera */
        bool m_cam_connected;
	
		int m_pcoError;

        struct stcBinning m_bin;
        struct stcRoi m_roi;
        //struct stcSize m_size;


		int		m_acq_frame_nb;

        int PcoCheckError(int err);
        void _allocBuffer();

		char *_pcoSet_Trig_Acq_Mode(int &error);
		char *_pcoSet_Storage_subRecord_Mode(int &error);
		char *_pcoSet_Exposure_Delay_Time(int &error);
		char *_pcoSet_Cameralink_GigE_Parameters(int &error);
		char *_pcoGet_Camera_Type(int &error);
		char *_pcoGet_TemperatureInfo(int &error);

		char *_pco_SetCameraSetup(DWORD dwSetup, int &error);

		int _init_edge();
		int _init_dimax();
		char *_prepare_cameralink_interface(int &error);
		char *_get_coc_runtime(int &error);
		char *_set_metadata_mode(WORD wMetaDataMode, int &error);



    };
  }
}


#endif
