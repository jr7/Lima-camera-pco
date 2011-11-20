#ifndef PCOCAMERA_H
#define PCOCAMERA_H
#include "Pco.h"
//#include "PcoCamera.h"
#include "Pco_errt.h"
#include "Debug.h"
#include "Constants.h"
#include "HwMaxImageSizeCallback.h"
//#include "PcoBufferCtrlObj.h"

#define PCO_BUFFER_NREVENTS 2
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
		WORD armWidth;		//unsigned int xarm;		/* Last armed ccd size */
		WORD armHeight;  //unsigned int yarm;


};


//struct stcSize {
  //unsigned int    m_maxwidth, m_maxheight;
  //unsigned int    m_pixbits, m_pixbytes;

  //unsigned int maxwidth;		// unsigned int xmax;		/* Max size */
	//unsigned int maxheight;        //unsigned int ymax;
	//WORD armwidth;		//unsigned int xarm;		/* Last armed ccd size */
	//WORD armheight;  //unsigned int yarm;
	//unsigned int pixbytes;     //unsigned int depth;
	//unsigned int pixbits;        //unsigned int bits;
//};

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
        Camera(const char*);
        ~Camera();

        void 	startAcq();
        void	reset();

		HANDLE& getHandle() {return m_handle;}

        void getMaxWidthHeight(DWORD& width,DWORD& height){width = m_pcoData.maxWidth, height = m_pcoData.maxHeight;}
        void getArmWidthHeight(WORD& width,WORD& height){width = m_pcoData.armWidth, height = m_pcoData.armHeight;}
        void getBytesPerPixel(unsigned int& pixbytes){pixbytes = m_pcoData.bytesPerPix;}
		void getBitsPerPixel(WORD& pixbits){pixbits = (WORD) m_pcoData.bitsPerPix;}

		int getNbAcquiredFrames() const {return m_acq_frame_nb;}

        void getCameraName(std::string& name);

        char *getInfo(char *output, int lg);

        unsigned long pcoGetFramesMax(int segmentPco);

		unsigned long	pcoGetFramesPerBuffer() { return m_pcoData.frames_per_buffer; }
		double pcoGetCocRunTime() { return m_pcoData.cocRunTime; }
		double pcoGetFrameRate() { return m_pcoData.frameRate; }

		WORD pcoGetActiveRamSegment() {return m_pcoData.activeRamSegment;}

		SyncCtrlObj*	_getSyncCtrlObj() { return m_sync;}
		struct stcPcoData * _getPcoData() {return & m_pcoData; }
		char* _PcoCheckError(int err) ;
		int pcoGetError() {return m_pcoData.pcoError;}

		char *_pcoSet_RecordingState(int state, int &error);

	private:
		SyncCtrlObj*	m_sync;

		std::string m_log;
        //char pcoErrorMsg[ERR_SIZE+1];

		struct stcPcoData m_pcoData;

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


    };
  }
}




#endif
