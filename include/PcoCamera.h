#ifndef PCOCAMERA_H
#define PCOCAMERA_H
#include "Pco.h"
#include "Pco_errt.h"
#include "Debug.h"
#include "Constants.h"
#include "HwMaxImageSizeCallback.h"
#include "PcoBufferCtrlObj.h"

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


	double	cocRunTime;		/* cam operation code - delay & exposure time & readout in s*/
	double	frameRate;
    WORD    activeRamSegment;				/* active ram segment */

      	//WORD		m_acq_mode;
      	WORD		storage_mode;
      	WORD		recorder_submode;

        DWORD   dwRamSize;
        WORD    wPixPerPage;
        DWORD   dwMaxFramesInSegment[4];
        DWORD   dwSegmentSize[4];

		char		camera_name[CAMERA_NAME_SIZE];
        char		sensor_type[64];

        
        unsigned int    nr_adc, max_adc;
        unsigned int    maxwidth_step, maxheight_step;

        struct stcTemp temperature;

};

/***
struct stcThreadData {
	void * m_cam;
	void*	m_sync;
};
***/


struct stcSize {
  //unsigned int    m_maxwidth, m_maxheight;
  //unsigned int    m_pixbits, m_pixbytes;

  unsigned int maxwidth;		// unsigned int xmax;		/* Max size */
	unsigned int maxheight;        //unsigned int ymax;
	WORD armwidth;		//unsigned int xarm;		/* Last armed ccd size */
	WORD armheight;  //unsigned int yarm;
	unsigned int pixbytes;     //unsigned int depth;
	unsigned int pixbits;        //unsigned int bits;



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






char* _PcoCheckError(int err) ;

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

        HANDLE& getHandle() {return m_handle;}
        void getMaxWidthHeight(DWORD& width,DWORD& height){width = m_size.maxwidth, height = m_size.maxheight;}
        void getBytesPerPixel(unsigned int& pixbytes){pixbytes = m_size.pixbytes;}
        int getNbAcquiredFrames() const {return m_acq_frame_nb;}


        void	getCameraName(std::string& name);

        void getFrameRate(struct strFrameRate& frame_rate);

        void 	startAcq();
        void	reset();

        char *getInfo(char *output, int lg);
        void getArmWidthHeight(WORD& width,WORD& height){width = m_size.armwidth, height = m_size.armheight;}

		void getBitsPerPixel(WORD& pixbits){pixbits = (WORD) m_size.pixbits;}
        unsigned long _getFramesMax(int segmentPco);
	

  
		struct stcPcoData * _getPcoData() {return & m_pcoData; }

		unsigned long	pcoGetFramesPerBuffer() { return m_frames_per_buffer; }

		SyncCtrlObj*	_getSyncCtrlObj() { return m_sync;}
		WORD _getActiveRamSegment() {return m_pcoData.activeRamSegment;}

    private:
		std::string log;
        int PcoCheckError(int err);
	struct stcPcoData m_pcoData;

        void 		_allocBuffer();

        //void _assignImage2Buffer(DWORD &dwFrameFirst, DWORD &dwFrameLast, DWORD dwRequestedFrames, int bufIdx);
        //void _xferImag();

		char *_pcoSet_Trig_Acq_Mode(int &error);
		char *_pcoSet_Storage_subRecord_Mode(int &error);
		char *_pcoSet_Exposure_Delay_Time(int &error);
		char *_pcoSet_Cameralink_GigE_Parameters(int &error);


        char pcoErrorMsg[ERR_SIZE+1];


        bool 		m_cam_connected;
        HANDLE	m_handle;				/* handle of opened camera */


		DWORD   segSize[4];
        DWORD   dwValidImageCnt[4];
        DWORD   dwMaxImageCnt[4];
        struct stcBinning m_bin;
        struct stcRoi m_roi;
      	//unsigned long	m_trigger;

          struct stcSize m_size;

	unsigned long	m_allocatedBufferSize;
	unsigned long	m_allocatedBufferSizeMax;         // unsigned int    m_max_buffsize;

  /**********

        char		m_camera_name[CAMERA_NAME_SIZE];
        char		m_sensor_type[64];

        
        unsigned int    m_nradc, m_maxadc;
        unsigned int    m_maxwidth_step, m_maxheight_step;

        struct stcTemp m_temperature;


        DWORD   m_dwRamSize;
        WORD    m_wPixPerPage;
        DWORD   m_dwMaxFramesInSegment[4];
        DWORD   m_dwSegmentSize[4];

      	WORD		m_acq_mode;
      	WORD		m_storage_mode;
      	WORD		m_recorder_submode;


  SHORT	m_allocatedBufferNr[8];				// bufnrM buffer number allocated by PCO_AllocateBuffer
	WORD	*m_allocatedBufferPtr[8];			// buffer allocated by PCO_AllocateBuffer
	HANDLE m_allocatedBufferEvent[8];

  
	DWORD m_allocatedBufferAssignedFrameFirst[8];
	DWORD m_allocatedBufferAssignedFrameLast[8];
	int m_allocatedBufferReady[8];

  
  ************/

	unsigned long	m_frames_per_buffer;

	unsigned long	m_imgsizeBytes;
	unsigned long	m_imgsizePixels;
	unsigned long	m_imgsizePages;
	unsigned long	m_imgsizeBuffer;

  	//double	m_cocRunTime;		/* cam operation code - delay & exposure time & readout in s*/
	//double	m_frameRate;

  SyncCtrlObj*	m_sync;

        int		m_acq_frame_nb;
        bool		m_continue_acq;

  /**********
        PCO_Description	m_pcoInfo;	// camera description structure 
        PCO_CameraType	m_stcCamType;
        char m_model[MODEL_TYPE_SIZE+1], m_iface[INTERFACE_TYPE_SIZE+1];
        int	m_interface_type;

	PCO_SC2_CL_TRANSFER_PARAM m_clTransferParam;
  ************/

	struct	stcFrame	m_frame;		/* Number of frames */
	
    };
  }
}




#endif
