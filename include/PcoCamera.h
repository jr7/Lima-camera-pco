#ifndef PCOCAMERA_H
#define PCOCAMERA_H
#include "Pco.h"
#include "Pco_errt.h"
#include "Debug.h"
#include "Constants.h"
#include "HwMaxImageSizeCallback.h"

struct stcXlatI2A {
		int code;
		char *str;
};





struct stcFrame {
	BOOL	changed;
	unsigned int nb;
	unsigned int done;

	unsigned int ram_current;

	unsigned long next2read;


};

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

struct stcTemp {
	short ccd, cam, power;
	short minCoolSet, maxCoolSet;
	short setpoint;
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

        HANDLE& getHandle() {return m_handle;}
        void getMaxWidthHeight(DWORD& width,DWORD& height){width = m_size.maxwidth, height = m_size.maxheight;}
        void getBytesPerPixel(unsigned int& pixbytes){pixbytes = m_size.pixbytes;}
        int getNbAcquiredFrames() const {return m_acq_frame_nb;}


        void	getCameraName(std::string& name);

        void getFrameRate(struct strFrameRate& frame_rate);

        void 	startAcq();
        void	reset();
		char *getInfo(char *output, int lg);
      private:
        unsigned long _getFramesMax(int segmentPco);
        int PcoCheckError(int err);

        void 		_allocBuffer();
        //static void 	_newFrameCBK(tPvFrame*);
        //void		_newFrame(tPvFrame*);

        char pcoErrorMsg[ERR_SIZE+1];


        bool 		m_cam_connected;
        HANDLE	m_handle;				/* handle of opened camera */
        char		m_camera_name[CAMERA_SIZE];
        char		m_sensor_type[64];
        //tPvUint32		m_ufirmware_maj, m_ufirmware_min;
        //tPvUint32		m_uid;
        //tPvFrame		m_frame[2];

        
        unsigned int    m_nradc, m_maxadc;
        unsigned int    m_maxwidth_step, m_maxheight_step;

        struct stcTemp m_temperature;

        DWORD   m_dwRamSize;
        WORD    m_wPageSize;
        DWORD   segSize[4];
        DWORD   m_dwSegmentSize[4];
        DWORD   dwValidImageCnt[4];
        DWORD   dwMaxImageCnt[4];
        DWORD   m_dwMaxFramesInSegment[4];
        WORD    m_activeRamSegment;				/* active ram segment */
        struct stcBinning m_bin;
        struct stcRoi m_roi;
      	//unsigned long	m_trigger;
      	WORD		m_acq_mode;
      	WORD		m_storage_mode;
      	WORD		m_recorder_submode;

          struct stcSize m_size;

	unsigned long	m_bufsize;
	unsigned long	m_bufsize_max;         // unsigned int    m_max_buffsize;

	SHORT	m_bufferNrM[8];				// bufnrM buffer number allocated by PCO_AllocateBuffer
	WORD	*m_bufferM[8];			// buffer allocated by PCO_AllocateBuffer
	HANDLE m_bufferM_events[8];


	unsigned long	m_frames_per_buffer;
	unsigned long	m_imgsizeBytes;
	unsigned long	m_imgsizePixels;
	unsigned long	m_imgsizePages;
	unsigned long	m_imgsizeBuffer;

  	double	m_cocRunTime;		/* cam operation code - delay & exposure time & readout in s*/
	  double	m_frameRate;

  SyncCtrlObj*	m_sync;

        int		m_acq_frame_nb;
        bool		m_continue_acq;

        PCO_Description	m_pcoInfo;	/* camera description structure */
        PCO_CameraType	m_stcCamType;
        char m_model[MODEL_SIZE+1], m_iface[MODEL_SIZE+1];
        int	m_interface_type;

	PCO_SC2_CL_TRANSFER_PARAM m_clTransferParam;
  struct	stcFrame	m_frame;		/* Number of frames */
	



    };
  }
}
#endif
