#ifndef PCOCAMERA_H
#define PCOCAMERA_H
#include "Pco.h"
#include "Debug.h"
#include "Constants.h"
#include "HwMaxImageSizeCallback.h"

struct stcXlatI2A {
		int code;
		char *str;
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
    class Camera : public HwMaxImageSizeCallbackGen
    {
      friend class Interface;
      DEB_CLASS_NAMESPC(DebModCamera,"Camera","Pco");
    public:
      Camera(const char*);
      ~Camera();
      
      bool isMonochrome() const;
      // tPvHandle& getHandle() {return m_handle;}
      // void getMaxWidthHeight(tPvUint32& width,tPvUint32& height){width = m_maxwidth, height = m_maxheight;}
      int getNbAcquiredFrames() const {return m_acq_frame_nb;}

      VideoMode getVideoMode() const;
      void 	setVideoMode(VideoMode);
      
      void	getCameraName(std::string& name);
	
      void 	startAcq();
      void	reset();

    private:
      int PcoCheckError(int err);
		
		void 		_allocBuffer();
      //static void 	_newFrameCBK(tPvFrame*);
      //void		_newFrame(tPvFrame*);

	char pcoErrorMsg[ERR_SIZE+1];


      bool 		m_cam_connected;
      // tPvHandle		m_handle;
      char		m_camera_name[128];
      char		m_sensor_type[64];
      //tPvUint32		m_ufirmware_maj, m_ufirmware_min;
      //tPvUint32		m_uid;
      //tPvFrame		m_frame[2];

	  unsigned int		m_maxwidth, m_maxheight;
	  unsigned int		m_pixbits, m_pixbytes;
	  unsigned int		m_nradc, m_maxadc;
		unsigned int m_max_buffsize;
		unsigned int m_maxwidth_step, m_maxheight_step;

	struct stcTemp m_temperature;
	
		SyncCtrlObj*	m_sync;
      VideoCtrlObj*	m_video;
      VideoMode		m_video_mode;
      int		m_acq_frame_nb;
      bool		m_continue_acq;

	  	HANDLE	hPco;				/* handle of opened camera */
		PCO_Description	pcoInfo;	/* camera description structure */
		PCO_CameraType	strCamType;
		char model[MODEL_SIZE+1], iface[MODEL_SIZE+1];
		int			interface_type;

    };
  }
}
#endif
