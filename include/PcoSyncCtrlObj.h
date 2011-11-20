#ifndef PCOSYNCCTRLOBJ_H
#define PCOSYNCCTRLOBJ_H

#include "Pco.h"

#include "HwSyncCtrlObj.h"
#include "HwInterface.h"

namespace lima
{
  namespace Pco
  {
    class Camera;
    class BufferCtrlObj;

    class  DLL_EXPORT  SyncCtrlObj : public HwSyncCtrlObj
    {
      DEB_CLASS_NAMESPC(DebModCamera,"SyncCtrlObj","Pco");
    public:
      SyncCtrlObj(Camera*, BufferCtrlObj*);
      virtual ~SyncCtrlObj();

      virtual bool checkTrigMode(TrigMode trig_mode);
      virtual void setTrigMode(TrigMode  trig_mode);
      virtual void getTrigMode(TrigMode& trig_mode);

      virtual void setExpTime(double  exp_time);
      virtual void getExpTime(double& exp_time);

      virtual void setLatTime(double  lat_time);
      virtual void getLatTime(double& lat_time);

      virtual void setNbFrames(int  nb_frames);
      virtual void getNbFrames(int& nb_frames);
			// these two functions calls the upper ones get/setNbFrames
      virtual void setNbHwFrames(int  nb_frames);
      virtual void getNbHwFrames(int& nb_frames);

	  void setAcqFrames(int  nb_acq_frames) { m_nb_acq_frames = nb_acq_frames; }
      void getAcqFrames(int& nb_acq_frames){ nb_acq_frames = m_nb_acq_frames; }

      virtual void getValidRanges(ValidRangesType& valid_ranges);

      void startAcq();
      void stopAcq(bool clearQueue = true);
      
      void getStatus(HwInterface::StatusType&);

		WORD getPcoTrigMode();
		WORD getPcoAcqMode();
	
		BufferCtrlObj* _getBufferCtrlObj() {return m_buffer;}
       void setExposing(bool exposing) {m_exposing = exposing;}
		bool getExposing() {return m_exposing;}

	
	private:


     	double	m_exp_time;			/* exposure time in s */
     	double	m_lat_time;			/* lattency - delay? */

      Camera*		m_cam;
      HANDLE&	m_handle;
      TrigMode		m_trig_mode;
      BufferCtrlObj*	m_buffer;
      int		m_nb_frames;
	  int m_nb_acq_frames;
	  bool		m_started;
	bool m_exposing;
	
	};

  } // namespace Pco
} // namespace lima

#endif // PCOSYNCCTRLOBJ_H
