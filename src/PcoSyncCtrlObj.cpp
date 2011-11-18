#include <sstream>
#include "Exceptions.h"
#include "PcoSyncCtrlObj.h"
#include "PcoBufferCtrlObj.h"
#include "PcoCamera.h"

#define THROW_LIMA_HW_EXC(Error, x)  { \
	printf("========*** LIMA_HW_EXC %s\n", x ); \
			throw LIMA_HW_EXC(Error, x); \
} 


using namespace lima;
using namespace lima::Pco;


SyncCtrlObj::SyncCtrlObj(Camera *cam,BufferCtrlObj *buffer) :
  m_cam(cam),
  m_handle(cam->getHandle()),
  m_trig_mode(IntTrig),
  m_buffer(buffer),
  m_nb_frames(1),
  m_started(false)
{
	// DONE
  DEB_CONSTRUCTOR();
}

SyncCtrlObj::~SyncCtrlObj()
{
	// DONE
  DEB_DESTRUCTOR();
}

bool SyncCtrlObj::checkTrigMode(TrigMode trig_mode)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(trig_mode);

	// DONE
  switch(trig_mode){
    case IntTrig:
    case IntTrigMult:
    case ExtTrigSingle:
      return true;

    default:
      return false;
    }
}

void SyncCtrlObj::setTrigMode(TrigMode trig_mode)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(trig_mode);

	// DONE
  if(checkTrigMode(trig_mode)){
    switch(trig_mode)	{
      case IntTrig: // 0 SOFT (spec)
	  case IntTrigMult: // 1 START (spec)
      case ExtTrigSingle:  // 2 GATE (spec)
    	  break;
	}
  }  else {
    throw LIMA_HW_EXC(NotSupported,"Trigger type not supported");
  }

	m_trig_mode = trig_mode;

}

void SyncCtrlObj::getTrigMode(TrigMode &trig_mode)
{
	// DONE
  trig_mode = m_trig_mode;
}




WORD SyncCtrlObj::getPcoAcqMode()
{
	// DONE
  DEB_MEMBER_FUNCT();

    switch( m_trig_mode)	{
	  case IntTrig: // 0 SOFT (spec)
    	  return 0x0000;

	  case IntTrigMult: // 1 START (spec)
	  case ExtTrigSingle:  // 2 GATE (spec)
    	  return 0x0001;

	  default:
		 throw LIMA_HW_EXC(NotSupported,"Invalid value");

	}

}

/*************
enum TrigMode { IntTrig 0, IntTrigMult 1, ExtTrigSingle 2, ExtTrigMult 3,
				ExtGate 4, ExtStartStop 5, Live 6, };
*******************/

WORD SyncCtrlObj::getPcoTrigMode(){
	// DONE

	// xlat from lima trig mode to PCO trig mode
  	//------------------------------------------------- triggering mode 
	switch (m_trig_mode) {  // trig mode in spec
			//  PCO = 0x0000
			// A new image exposure is automatically started best possible compared to
			// the readout of an image. If a CCD is used and the images are taken in a
			// sequence, then exposures and sensor readout are started simultaneously.
			// Signals at the trigger input (<exp trig>) are irrelevant.
		case IntTrig: return 0x0000;  // 0 = SOFT (spec)

			// PCO = 0x0002
			// A delay / exposure sequence is started at the RISING or FALLING edge
			// (depending on the DIP switch setting) of the trigger input (<exp trig>).
		case IntTrigMult: return 0x0002;   // 1 = START (spec)
			
			// PCO = 0x0003
			// The exposure time is defined by the pulse length at the trigger
			// input(<exp trig>). The delay and exposure time values defined by the
			// set/request delay and exposure command are ineffective. (Exposure
			// time length control is also possible for double image mode; exposure
			// time of the second image is given by the readout time of the first image.)
		case ExtTrigSingle: return 0x0003;  // 2 = GATE (spec)
		
    default: return 0x0000;			  // SOFT
	}
}


void SyncCtrlObj::setExpTime(double exp_time)
{
	// DONE

	ValidRangesType valid_ranges;
  getValidRanges(valid_ranges);

  if ((exp_time <valid_ranges.min_exp_time)||(exp_time >valid_ranges.max_exp_time)){ 
     THROW_LIMA_HW_EXC(Error,"Exposure time out of range");
  }

  m_exp_time = exp_time;
  //ds->ccd.cocRunTime = 0;


}

void SyncCtrlObj::getExpTime(double &exp_time)
{
  DEB_MEMBER_FUNCT();

  ValidRangesType valid_ranges;
  getValidRanges(valid_ranges);
  
  if (m_exp_time < valid_ranges.min_exp_time) m_exp_time = valid_ranges.min_exp_time;
  if (m_exp_time > valid_ranges.max_exp_time) m_exp_time = valid_ranges.max_exp_time;

  exp_time = m_exp_time;
  DEB_RETURN() << DEB_VAR1(exp_time);
}




void SyncCtrlObj::setLatTime(double  lat_time)
{
  //No latency managed
  //delay ???

  m_lat_time = lat_time;
}

void SyncCtrlObj::getLatTime(double& lat_time)
{
  m_lat_time = 0.;
  lat_time = m_lat_time;		// Don't know - delay????
}




void SyncCtrlObj::setNbFrames(int  nb_frames)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(nb_frames);

  m_nb_frames = nb_frames;
}

void SyncCtrlObj::getNbFrames(int& nb_frames)
{
  nb_frames = m_nb_frames;
}

// these two functions calls the upper ones get/setNbFrames
void SyncCtrlObj::setNbHwFrames(int  nb_frames)
{
  setNbFrames(nb_frames);
}

void SyncCtrlObj::getNbHwFrames(int& nb_frames)
{
  getNbFrames(nb_frames);
}





void SyncCtrlObj::getValidRanges(ValidRangesType& valid_ranges)
{
	// DONE
  valid_ranges.min_exp_time = m_cam->m_pcoData.pcoInfo.dwMinExposureDESC * 1e-9 ; // Don't know
  valid_ranges.max_exp_time = m_cam->m_pcoData.pcoInfo.dwMaxExposureDESC * 1e-3 ; // Don't know
  valid_ranges.min_lat_time = 0.; // Don't know
  valid_ranges.max_lat_time = 0.; // Don't know
}

void SyncCtrlObj::startAcq()
{
  tPvErr error=0;
  DEB_MEMBER_FUNCT();
  if(!m_started)
    {
      //tPvErr error = PvCaptureStart(m_handle);
      if(error)
	throw LIMA_HW_EXC(Error,"Can't start acquisition capture");

      //error = PvCommandRun(m_handle, "AcquisitionStart");
      if(error)
	throw LIMA_HW_EXC(Error,"Can't start acquisition");
  
      if(m_buffer)
	m_buffer->startAcq();
      else
	m_cam->startAcq();
    }
  m_started = true;
}

void SyncCtrlObj::stopAcq(bool clearQueue)
{
    tPvErr error=0;

  DEB_MEMBER_FUNCT();
  if(m_started)
    {
      DEB_TRACE() << "Try to stop Acq";
      //tPvErr error = PvCommandRun(m_handle,"AcquisitionStop");
      if(error)
	{
	  DEB_ERROR() << "Failed to stop acquisition";
	  throw LIMA_HW_EXC(Error,"Failed to stop acquisition");
	}

      DEB_TRACE() << "Try to stop Capture";
      //error = PvCaptureEnd(m_handle);
      if(error)
	{
	  DEB_ERROR() << "Failed to stop acquisition";
	  throw LIMA_HW_EXC(Error,"Failed to stop acquisition");
	}

      if(clearQueue)
	{
	  DEB_TRACE() << "Try to clear queue";
	  //error = PvCaptureQueueClear(m_handle);
	  if(error)
	    {
	      DEB_ERROR() << "Failed to stop acquisition";
	      throw LIMA_HW_EXC(Error,"Failed to stop acquisition");
	    }
	}
    }
  m_started = false;
}

void SyncCtrlObj::getStatus(HwInterface::StatusType& status)
{
    tPvErr error;

  DEB_MEMBER_FUNCT();
  if(m_started)
    {
      //tPvErr error = ePvErrSuccess;
      if(m_buffer)
	{
	  bool exposing;
	  m_buffer->getStatus(error,exposing);
	  if(error)
	    {
	      status.acq = AcqFault;
	      status.det = DetFault;
	    }
	  else
	    {
	      status.acq = AcqRunning;
	      status.det = exposing ? DetExposure : DetIdle;
	    }
	}
      else			// video mode, don't need to be precise
	{
	  status.acq = AcqRunning;
	  status.det = DetExposure;
	}
    }
  else
    {
      status.acq = AcqReady;
      status.det = DetIdle;
    }
  DEB_RETURN() << DEB_VAR1(status);
}
