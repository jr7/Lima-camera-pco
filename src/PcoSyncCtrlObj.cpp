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
#include <sstream>
#include "Exceptions.h"
#include "Pco.h"
#include "PcoCamera.h"
#include "PcoSyncCtrlObj.h"
#include "PcoBufferCtrlObj.h"

#define THROW_LIMA_HW_EXC(Error, x)  { \
	printf("========*** LIMA_HW_EXC %s\n", x ); \
			throw LIMA_HW_EXC(Error, x); \
} 


using namespace lima;
using namespace lima::Pco;

//=========================================================================================================
char* _timestamp_pcosyncctrlobj() {return ID_TIMESTAMP ;}
//=========================================================================================================

//=========================================================================================================
//=========================================================================================================
SyncCtrlObj::SyncCtrlObj(Camera *cam,BufferCtrlObj *buffer) :
  m_cam(cam),
  m_handle(cam->getHandle()),
  m_pcoData(cam->_getPcoData()),
  m_trig_mode(IntTrig),
  m_buffer(buffer),
  m_lat_time(0.), m_exp_time(0.),
  m_nb_frames(1),
  m_exposing(pcoAcqIdle),
  m_started(false)
{
	// DONE
  DEB_CONSTRUCTOR();
}

//=========================================================================================================
//=========================================================================================================
SyncCtrlObj::~SyncCtrlObj()
{
	// DONE
  DEB_DESTRUCTOR();
}

//=========================================================================================================
//=========================================================================================================
bool SyncCtrlObj::checkTrigMode(TrigMode trig_mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(trig_mode);

	// DONE
	switch(trig_mode){
		case IntTrig:
		//case IntTrigMult:
		case ExtTrigMult:
		case ExtTrigSingle:
		case ExtGate:
			return true;

		default: break;
	}

	DEB_ALWAYS() << "Trig mode not allowed" << DEB_VAR1(trig_mode);
	return false;
}

//=========================================================================================================
//=========================================================================================================
void SyncCtrlObj::setTrigMode(TrigMode trig_mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(trig_mode);

	// DONE

	if(!checkTrigMode(trig_mode)){
		throw LIMA_HW_EXC(NotSupported,"Trigger type not supported");
	}

	m_trig_mode = trig_mode;
}

//=========================================================================================================
//=========================================================================================================
void SyncCtrlObj::getTrigMode(TrigMode &trig_mode)
{
	// DONE
  trig_mode = m_trig_mode;
}

//=========================================================================================================
//=========================================================================================================
WORD SyncCtrlObj::xlatLimaTrigMode2PcoAcqMode()
{
	WORD ret;

	// DONE
  DEB_MEMBER_FUNCT();


	if(!checkTrigMode(m_trig_mode)){
		throw LIMA_HW_EXC(NotSupported,"Trigger type not supported");
	}


// · acquire mode to be selected:
// - 0x0000 = [auto] - all images taken are stored

// - 0x0001 = [external] - the external control input <acq enbl> is a static enable signal of
//            images. If this input is TRUE (level depending on the DIP switch), exposure triggers are
//            accepted and images are taken. If this signal is set FALSE, all exposure triggers are
//            ignored and the sensor readout is stopped.

// - 0x0002 = [external] - the external control input <acq enbl> is a dynamic frame start
//            signal. If this input has got a rising edge TRUE (level depending on the DIP switch), a
//            frame will be started with modulation mode. This is only available with modulation mode
//            enabled (see camera description).


    switch( m_trig_mode)	{
	  case IntTrig: // 0 SOFT (spec)
    	  ret= 0x0000;
			break;

		case ExtTrigMult:
		//case IntTrigMult: // 1 START (spec)
	  case ExtTrigSingle:  // 2 GATE (spec)
      case ExtGate:  // 2 GATE (spec)
#ifdef DISABLE_ACQ_ENBL_SIGNAL
    	  ret= 0x0000;
#else
		  ret= 0x0001;
#endif
		  break;

	  default:
		 throw LIMA_HW_EXC(NotSupported,"Invalid value");

	}

	DEB_ALWAYS() << "returned values" << DEB_VAR2(ret, m_trig_mode);
	return ret;

}

//=========================================================================================================
//=========================================================================================================
/*************
enum TrigMode { IntTrig 0, IntTrigMult 1, ExtTrigSingle 2, ExtTrigMult 3,
				ExtGate 4, ExtStartStop 5, Live 6, };
*******************/

WORD SyncCtrlObj::xlatLimaTrigMode2PcoTrigMode(bool &ext_trig){
	// DONE

	WORD ret;

	DEB_MEMBER_FUNCT();

	if(!checkTrigMode(m_trig_mode)){
		throw LIMA_HW_EXC(NotSupported,"Trigger type not supported");
	}


	// xlat from lima trig mode to PCO trig mode
  	//------------------------------------------------- triggering mode 
	switch (m_trig_mode) {  // trig mode in spec
			//  PCO = 0x0000
			// A new image exposure is automatically started best possible compared to
			// the readout of an image. If a CCD is used and the images are taken in a
			// sequence, then exposures and sensor readout are started simultaneously.
			// Signals at the trigger input (<exp trig>) are irrelevant.
	    default: 
		case IntTrig: 
			ext_trig = false;
			ret= 0x0000;  // 0 = SOFT (spec)
			break;

			// PCO = 0x0002
			// A delay / exposure sequence is started at the RISING or FALLING edge
			// (depending on the DIP switch setting) of the trigger input (<exp trig>).
		//case IntTrigMult: return 0x0002;   // 1 = START (spec)
		case ExtTrigMult: 
			ext_trig = true;
			ret= 0x0002;   // 1 = START (spec)
			break;

			// PCO = 0x0003
			// The exposure time is defined by the pulse length at the trigger
			// input(<exp trig>). The delay and exposure time values defined by the
			// set/request delay and exposure command are ineffective. (Exposure
			// time length control is also possible for double image mode; exposure
			// time of the second image is given by the readout time of the first image.)
		case ExtTrigSingle: 
		case ExtGate: 
			ext_trig = true;
			ret= 0x0003;  // 2 = GATE (spec)
			break;

	
	}


	DEB_ALWAYS() << "returned values" << DEB_VAR2(ret, m_trig_mode);

	return ret;


}


//=========================================================================================================
//=========================================================================================================
void SyncCtrlObj::setExpTime(double exp_time)
{
	// DONE
	DEB_MEMBER_FUNCT();

	ValidRangesType valid_ranges;
  getValidRanges(valid_ranges);
  double diff;


	if ((exp_time >= valid_ranges.min_exp_time) &&
				(exp_time <= valid_ranges.max_exp_time))	{
		m_exp_time = exp_time;
		return;
	}

	if (exp_time < m_pcoData->min_exp_time_err){
	  diff = exp_time - valid_ranges.min_exp_time;
	  DEB_ALWAYS() << "Exposure time out of range (exp < min): "
		<< DEB_VAR3(diff, exp_time, valid_ranges.min_exp_time);
	  THROW_HW_ERROR(NotSupported) << "Exposure time out of range" ;
	}

	if (exp_time > m_pcoData->max_exp_time_err){ 
		diff = exp_time - valid_ranges.max_exp_time;
		DEB_ALWAYS() << "Exposure time out of range (exp > max): "
			<< DEB_VAR3(diff, exp_time, valid_ranges.max_exp_time);
		THROW_HW_ERROR(NotSupported) << "Exposure time out of range" ;
	}

	if (exp_time < valid_ranges.min_exp_time){
		m_exp_time = m_pcoData->min_exp_time;
		DEB_ALWAYS() << "Exp time fixed " << DEB_VAR2(m_exp_time, exp_time);
		return;
	}

	m_exp_time = m_pcoData->max_exp_time;
	DEB_ALWAYS() << "Exp time fixed " << DEB_VAR2(m_exp_time, exp_time);
	return;
	
	

}

//=========================================================================================================
//=========================================================================================================
void SyncCtrlObj::getExpTime(double &exp_time)
{
  DEB_MEMBER_FUNCT();

  ValidRangesType valid_ranges;
  getValidRanges(valid_ranges);
	// DONE
  
  if (m_exp_time < valid_ranges.min_exp_time) m_exp_time = valid_ranges.min_exp_time;
  else if (m_exp_time > valid_ranges.max_exp_time) m_exp_time = valid_ranges.max_exp_time;
  

  exp_time = m_exp_time;
  DEB_RETURN() << DEB_VAR1(exp_time);
}

//=========================================================================================================
//=========================================================================================================
void SyncCtrlObj::setLatTime(double  lat_time)
{
	// DONE
  DEB_MEMBER_FUNCT();
  //delay ???

  m_lat_time = lat_time;
  DEB_PARAM() << DEB_VAR2(m_lat_time, lat_time);
}

//=========================================================================================================
//=========================================================================================================
void SyncCtrlObj::getLatTime(double& lat_time)
{
	// DONE
  DEB_MEMBER_FUNCT();

  //m_lat_time = 0.;
  lat_time = m_lat_time;		// Don't know - delay????
  DEB_PARAM() << DEB_VAR2(m_lat_time, lat_time);
}

//=========================================================================================================
//=========================================================================================================
void SyncCtrlObj::setNbFrames(int  nb_frames)
{
	// DONE
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(nb_frames);

  m_nb_frames = nb_frames;
}

//=========================================================================================================
//=========================================================================================================
void SyncCtrlObj::getNbFrames(int& nb_frames)
{
	// DONE
  nb_frames = m_nb_frames;
}

//=========================================================================================================
//=========================================================================================================
// these two functions calls the upper ones get/setNbFrames
void SyncCtrlObj::setNbHwFrames(int  nb_frames)
{
	// DONE
  setNbFrames(nb_frames);
}

//=========================================================================================================
//=========================================================================================================
void SyncCtrlObj::getNbHwFrames(int& nb_frames)
{
	// DONE
  getNbFrames(nb_frames);
}

//=========================================================================================================
//=========================================================================================================
void SyncCtrlObj::getValidRanges(ValidRangesType& valid_ranges)
{
	DEF_FNID;
	// DONE

/*
	valid_ranges.min_exp_time = float(m_pcoData->stcPcoDescription.dwMinExposureDESC * 1e-9) ;	//Minimum exposure time in ns
	valid_ranges.max_exp_time = float(m_pcoData->stcPcoDescription.dwMaxExposureDESC * 1e-3) ;   // Maximum exposure time in ms  

	valid_ranges.min_lat_time = float(m_pcoData->stcPcoDescription.dwMinDelayDESC * 1e-9) ; // Minimum delay time in ns
	valid_ranges.max_lat_time = float(m_pcoData->stcPcoDescription.dwMaxDelayDESC * 1e-3) ; // Maximum delay time in ms
*/


	m_pcoData->min_exp_time = (m_pcoData->stcPcoDescription.dwMinExposureDESC) * 1e-9 ;	//Minimum exposure time in ns
	valid_ranges.min_exp_time = m_pcoData->min_exp_time_err = m_pcoData->min_exp_time - 1e-9 ;	

	m_pcoData->max_exp_time = (m_pcoData->stcPcoDescription.dwMaxExposureDESC) * 1e-3 ;   // Maximum exposure time in ms  
	valid_ranges.max_exp_time = m_pcoData->max_exp_time_err = m_pcoData->max_exp_time + 1e-3 ;	

	m_pcoData->min_lat_time = (m_pcoData->stcPcoDescription.dwMinDelayDESC) * 1e-9 ; // Minimum delay time in ns
	valid_ranges.min_lat_time = m_pcoData->min_lat_time_err = m_pcoData->min_lat_time - 1e-9 ;	

	m_pcoData->max_lat_time = (m_pcoData->stcPcoDescription.dwMaxDelayDESC) * 1e-3 ; // Maximum delay time in ms
	valid_ranges.max_lat_time = m_pcoData->max_lat_time_err = m_pcoData->max_lat_time + 1e-3 ;	


}

//=========================================================================================================
//=========================================================================================================
void SyncCtrlObj::startAcq()
{
  DEB_MEMBER_FUNCT();
  
  m_cam->msgLog("startAcq");

  if(!m_started)
    {
  
		if(m_buffer) {
			m_buffer->startAcq();
			m_started = true;
			m_buffer->_setRequestStop(false);
		}
      //else m_cam->startAcq();
    }
}

//=========================================================================================================
//=========================================================================================================
void SyncCtrlObj::stopAcq(bool clearQueue)
{
	int error;

  DEB_MEMBER_FUNCT();
  DEF_FNID;

  m_cam->msgLog("stopAcq");

  if(m_started)
    {
		//if(m_buffer->_getRequestStop()) return;
		m_buffer->_setRequestStop(true);

		m_cam->_pcoSet_RecordingState(0, error);
		PCO_THROW_OR_TRACE(error, "Try to stop Acq") ;

    //  if(clearQueue) - ignored
    }

	m_started = false;

}


//=========================================================================================================
//=========================================================================================================
void SyncCtrlObj::getStatus(HwInterface::StatusType& status)
{
	// DONE
  DEB_MEMBER_FUNCT();
DEB_TRACE() << DEB_VAR3(m_started, m_buffer, m_exposing);
 DEF_FNID;
  if(m_started){
      if(m_buffer){

		  switch(m_exposing) {

			case pcoAcqStart: 
			case pcoAcqRecordStart: 
			  status.acq = AcqRunning;
			  status.det = DetExposure;
			  break;

			case pcoAcqRecordEnd:  
			case pcoAcqTransferStart: 
			  status.acq = AcqRunning;
			  status.det = DetIdle;
			  break;

			case pcoAcqRecordTimeout:
			case pcoAcqWaitTimeout:
			case pcoAcqWaitError:
			case pcoAcqError:
			case pcoAcqPcoError:
			  status.acq = AcqFault;
			  status.det = DetFault;
			  break;


			case pcoAcqStop: 
			case pcoAcqTransferStop: 
			case pcoAcqIdle: 
			case pcoAcqTransferEnd: 
		      status.acq = AcqReady;
			  status.det = DetIdle;
			  break;


			default:
					THROW_HW_ERROR(NotSupported) << "Undefined value";
		  } // sw
		} // m_buffer
    } else { // not started
      status.acq = AcqReady;
      status.det = DetIdle;
    }

  DEB_RETURN() << DEB_VAR1(status);
}
