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
#include "Exceptions.h"

#include "PcoInterface.h"
#include "PcoCamera.h"
#include "PcoDetInfoCtrlObj.h"
#include "PcoBufferCtrlObj.h"
#include "PcoSyncCtrlObj.h"

using namespace lima;
using namespace lima::Pco;


//=========================================================================================================
char* _timestamp_pcointerface() {return "$Id: " __TIMESTAMP__ " (" __FILE__ ") $";}
//=========================================================================================================


//=========================================================================================================
//=========================================================================================================
Interface::Interface(Camera *cam) :
  m_cam(cam)
{
  DEB_CONSTRUCTOR();
  m_det_info = new DetInfoCtrlObj(cam);
  m_buffer = new BufferCtrlObj(cam);
  m_sync = new SyncCtrlObj(cam, m_buffer);
  cam->m_sync = m_sync;

  	DEB_TRACE() << DEB_VAR2(cam, m_buffer);

  if(m_buffer){
    m_buffer->m_sync = m_sync;
  }

}

//=========================================================================================================
//=========================================================================================================
Interface::~Interface()
{
	// DONE
	DEB_DESTRUCTOR();
  delete m_buffer;
  delete m_det_info;
  delete m_sync;
}

//=========================================================================================================
//=========================================================================================================
void Interface::getCapList(CapList &cap_list) const
{
	// DONE
  cap_list.push_back(HwCap(m_sync));
  cap_list.push_back(HwCap(m_det_info));
  cap_list.push_back(HwCap(m_buffer));
}

//=========================================================================================================
//=========================================================================================================
void Interface::reset(ResetLevel reset_level)
{
	// DONE
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(reset_level);

  m_sync->stopAcq();
  m_cam->reset();
}

//=========================================================================================================
//=========================================================================================================
void Interface::prepareAcq()
{
  DEB_MEMBER_FUNCT();
	// DONE
  if(m_buffer)
    m_buffer->prepareAcq();
}

//=========================================================================================================
//=========================================================================================================
void Interface::startAcq()
{
  DEB_MEMBER_FUNCT();
	// DONE

  if(m_buffer)
    m_buffer->getBuffer().setStartTimestamp(Timestamp::now());
  m_sync->startAcq();
}

//=========================================================================================================
//=========================================================================================================
void Interface::stopAcq()
{
	// DONE
  DEB_MEMBER_FUNCT();

  m_sync->stopAcq();
}

//=========================================================================================================
//=========================================================================================================
void Interface::getStatus(StatusType& status)
{
	// DONE
  m_sync->getStatus(status);
}

//=========================================================================================================
//=========================================================================================================
int Interface::getNbAcquiredFrames()
{
  DEB_MEMBER_FUNCT();
	// DONE

  int aNbAcquiredFrames;
  if(m_buffer)
    aNbAcquiredFrames = m_buffer->getNbAcquiredFrames();
  else
    aNbAcquiredFrames = m_cam->getNbAcquiredFrames();

  DEB_RETURN() << DEB_VAR1(aNbAcquiredFrames);
  return aNbAcquiredFrames;
}

//=========================================================================================================
//=========================================================================================================
int Interface::getNbHwAcquiredFrames()
{
	// DONE
  return getNbAcquiredFrames();
}

