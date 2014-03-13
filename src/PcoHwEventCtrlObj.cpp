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
#include "PcoCamera.h"
#include "PcoHwEventCtrlObj.h"


using namespace lima;
using namespace lima::Pco;

//=========================================================================================================
char* _timestamp_pcohweventctrlobj() {return ID_TIMESTAMP ;}
//=========================================================================================================

//=========================================================================================================
//=========================================================================================================
PcoHwEventCtrlObj::PcoHwEventCtrlObj(Camera *cam):
  m_cam(cam),
  m_handle(cam->getHandle())
{
	DEB_CONSTRUCTOR();
}

//=========================================================================================================
//=========================================================================================================
PcoHwEventCtrlObj::~PcoHwEventCtrlObj()
{
}

//=========================================================================================================
//=========================================================================================================
void PcoHwEventCtrlObj::registerEventCallback(EventCallback& cb)
{


  DEB_MEMBER_FUNCT();
  DEB_ALWAYS() << " FUNCTION entry";

  EventCallbackGen::registerEventCallback(cb);

  DEB_ALWAYS() << " FUNCTION exit";
}

//=========================================================================================================
//=========================================================================================================
void PcoHwEventCtrlObj::unregisterEventCallback(EventCallback& cb)
{	
  DEB_MEMBER_FUNCT();
  DEB_ALWAYS() << "--- FUNCTION ---";
}

//=========================================================================================================
//=========================================================================================================
bool PcoHwEventCtrlObj::hasRegisteredCallback()
{
  DEB_MEMBER_FUNCT();
  DEB_ALWAYS() << " FUNCTION entry";
  return EventCallbackGen::hasRegisteredCallback();
  DEB_ALWAYS() << " FUNCTION exit";
}


//=========================================================================================================
//=========================================================================================================
void PcoHwEventCtrlObj::reportEvent(Event *event)
{	
  DEB_MEMBER_FUNCT();
  DEB_ALWAYS() << " FUNCTION entry";
  EventCallbackGen::reportEvent(event);
  DEB_ALWAYS() << " FUNCTION exit";
}

