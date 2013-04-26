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

//#include <cstdlib>
//#include <WinDef.h>
//#include <WinNt.h>
#include "Exceptions.h"
#include "PcoCamera.h"
#include "PcoRoiCtrlObj.h"


using namespace lima;
using namespace lima::Pco;

//=========================================================================================================
char* _timestamp_pcoroictrlobj() {return ID_TIMESTAMP ;}
//=========================================================================================================

//=========================================================================================================
//=========================================================================================================
RoiCtrlObj::RoiCtrlObj(Camera *cam):
  m_cam(cam),
  m_handle(cam->getHandle())
{
}

//=========================================================================================================
//=========================================================================================================
RoiCtrlObj::~RoiCtrlObj()
{
}

void RoiCtrlObj::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();
    
    Point align(24,2);
    hw_roi = set_roi;
    hw_roi.alignTo(align, Ceil);
    
}

void RoiCtrlObj::setRoi(const Roi& set_roi)
{
    DEB_MEMBER_FUNCT();
    if (set_roi.isEmpty()) {
        // Full frame
    } else {
        // Activate Roi
        Point align(24,2);
        hw_roi = set_roi;
        hw_roi.alignTo(align, Ceil);
   }
}

void RoiCtrlObj::getRoi(Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();
}
