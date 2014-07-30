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
	DEB_CONSTRUCTOR();
}

//=========================================================================================================
//=========================================================================================================
RoiCtrlObj::~RoiCtrlObj()
{
}

void RoiCtrlObj::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();
	
    Point align; m_cam->_get_XYsteps(align);

    hw_roi = set_roi;
    hw_roi.alignCornersTo(align, Ceil);

	if(m_cam->_getDebug(DBG_ROI)) {DEB_ALWAYS() << DEB_VAR3(align, set_roi, hw_roi);}

}

void RoiCtrlObj::setRoi(const Roi& set_roi)
{
    DEB_MEMBER_FUNCT();

	Roi hw_roi;
	int error = 0;
	int iRoi_error = 0;
	
    if (set_roi.isEmpty()) {
		m_cam->_get_MaxRoi(hw_roi);
	} else {
		hw_roi = set_roi;
		iRoi_error = m_cam->_checkValidRoi(set_roi);
	}

	if(m_cam->_getDebug(DBG_ROI)) {DEB_ALWAYS() << DEB_VAR3(set_roi, hw_roi, iRoi_error);}

	if(iRoi_error == 0){
		m_cam->_set_Roi(hw_roi, error);
		if(error) {DEB_ALWAYS() << "m_cam->_set_Roi " << DEB_VAR2(hw_roi, error);}
	} else {
		DEB_ALWAYS() << "ERROR - invalid ROI " << DEB_VAR2(set_roi, iRoi_error);
		throw LIMA_HW_EXC(InvalidValue, "Invalid ROI ") << DEB_VAR2(set_roi, iRoi_error);
	}
}

void RoiCtrlObj::getRoi(Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();

	m_cam->_get_Roi(hw_roi);
	if(m_cam->_getDebug(DBG_ROI)) {DEB_ALWAYS() << "m_cam->_get_Roi " << DEB_VAR1(hw_roi);}

}
