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
#include "PcoDetInfoCtrlObj.h"


using namespace lima;
using namespace lima::Pco;

//=========================================================================================================
char* _timestamp_pcodetinfoctrlobj() {return ID_TIMESTAMP ;}
//=========================================================================================================

//=========================================================================================================
//=========================================================================================================
DetInfoCtrlObj::DetInfoCtrlObj(Camera *cam):
  m_cam(cam),
  m_handle(cam->getHandle())
{
}

//=========================================================================================================
//=========================================================================================================
DetInfoCtrlObj::~DetInfoCtrlObj()
{
}

//=========================================================================================================
//=========================================================================================================
void DetInfoCtrlObj::getMaxImageSize(Size& max_image_size)
{

  // ---- DONE
  DWORD width,height;

  m_cam->getMaxWidthHeight(width,height);
  max_image_size = Size(int(width),int(height));

}


//=========================================================================================================
//=========================================================================================================
void DetInfoCtrlObj::getDetectorImageSize(Size& det_image_size)
{
  // ---- DONE
  getMaxImageSize(det_image_size);
}


//=========================================================================================================
//=========================================================================================================
void DetInfoCtrlObj::getDefImageType(ImageType& def_image_type)
{
    // ---- DONE
  unsigned int pixbytes;
  m_cam->getBytesPerPixel(pixbytes);
    def_image_type = (pixbytes == 2) ? Bpp16 : Bpp8;
}


//=========================================================================================================
//=========================================================================================================
void DetInfoCtrlObj::getCurrImageType(ImageType& curr_image_type)
{
    // ---- DONE
  getDefImageType(curr_image_type);
}


//=========================================================================================================
//=========================================================================================================
void DetInfoCtrlObj::setCurrImageType(ImageType curr_image_type)
{
    // ---- DONE
	// only check if it valid, BUT don't set it ????
  switch(curr_image_type)
    {
    case Bpp16:
    case Bpp8:
      break;

    default:
      throw LIMA_HW_EXC(InvalidValue,"This image type is not Managed");
    }

}


//=========================================================================================================
//=========================================================================================================
#if 0
void DetInfoCtrlObj::getPixelSize(double& pixel_size)
{  
    // ---- DONE for the moment
	// pixel size in micrometer (???)
  pixel_size = -1.;		// @todo don't know
}
#endif

void DetInfoCtrlObj::getPixelSize(double& x_size,double &y_size)
{  
    // ---- TODO
	// pixel size in micrometer (???)
  x_size = y_size = -1.;		// @todo don't know

}



//=========================================================================================================
//=========================================================================================================
void DetInfoCtrlObj::getDetectorType(std::string& det_type)
{
    // ---- DONE
   det_type = "Pco";
}


//=========================================================================================================
//=========================================================================================================
void DetInfoCtrlObj::getDetectorModel(std::string& det_model)
{
    // ---- DONE
  m_cam->getCameraName(det_model);
}



//=========================================================================================================
//=========================================================================================================
void DetInfoCtrlObj::registerMaxImageSizeCallback(HwMaxImageSizeCallback& cb)
{
	// TOCHECK
	// will not be used - from HwMaxImageSizeCallbackGen::registerMaxImageSizeCallback
  m_cam->registerMaxImageSizeCallback(cb);
}


//=========================================================================================================
//=========================================================================================================
void DetInfoCtrlObj::unregisterMaxImageSizeCallback(HwMaxImageSizeCallback& cb)
{
	// TOCHECK
  m_cam->unregisterMaxImageSizeCallback(cb);
}

