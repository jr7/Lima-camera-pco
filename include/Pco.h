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
#ifndef PCO_H
#define PCO_H

#define _LINUX

#ifdef __x86_64
	#define _x64
#else
	#define _x86
#endif


#include "Compatibility.h"
#include "PCO_Structures.h"
#include "PCO_ConvStructures.h"
#include "Pco_ConvDlgExport.h"
#include "sc2_SDKStructures.h"
#include "sc2_common.h"
#include "SC2_CamExport.h"
#include "sc2_defs.h"
#include "SC2_SDKAddendum.h"
#include "PCO_errt.h"


#include <math.h>

#define ERR_SIZE	256
#define ERRMSG_SIZE	(256+128)
#define MODEL_TYPE_SIZE	32
#define INTERFACE_TYPE_SIZE	32
#define CAMERA_NAME_SIZE	128
#define MSG_SIZE	512
#define BUFF_XLAT_SIZE 128


typedef DWORD tPvUint32;
typedef int tPvErr;

#define THROW_LIMA_HW_EXC(e, x)  { \
	printf("========*** LIMA_HW_EXC %s\n", x ); \
			throw LIMA_HW_EXC(e, x); \
} 


#if 0
#define PCO_TRACE(x)  \
{ \
		if(error){ \
			char msg[ERRMSG_SIZE+1]; \
			sprintf_s(msg, "=== %s PcoError[x%08x][%s]", x, m_pcoData->pcoError,  m_pcoData->pcoErrorMsg); \
			DEB_TRACE() << msg; \
			throw LIMA_HW_EXC(Error, msg); \
		} \
		DEB_TRACE() << "*** " <<  x << " OK" ; \
}
#define _PCO_TRACE(x, s)  \
{ \
		if(error){ \
			DEB_TRACE() << "*** " <<  x << " PCO ERROR " << s; \
			throw LIMA_HW_EXC(Error, x); \
		} \
		DEB_TRACE() << "*** " <<  x << " OK" ; \
}

#endif

#define PCO_THROW_OR_TRACE(__err__, __msg__)  \
{ \
		if(__err__){ \
			char msg[ERRMSG_SIZE+1]; \
			sprintf_s(msg, "=== %s PcoError[x%08x][%s]", __msg__, m_pcoData->pcoError,  m_pcoData->pcoErrorMsg); \
			DEB_TRACE() << msg; \
			throw LIMA_HW_EXC(Error, msg); \
		} \
		DEB_TRACE() << "*** " <<  __msg__ << " OK" ; \
}


#define DEF_FNID 	static char *fnId =__FUNCTION__;

#endif
