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
#include "SC2_CamExport.h"
#include "sc2_defs.h"
#include "SC2_SDKAddendum.h"
#include "PCO_errt.h"


#include <math.h>

#define ERR_SIZE	256
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


#define PCO_TRACE(x)  \
{ \
		if(error){ \
			DEB_TRACE() << "*** " <<  x << " PCO ERROR " << pcoErrorMsg; \
			throw LIMA_HW_EXC(Error, x); \
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
