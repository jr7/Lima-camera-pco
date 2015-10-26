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

#ifdef __linux__


#include <stdint.h>
typedef unsigned long       DWORD;
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef bool                BOOL;

typedef void *HANDLE;

typedef short SHORT;
typedef long LONG;

typedef uint64_t UINT64;

#ifndef ULLONG_MAX
#define LONG_MAX      2147483647L
#define ULONG_MAX     0xffffffffUL
#define ULLONG_MAX    0xffffffffffffffffULL      /* maximum unsigned long long int value */
#endif

#define far

#define DECLARE_HANDLE(n) typedef struct n##__{int i;}*n

DECLARE_HANDLE(HWND);

#ifndef __TIMESTAMP__
#define __TIMESTAMP__
#endif



//#define sprintf_s(buffer, buffer_size, stringbuffer, ...) (snprintf(buffer, buffer_size, stringbuffer, __VA_ARGS__))

#define sprintf_s snprintf

#define _stricmp strcasecmp
#define strcpy_s(d, l, s) strncpy( (d), (s), (l) )
#define strncpy_s(d, s, l) strncpy( (d), (s), (l) )

#define VS_PLATFORM osLinux
#define VS_CONFIGURATION x64

#define localtime_s(stc, tm )  (localtime_r( (tm) , (stc) ))

#define  sscanf_s sscanf
#define  strtok_s strtok_r
typedef struct timeval TIME_USEC;
#define  TIME_UTICKS struct timespec
#define UNUSED __attribute__((unused))

#else
#define UNUSED

typedef struct __timeb64 TIME_USEC;
#define  TIME_UTICKS LARGE_INTEGER 

#endif

#include "processlib/Compatibility.h"
#include "PCO_Structures.h"
#include "Pco_ConvStructures.h"
#include "Pco_ConvDlgExport.h"
#include "sc2_SDKStructures.h"
#include "sc2_common.h"
#include "SC2_CamExport.h"
#include "sc2_defs.h"
#include "SC2_SDKAddendum.h"
#include "PCO_errt.h"














#include <math.h>


#define DWORD_MAX ULONG_MAX 

#define ERR_SIZE	256
#define ERRMSG_SIZE	(256+128)
#define MODEL_TYPE_SIZE	32
#define INTERFACE_TYPE_SIZE	32
#define CAMERA_NAME_SIZE	128
#define MSG_SIZE	512
#define BUFF_XLAT_SIZE 128

#define ID_TIMESTAMP "$Id: [" __DATE__ " " __TIME__ "] [" __TIMESTAMP__ "] [" __FILE__ "] $"

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
			char ___buff___[ERRMSG_SIZE+1]; \
			sprintf_s(___buff___, ERRMSG_SIZE, "LIMA_HW_EXC ===> %s PcoError[x%08x][%s]", __msg__, m_pcoData->pcoError,  m_pcoData->pcoErrorMsg); \
			DEB_ALWAYS() << ___buff___; \
			throw LIMA_HW_EXC(Error, ___buff___); \
		} \
		DEB_TRACE() << "*** " <<  __msg__ << " OK" ; \
}

#define PCO_PRINT_ERR(__err__, __msg__)  \
{ \
		if(__err__){ \
			char ___buff___[ERRMSG_SIZE+1]; \
			sprintf_s(___buff___,  ERRMSG_SIZE, "=== %s PcoError[x%08x][%s]", __msg__, m_pcoData->pcoError,  m_pcoData->pcoErrorMsg); \
			printf("%s [%s][%d]\n", ___buff___, __FILE__, __LINE__); \
			DEB_TRACE() << ___buff___; \
		} \
}

#define DEF_FNID 	const char *fnId =__FUNCTION__;

#define PRINTLINES { for(int i = 0; i<50;i++) printf("=====  %s [%d]/[%d]\n", __FILE__, __LINE__,i); }


// ----- PCO FUNCTIONS
#define PCO_FN0(er,mg, fn) {mg = #fn; er = PcoCheckError(__LINE__, __FILE__, __##fn ( ), #fn ); }
#define PCO_FN1(er,mg, fn, x1) {mg = #fn; er = PcoCheckError(__LINE__, __FILE__, __##fn ( (x1) ), #fn ); }
#define PCO_FN2(er,mg, fn, x1, x2) {mg = #fn; er = PcoCheckError(__LINE__, __FILE__, __##fn ( (x1),(x2) ), #fn ) ; }
#define PCO_FN3(er,mg, fn, x1, x2, x3) {mg = #fn; er = PcoCheckError(__LINE__, __FILE__, __##fn ( (x1),(x2),(x3) ), #fn ) ; }
#define PCO_FN4(er,mg, fn, x1, x2, x3, x4) {mg = #fn; er = PcoCheckError(__LINE__, __FILE__, __##fn ( (x1),(x2),(x3),(x4) ), #fn ) ; }
#define PCO_FN5(er,mg, fn, x1, x2, x3, x4, x5) {mg = #fn; er = PcoCheckError(__LINE__, __FILE__, __##fn ( (x1),(x2),(x3),(x4),(x5) ), #fn ) ; }


// ----- PCO sdk 
#define __PCO_ResetLib() ( PCO_ResetLib()    )

#define __PCO_CloseCamera(y1) ( PCO_CloseCamera( (y1)  ) )
#define __PCO_RebootCamera(y1) ( PCO_RebootCamera( (y1)  ) )
#define __PCO_CancelImages(y1) ( PCO_CancelImages( (y1)  ) )
#define __PCO_ResetSettingsToDefault(y1) ( PCO_ResetSettingsToDefault( (y1)  ) )
#define __PCO_ArmCamera(y1) ( PCO_ArmCamera( (y1)  ) )

#define __PCO_GetPixelRate(y1, y2) ( PCO_GetPixelRate( (y1) , (y2) ) )
#define __PCO_SetPixelRate(y1, y2) ( PCO_SetPixelRate( (y1) , (y2) ) )
#define __PCO_SetADCOperation(y1, y2) ( PCO_SetADCOperation( (y1) , (y2) ) )
#define __PCO_GetADCOperation(y1, y2) ( PCO_GetADCOperation( (y1) , (y2) ) )
#define __PCO_GetHWIOSignalCount(y1, y2) ( PCO_GetHWIOSignalCount( (y1) , (y2) ) )
#define __PCO_GetRecordingState(y1, y2) ( PCO_GetRecordingState( (y1) , (y2) ) )
#define __PCO_GetPendingBuffer(y1, y2) ( PCO_GetPendingBuffer( (y1) , (y2) ) )
#define __PCO_SetRecordingState(y1, y2) ( PCO_SetRecordingState( (y1) , (y2) ) )
#define __PCO_GetRecordingState(y1, y2) ( PCO_GetRecordingState( (y1) , (y2) ) )
#define __PCO_GetCoolingSetpointTemperature(y1, y2) ( PCO_GetCoolingSetpointTemperature( (y1) , (y2) ) )
#define __PCO_GetStorageStruct(y1, y2) ( PCO_GetStorageStruct( (y1) , (y2) ) )
#define __PCO_GetRecordingStruct(y1, y2) ( PCO_GetRecordingStruct( (y1) , (y2) ) )
#define __PCO_GetTimingStruct(y1, y2) ( PCO_GetTimingStruct( (y1) , (y2) ) )
#define __PCO_GetSensorStruct(y1, y2) ( PCO_GetSensorStruct( (y1) , (y2) ) )
#define __PCO_GetSensorStruct(y1, y2) ( PCO_GetSensorStruct( (y1) , (y2) ) )
#define __PCO_GetCameraType(y1, y2) ( PCO_GetCameraType( (y1) , (y2) ) )
#define __PCO_GetCameraDescription(y1, y2) ( PCO_GetCameraDescription( (y1) , (y2) ) )
#define __PCO_GetGeneral(y1, y2) ( PCO_GetGeneral( (y1) , (y2) ) )
#define __PCO_GetImageTiming(y1, y2) ( PCO_GetImageTiming( (y1) , (y2) ) )
#define __PCO_GetRecorderSubmode(y1, y2) ( PCO_GetRecorderSubmode( (y1) , (y2) ) )
#define __PCO_SetRecorderSubmode(y1, y2) ( PCO_SetRecorderSubmode( (y1) , (y2) ) )
#define __PCO_GetStorageMode(y1, y2) ( PCO_GetStorageMode( (y1) , (y2) ) )
#define __PCO_SetStorageMode(y1, y2) ( PCO_SetStorageMode( (y1) , (y2) ) )
#define __PCO_SetAcquireMode(y1, y2) ( PCO_SetAcquireMode( (y1) , (y2) ) )
#define __PCO_SetTriggerMode(y1, y2) ( PCO_SetTriggerMode( (y1) , (y2) ) )
#define __PCO_GetActiveRamSegment(y1, y2) ( PCO_GetActiveRamSegment( (y1) , (y2) ) )
#define __PCO_SetCameraRamSegmentSize(y1, y2) ( PCO_SetCameraRamSegmentSize( (y1) , (y2) ) )
#define __PCO_GetCameraRamSegmentSize(y1, y2) ( PCO_GetCameraRamSegmentSize( (y1) , (y2) ) )
#define __PCO_OpenCamera(y1, y2) ( PCO_OpenCamera( (y1) , (y2) ) )

#define __PCO_SetHWIOSignal(y1, y2, y3) ( PCO_SetHWIOSignal( (y1) , (y2) , (y3) ) )
#define __PCO_GetHWIOSignal(y1, y2, y3) ( PCO_GetHWIOSignal( (y1) , (y2) , (y3) ) )
#define __PCO_GetHWIOSignalDescriptor(y1, y2, y3) ( PCO_GetHWIOSignalDescriptor( (y1) , (y2) , (y3) ) )
#define __PCO_SetTimeouts(y1, y2, y3) ( PCO_SetTimeouts( (y1) , (y2) , (y3) ) )
#define __PCO_GetCOCRuntime(y1, y2, y3) ( PCO_GetCOCRuntime( (y1) , (y2) , (y3) ) )
#define __PCO_GetActiveLookupTable(y1, y2, y3) ( PCO_GetActiveLookupTable( (y1) , (y2) , (y3) ) )
#define __PCO_SetActiveLookupTable(y1, y2, y3) ( PCO_SetActiveLookupTable( (y1) , (y2) , (y3) ) )
#define __PCO_SetTransferParameter(y1, y2, y3) ( PCO_SetTransferParameter( (y1) , (y2) , (y3) ) )
#define __PCO_GetTransferParameter(y1, y2, y3) ( PCO_GetTransferParameter( (y1) , (y2) , (y3) ) )
#define __PCO_CamLinkSetImageParameters(y1, y2, y3) ( PCO_CamLinkSetImageParameters( (y1) , (y2) , (y3) ) )
#define __PCO_GetBinning(y1, y2, y3) ( PCO_GetBinning( (y1) , (y2) , (y3) ) )
#define __PCO_SetBinning(y1, y2, y3) ( PCO_SetBinning( (y1) , (y2) , (y3) ) )
#define __PCO_GetCameraRamSize(y1, y2, y3) ( PCO_GetCameraRamSize( (y1) , (y2) , (y3) ) )

#define __PCO_GetCameraSetup(y1, y2, y3, y4) ( PCO_GetCameraSetup( (y1) , (y2) , (y3) , (y4) ) )
#define __PCO_SetCameraSetup(y1, y2, y3, y4) ( PCO_SetCameraSetup( (y1) , (y2) , (y3) , (y4) ) )
#define __PCO_SetMetaDataMode(y1, y2, y3, y4) ( PCO_SetMetaDataMode( (y1) , (y2) , (y3) , (y4) ) )
#define __PCO_GetTemperature(y1, y2, y3, y4) ( PCO_GetTemperature( (y1) , (y2) , (y3) , (y4) ) )
#define __PCO_GetNumberOfImagesInSegment(y1, y2, y3, y4) ( PCO_GetNumberOfImagesInSegment( (y1) , (y2) , (y3) , (y4) ) )

#define __PCO_SetDelayExposureTime(y1, y2, y3, y4, y5) ( PCO_SetDelayExposureTime( (y1) , (y2) , (y3) , (y4) , (y5) ) )
#define __PCO_GetSizes(y1, y2, y3, y4, y5) ( PCO_GetSizes( (y1) , (y2) , (y3) , (y4) , (y5) ) )
#define __PCO_GetROI(y1, y2, y3, y4, y5) ( PCO_GetROI( (y1) , (y2) , (y3) , (y4) , (y5) ) )
#define __PCO_SetROI(y1, y2, y3, y4, y5) ( PCO_SetROI( (y1) , (y2) , (y3) , (y4) , (y5) ) )

#endif
