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
#define MODEL_SIZE	32
#define CAMERA_SIZE	128
#define MSG_SIZE	512


typedef DWORD tPvUint32;
typedef int tPvErr;

#endif
