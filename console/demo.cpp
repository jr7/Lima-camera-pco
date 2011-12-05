#undef COMPILE_EMPTY
//#define COMPILE_EMPTY

//#define DEBUG_H

#define PCO_ERRT_H_CREATE_OBJECT

#define DATA_NR_FRAMES 50
#define DATA_EXP_TIME 0.000002
#define EBUFS 400
#define INFO_SIZE 5000


#undef __BYPASSINLINE__

#include "Debug.h"


#ifndef COMPILE_EMPTY
#include "Constants.h"
#include "CtControl.h"
#include "CtAcquisition.h"
#include "CtSaving.h"
#include "HwInterface.h"
#include "PcoCamera.h"
#include "PcoInterface.h"
#include "PcoSyncCtrlObj.h"
#include "PCO_errt.h"
#include "PoolThreadMgr.h"
#endif



#ifndef COMPILE_EMPTY
#include "load.h"
#endif

#pragma pack (4)
#pragma pack ()


int getlib(void);
//===============================================================================================================
//===============================================================================================================

	//; FALSE// Set to TRUE, if you want to allocate the buffer on your own.
	/************************************************************
	load SC2 library; please note SC2_Cam.dll has dependency on 
	sc2_1394.dll, this dll must be present in your dll search path.
	************************************************************/

/************
enum DebFormat {
	DebFmtDateTime		= 1 << 0,  1
	DebFmtThread		= 1 << 1,  2
	DebFmtModule		= 1 << 2,  4
	DebFmtObj			= 1 << 3,  8   
	DebFmtFunct			= 1 << 4,  10	
	DebFmtFileLine		= 1 << 5,  20
	DebFmtType			= 1 << 6,  40 
	DebFmtIndent		= 1 << 7,  80
	DebFmtColor			= 1 << 8,  100
};
***********/
#define DEBUG_FORMAT 0x30
#define DEBUG_TYPE 0xffffffff
#define DEBUG_MODULE 0xffffffff


/*********
#cam=Basler.Camera("192.168.180.2",8000)
#i=Basler.Interface(cam)
#c=Core.CtControl(i)
#s=c.saving()
#par=s.Parameters()
#par.directory='/tmp/write_test/t0'
#par.prefix='seb_test_'
#par.suffix='.edf'
#par.fileFormat=1
#par.savingMode=1
#s.setParameters(par)

#a = c.acquisition()
#a.setAcqExpoTime(5)
#a.setAcqNbFrames(100)

#Core.DebParams.setTypeFlags(0xff)
#Core.DebParams.setModuleFlags(0xffff)
***********/

//===============================================================================================================
//===============================================================================================================
using namespace lima;

#if 0
	DebParams::Flags lima::DebParams::s_type_flags;
DebParams::Flags lima::DebParams::s_fmt_flags;
DebParams::Flags lima::DebParams::s_mod_flags;
Mutex LIMACORE_API  *DebParams::s_mutex = NULL;
LIMACORE_API DebStream *DebParams::s_deb_stream = NULL;
#endif

DEB_GLOBAL_NAMESPC(DebModNone,"main")	;

int myMain(int argc, char* argv[])
{
#ifndef COMPILE_EMPTY
	char *fnId; fnId = "main";


	//DEB_GLOBAL_FUNCT();
	DebObj deb(getDebParams(), false, __FUNCTION__,
		   NULL, __FILE__, __LINE__);

	//char info[INFO_SIZE+1];
	char *msg;
	char errbuffer[EBUFS];

	bool bbufferextern = TRUE;

	CtControl::Status status;

	static int err;

	
	printf("\n\n[%s %d] ======================= %s\r\n", __FILE__, __LINE__, "pco.camera demo program");
	err = getlib();
	if (err != 0)
	{
		sprintf_s(errbuffer, EBUFS, "\nCan not load SC2 library. Error: %d\n", err);
		printf(errbuffer);
		printf("Refer to PCO_err.h for explanation of some error codes\n");

		exit(-1);
		return -1;
	}
	printf("\n\n[%s %d] ======================= %s\r\n", __FILE__, __LINE__, "Libraries loaded");
  

	//DebParams::setModuleFlags(DEBUG_MODULE);
	//DebParams::setTypeFlags(DEBUG_TYPE);
	//DebParams::setFormatFlags(DEBUG_FORMAT);

	msg = "new cam" ; printf("\n\n%s> ======================= [%s %d]\r\n", msg, __FILE__, __LINE__);
	Pco::Camera *myCam = new Pco::Camera() ;

	msg = "new if" ; printf("\n\n%s> ======================= [%s %d]\r\n", msg, __FILE__, __LINE__);
	Pco::Interface *myIf = new Pco::Interface(myCam) ;

	msg = "new control[begin]" ; printf("\n\n%s> ======================= [%s %d]\r\n", msg, __FILE__, __LINE__);
	CtControl *myCt = new CtControl(myIf) ;
	msg = "new control [end]" ; printf("%s\n\n> ======================= [%s %d]\r\n", msg, __FILE__, __LINE__);

	CtAcquisition* 	myCtAcq = myCt->acquisition();


	CtSaving::Parameters myPar;
	CtSaving *mySav = myCt->saving();

	myPar.directory="d:\\huge" ;
	myPar.prefix="test_";
	myPar.suffix=".edf";
	myPar.fileFormat= CtSaving::EDF;
	myPar.savingMode= CtSaving::AutoFrame;
	myPar.overwritePolicy= CtSaving::Overwrite;
	mySav->setParameters(myPar);

	struct stcPcoData * m_pcoData = myCam->_getPcoData();

	WORD activeRamSegment = myCam->pcoGetActiveRamSegment();

	msg = "myCam->_getFramesMax(activeRamSegment)" ; printf("\n\n%s> ======================= [%s %d]\r\n", msg, __FILE__, __LINE__);
	unsigned long  framesMax = myCam->pcoGetFramesMax(activeRamSegment);

	printf("=== %s> activeRamSegment[%d] framesMax[%ld]\n", fnId, activeRamSegment, framesMax);
	printf("=== %s> cocRunTime[%g] framesRate[%g]\n", fnId, myIf->getCocRunTime(), myIf->getFrameRate() );


	int nb_frames1, nb_frames2, nb_acq_frames; 	
	myCtAcq->getAcqNbFrames(nb_frames1);
	myCtAcq->setAcqNbFrames(DATA_NR_FRAMES);
	myCtAcq->getAcqNbFrames(nb_frames2);
	printf("=== %s> requestedFrames[%d] [%d]\n", fnId, nb_frames1,nb_frames2 );

	double acq_time1, acq_time2;
	myCtAcq->getAcqExpoTime(acq_time1);
	myCtAcq->setAcqExpoTime(DATA_EXP_TIME);
	myCtAcq->getAcqExpoTime(acq_time2);
	printf("=== %s> expTime[%g] [%g]\n", fnId, acq_time1,acq_time2 );

	msg = "myCt->prepareAcq()" ; printf("\n\n%s> ======================= [%s %d]\r\n", msg, __FILE__, __LINE__);
	myCt->prepareAcq();

	msg = "startAcq" ; printf("\n\n%s> ======================= [%s %d]\r\n", msg, __FILE__, __LINE__);
	myCt->startAcq();

	//myCt->getStatus(status);
	printf("\n\n======================================================================================= %s> SLEEP begin\n", fnId );

	while(1) {
		myCt->getStatus(status);
		if(status.AcquisitionStatus == AcqReady) {
			printf("\n\n============================================== %s> [%d] READY\n\n", fnId, status.AcquisitionStatus );
			break;
		}
		if(status.AcquisitionStatus == AcqFault) {
			printf("\n\n============================================== %s> [%d] ERROR\n\n", fnId,  status.AcquisitionStatus);
			break;
		}
	printf("\n\n [%d]======================================================================================= %s> SLEEP begin\n", status.AcquisitionStatus, fnId );
		Sleep(0.1);
	}

	myCtAcq->getAcqNbFrames(nb_acq_frames);
	printf("=== %s> ACQFrames[%d]\n", fnId, nb_acq_frames );

	msg = "getInfo" ; printf("\n\n%s> ======================= [%s %d]\r\n", msg, __FILE__, __LINE__);
	//printf("%s\n", myCam->getInfo(info, INFO_SIZE));
	printf("%s\n", myCam->getInfo());

	msg = "delete cam" ; printf("\n\n%s> ======================= [%s %d]\r\n", msg, __FILE__, __LINE__);
	delete myCam;
	msg = "after delete cam" ; printf("\n\n%s> ======================= [%s %d]\r\n", msg, __FILE__, __LINE__);

	PoolThreadMgr::get().quit();
	msg = "after PoolThreadMgr::get().quit()" ; printf("\n\n%s> ======================= [%s %d]\r\n", msg, __FILE__, __LINE__);

#endif
	return 0;
}

//} // lima
//===============================================================================================================
//===============================================================================================================

int getlib(void)
{
#ifndef COMPILE_EMPTY
	HINSTANCE SC2Lib = NULL;
	HINSTANCE LibLimaCore = NULL;
	HINSTANCE libprocesslib = NULL;
	HINSTANCE liblimapco = NULL;

	DWORD liberror;
	SC2Lib = LoadLibrary("SC2_Cam");
	if (SC2Lib == NULL) {liberror = GetLastError(); return liberror; }

	LibLimaCore = LoadLibrary("LibLimaCore");
	if (LibLimaCore == NULL) {liberror = GetLastError(); return liberror; }

	libprocesslib = LoadLibrary("libprocesslib");
	if (libprocesslib == NULL) {liberror = GetLastError(); return liberror; }

	liblimapco = LoadLibrary("liblimapco");
	if (liblimapco == NULL) {liberror = GetLastError(); return liberror; }
#endif
	return 0;
}

int main(int argc, char* argv[]) { 
	
#ifndef COMPILE_EMPTY
  return myMain(argc, argv);
#else
	return 0 ;
#endif

}
