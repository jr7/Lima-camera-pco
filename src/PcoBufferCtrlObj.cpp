#include "Exceptions.h"
#include "PcoBufferCtrlObj.h"
#include "PcoSyncCtrlObj.h"
#include "PcoCamera.h"

#undef COMPILE_WAIT_CONDITION
#undef COMPILEIT
#define COMPILE_PCO_ALLOC_BUFFER


#define THROW_LIMA_HW_EXC(e, x)  { \
	printf("========*** LIMA_HW_EXC %s\n", x ); \
			throw LIMA_HW_EXC(e, x); \
} 

using namespace lima;
using namespace lima::Pco;



BufferCtrlObj::BufferCtrlObj(Camera *cam) :
  m_handle(cam->getHandle()),
  m_cam(cam)
  //m_status(0)
{
  DEB_CONSTRUCTOR();


	SoftBufferCtrlMgr::Sync &m_bufferSync = *getBufferSync(cond);

  m_requestStop = false;

  //----------------------------------------------- initialization buffers & creating events
  for(int i=0; i < PCO_BUFFER_NREVENTS; i++) {
		m_allocBuff.pcoAllocBufferNr[i] = -1;
		m_allocBuff.pcoAllocBufferPtr[i]	= NULL;
		m_allocBuff.dwPcoAllocBufferSize[i]	= 0;

		m_allocBuff.limaAllocBufferPtr[i]	= NULL;
		m_allocBuff.dwLimaAllocBufferSize[i]	= 0;
  }
	m_allocBuff.pcoAllocBufferDone = false;
	m_allocBuff.createEventsDone = false;

}
void BufferCtrlObj::prepareAcq()
{
  DEB_MEMBER_FUNCT();

  //char *ptr;


 
  _pcoAllocBuffers();

  FrameDim dim;

  getFrameDim(dim);
  m_ImageBufferSize = dim.getMemSize();


  //int buffer_nb,concat_frame_nb;
  //m_buffer_cb_mgr.acqFrameNb2BufferNb(0,buffer_nb,concat_frame_nb);
	//ptr = (char*) m_buffer_cb_mgr.getBufferPtr(buffer_nb, concat_frame_nb);

	DEB_TRACE() << DEB_VAR1(m_ImageBufferSize);


#ifdef COMPILEIT
  
  int buffer_nb,concat_frame_nb;
  m_buffer_cb_mgr.acqFrameNb2BufferNb(0,buffer_nb,concat_frame_nb);
  //tPvFrame& frame0 = m_frame[0];
  //frame0.ImageBuffer = (char*) m_buffer_cb_mgr.getBufferPtr(buffer_nb, concat_frame_nb);

  m_buffer_cb_mgr.acqFrameNb2BufferNb(1,buffer_nb,concat_frame_nb);
  tPvFrame& frame1 = m_frame[1];
  frame1.ImageBuffer = (char*) m_buffer_cb_mgr.getBufferPtr(buffer_nb,
    concat_frame_nb);
#endif
}

void BufferCtrlObj::startAcq()
{
  DEB_MEMBER_FUNCT();
  std::string name;

	StdBufferCbMgr& buffer_mgr = m_buffer_cb_mgr;
    buffer_mgr.setStartTimestamp(Timestamp::now());

	DEB_TRACE() << "=== TRACE";
	m_requestStop = false;

	DEB_TRACE() << "=== TRACE";
	m_sync->setExposing(pcoAcqStart);

	m_cam->getCameraName(name);
	DEB_TRACE() << "=== TRACE" << name;
	m_cam->startAcq();
	DEB_TRACE() << "=== TRACE";
	


#ifdef COMPILEIT
  tPvFrame& frame = m_frame[0];
  m_status = PvCaptureQueueFrame(m_handle,&frame,_newFrame);
  
/**  int requested_nb_frames;
  m_sync->getNbFrames(requested_nb_frames);
  if(!requested_nb_frames || requested_nb_frames > 1)
    {
      tPvFrame& frame = m_frame[1];
      m_status = PvCaptureQueueFrame(m_handle,&frame,_newFrame);
    }
*/
#endif

}

#ifdef COMPILEIT
void BufferCtrlObj::_newFrame(tPvFrame* aFrame)
{
  DEB_STATIC_FUNCT();


  BufferCtrlObj *bufferPt = (BufferCtrlObj*)aFrame->Context[0];

  int requested_nb_frames;
  bufferPt->m_sync->getNbFrames(requested_nb_frames);

  bufferPt->m_exposing = false;
  if(bufferPt->m_status || aFrame->Status != ePvErrSuccess) // error
    {
      // it's not really an error,continue
      if(aFrame->Status == ePvErrDataMissing)
	{
	  DEB_WARNING() << DEB_VAR1(aFrame->Status);
          PvCaptureQueueFrame(bufferPt->m_handle,aFrame,_newFrame);
	  return;
	}
      else if(aFrame->Status == ePvErrCancelled) // we stopped the acqusition so not an error
	return;
      else 
	{
	  if(!bufferPt->m_status) // Keep error status
	    bufferPt->m_status = aFrame->Status;

	  if(aFrame->Status)
	    DEB_ERROR() << DEB_VAR1(aFrame->Status);
	    
	  return;
	}
    }
  
  ++bufferPt->m_acq_frame_nb;
  
  bool stopAcq = false;
  if(!requested_nb_frames || 
     bufferPt->m_acq_frame_nb < (requested_nb_frames - 1))
    {
      int buffer_nb, concat_frame_nb;
      bufferPt->m_buffer_cb_mgr.acqFrameNb2BufferNb(bufferPt->m_acq_frame_nb,
						    buffer_nb,
						    concat_frame_nb);
      aFrame->ImageBuffer = (char*)bufferPt->m_buffer_cb_mgr.getBufferPtr(buffer_nb,
									  concat_frame_nb);
      bufferPt->m_exposing = true;
      bufferPt->m_status = PvCaptureQueueFrame(bufferPt->m_handle,aFrame,_newFrame);
    }
  else
    stopAcq = true;
  
  HwFrameInfoType frame_info;
  frame_info.acq_frame_nb = bufferPt->m_acq_frame_nb;
  bufferPt->m_buffer_cb_mgr.newFrameReady(frame_info);
  
  if(stopAcq)
    bufferPt->m_sync->stopAcq(false);


}




#endif




//===================================================================================================================
//===================================================================================================================
    /********************************************************************************************************
      4.7.22 PCO_AddBufferExtern (Only for experienced users!)

      Adds a buffer to the driver queue. This function returns immediately. If the desired image is
      transferred to the buffer the buffer event will be fired. The user can start a thread, which can wait
      for the event of the buffer (WaitFor(Single/Multiple)Objects). This function can be used to view
      images while the recording is enabled (the user must set dw1stImage=dwLastImage=0).
      To read out previously recorded images with recording disabled, the user can call
      PCO_GetImageEx. Nevertheless you can use this function to read out single images while the
      camera is not in recording state, by setting dw1stImage=dwLastImage=x, where x is a valid image
      number (1…max available).

      a.) Prototype:
      SC2_SDK_FUNC int WINAPI PCO_AddBufferExtern(HANDLE ph, HANDLE hEvent, DWORD dw1stImage,
      DWORD dwLastImage, DWORD dwSynch, void* pBuf, DWORD dwLen, DWORD* dwStatus)

b.) Input parameter:
      · HANDLE ph: Handle to a previously opened camera device.
      · HANDLE hEvent: Handle to an externally allocated event.
      · DWORD dw1stImage: Set dw1stImage=dwLastImage=0 during record for actual image
      · DWORD dwLastImage: Set dw1stImage=dwLastImage=x after record for desired image
      · DWORD dwSynch: Synchronization paremeter, usually 0.
      · void *pBuf: Pointer to the buffer to receive the transferred image.
      · DWORD dwLen: Length of the buffer.
      · DWORD *dwStatus: Driver status.

      The input data should be filled with the following parameter:
      · hEvent = externally created event used to signal an occurred transfer.

      · dw1stImage = set to 0 for live view mode(“live view” transfers the most recent image to the
      PC for viewing / monitoring)
      - 0 = live view mode. x = set to the same value as dwLastImage. Has to be a valid
      image number (see PCO_GetNumberOfImagesInSegment, 1…max available).

      · dwLastImage = set to 0 in preview mode.
      - 0 = live view mode. x = set to the same value as dw1stImage. Has to be a valid image
      number (see PCO_GetNumberOfImagesInSegment, 1…max available).

      · dwSynch: set to 0.
      · pBuf: Address of the first buffer element to which the image should be transferred.
      · dwLen: Length of the buffer in bytes.
      · dwStatus: Address of a DWORD to receive the buffer status.

      c.) Return value:
      · int: Error message, 0 in case of success else less than 0: see Error / Warning Codes
    ********************************************************************************************************/
//===================================================================================================================
//===================================================================================================================
#define BUFFER_DUMMY_IMG_LEN	(2 * 2016 * 2016)

int BufferCtrlObj::_assignImage2Buffer(DWORD &dwFrameFirst, DWORD &dwFrameLast, DWORD dwRequestedFrames, int bufIdx) {
	

	static char _buffer[2][BUFFER_DUMMY_IMG_LEN]; 
	DEB_MEMBER_FUNCT();
    int error = 0;
    char *sErr;
	void *myBuffer;
	DWORD myBufferLen;
	int lima_buffer_nb;
	double timeout = 30;

	DEB_TRACE() << "entry -> " << DEB_VAR4(dwFrameFirst, dwFrameLast, dwRequestedFrames, bufIdx);;
	StdBufferCbMgr& buffer_mgr = m_buffer_cb_mgr;


		//int lima_buffer_nb, concat_frame_nb;		
 // buffer_mgr.acqFrameNb2BufferNb(dwFrameFirst, lima_buffer_nb, concat_frame_nb);
  // void *ptr = buffer_mgr.getFramePtr(lima_buffer_nb, concat_frame_nb);

    buffer_mgr.setStartTimestamp(Timestamp::now());
	lima_buffer_nb = dwFrameFirst-1;		


#ifdef COMPILE_WAIT_CONDITION
	Sync::Status status;
	AutoMutex lock(cond.mutex()); 
	status = m_bufferSync.wait(lima_buffer_nb, timeout);
	if(status == Sync::AVAILABLE){
		myBuffer = buffer_mgr.getFrameBufferPtr(lima_buffer_nb);
	} else {
		// (status == Sync::TIMEOUT)
		// (status == Sync::INTERRUPTED)
		return -1;
	}
#else
#pragma message ("============================================== BYPASSED ----- COMPILE_WAIT_CONDITION -----")

	myBuffer = buffer_mgr.getFrameBufferPtr(lima_buffer_nb);
 	myBufferLen = m_ImageBufferSize;
#endif

    
    DWORD dwMaxWidth, dwMaxHeight;
    WORD wArmWidth, wArmHeight, wBitPerPixel;

    unsigned int bytesPerPixel;
    
    m_cam->getArmWidthHeight(wArmWidth, wArmHeight);
    m_cam->getMaxWidthHeight(dwMaxWidth, dwMaxHeight);
    m_cam->getBytesPerPixel(bytesPerPixel);
    m_cam->getBitsPerPixel(wBitPerPixel);

    DWORD dwLen = wArmWidth * wArmHeight * bytesPerPixel;
    DWORD dwAllocatedBufferSize = dwMaxWidth * dwMaxHeight * (DWORD) bytesPerPixel;

	if(myBuffer == NULL) {
		DEB_TRACE() << "ERROR myBuffer = NULL";
		return -1;
	}

	if(myBufferLen < dwLen) {
		DEB_TRACE() << "ERROR invalid myBufferLen" << DEB_VAR2(myBufferLen, dwLen);
		return -1;
	}
	if(m_ImageBufferSize < (int) dwLen) {
		DEB_TRACE() << "ERROR invalid m_ImageBufferSize" << DEB_VAR2(m_ImageBufferSize, dwLen);
		return -1;
	}

	m_allocBuff.limaAllocBufferPtr[bufIdx] = (WORD *) myBuffer; 
	m_allocBuff.dwLimaAllocBufferSize[bufIdx] = myBufferLen; 
	
// ---------- NEW function with our assigned buffers
//SC2_SDK_FUNC int WINAPI PCO_AddBufferExtern(HANDLE ph, HANDLE hEvent, DWORD dw1stImage,
//        DWORD dwLastImage, DWORD dwSynch, void* pBuf, DWORD dwLen, DWORD* dwStatus)

      DWORD dwSynch = 0;  // must be 0
      DWORD dwStatus = 0;
      HANDLE hEvent = m_allocBuff.bufferAllocEvent[bufIdx];   // assigned in the constructor of  BufferCtrlObj

	  WORD wActSeg = m_cam->pcoGetActiveRamSegment();
		sErr = m_cam->_PcoCheckError(PCO_GetActiveRamSegment(m_handle, &wActSeg), error);
        //_PCO_TRACE("PCO_GetActiveRamSegment", sErr) ;

#ifndef COMPILE_PCO_ALLOC_BUFFER 
		
	sErr =  m_cam->_PcoCheckError(PCO_AddBufferExtern(m_handle, hEvent,wActSeg,dwFrameFirst, dwFrameLast, dwSynch, myBuffer, \
	            dwLen, &dwStatus), error);
	if(error) {
    	DEB_TRACE() << sErr;
    	DEB_TRACE() << DEB_VAR3(wActSeg,dwFrameFirst, dwFrameLast);
    	DEB_TRACE() << DEB_VAR3(dwSynch, myBuffer,dwLen);
		THROW_HW_ERROR(NotSupported) << sErr;
	}
#else
	sErr =  m_cam->_PcoCheckError(PCO_AddBufferEx(m_handle, \
				dwFrameFirst, dwFrameLast, \
				m_allocBuff.pcoAllocBufferNr[bufIdx], \
				wArmWidth, wArmHeight, wBitPerPixel), error) ;

	if(error) {
    	DEB_TRACE() << sErr;
    	DEB_TRACE() << DEB_VAR3(wActSeg,dwFrameFirst, dwFrameLast);
    	DEB_TRACE() << DEB_VAR3(wArmWidth, wArmHeight, wBitPerPixel);
		THROW_HW_ERROR(NotSupported) << sErr;
	}

    	DEB_TRACE() << DEB_VAR3(wActSeg,dwFrameFirst, dwFrameLast);
    	DEB_TRACE() << DEB_VAR3(wArmWidth, wArmHeight, wBitPerPixel);
#endif

	m_allocBuff.bufferAssignedFrameFirst[bufIdx] = dwFrameFirst;
	m_allocBuff.bufferAssignedFrameLast[bufIdx] = dwFrameLast;
	m_allocBuff.bufferReady[bufIdx] = 0;

     //----- prepartion of dwFrameFirst2assign & dwFrameLast2assign for the NEXT call to addBuffer
	dwFrameFirst = dwFrameLast + 1;
	dwFrameLast = dwFrameFirst + m_cam->pcoGetFramesPerBuffer() - 1;
	if(dwFrameLast > dwRequestedFrames) dwFrameLast = dwRequestedFrames;
	DEB_TRACE() << "exit -> " << DEB_VAR4(dwFrameFirst, dwFrameLast, dwRequestedFrames, bufIdx);;

	return 0;

}

//===================================================================================================================
//===================================================================================================================
// cloned from  Write file to disk in a separate thread

int BufferCtrlObj::_xferImag()
{
	DEB_MEMBER_FUNCT();
		

	DWORD dwFrameIdx, dwFrameIdxLast;
	DWORD dwFrameFirst2assign, dwFrameLast2assign;
	DWORD dwEvent;
	long long nr =0;
	long long bytesWritten = 0;
	int bufIdx;
	int error;
	int lima_buffer_nb;

	int maxWaitTimeout = 10;

	
// --------------- get the requested nr of images 
	int requested_nb_frames;
	DWORD dwFramesPerBuffer, dwRequestedFrames;

	m_sync->getAcqFrames(requested_nb_frames);
	dwRequestedFrames = (DWORD) requested_nb_frames;
	dwFramesPerBuffer = m_cam->pcoGetFramesPerBuffer(); // for dimax = 1


// --------------- prepare the first buffer 
  // ------- in PCO DIMAX only 1 image can be retreived
  //         (dwFramesPerBuffer = 1) ====> (dwFrameLast2assign = dwFrameFirst2assign)
	dwFrameFirst2assign = 1;
	dwFrameLast2assign = dwFrameFirst2assign + dwFramesPerBuffer - 1;
	if(dwFrameLast2assign > dwRequestedFrames) dwFrameLast2assign = dwRequestedFrames;

	bufIdx = 0;
	if(error = _assignImage2Buffer(dwFrameFirst2assign, dwFrameLast2assign, dwRequestedFrames, bufIdx)) {
		DEB_TRACE() << "ERROR _assignImage2Buffer";
		return pcoAcqPcoError;
	}

// --------------- if needed prepare the 2nd buffer 
	if(dwFrameFirst2assign <= dwRequestedFrames) {
		bufIdx = 1;
		if(error = _assignImage2Buffer(dwFrameFirst2assign, dwFrameLast2assign, dwRequestedFrames, bufIdx)) {
		DEB_TRACE() << "ERROR _assignImage2Buffer";
			return pcoAcqPcoError;
		}
	}

  // --------------- loop - process the N frames
	dwFrameIdx = 1;

	while(dwFrameIdx <= dwRequestedFrames) {

_RETRY:

		if(	m_requestStop) {
			return pcoAcqTransferStop;
		}


// --------------- look if one of buffer is READY and has the NEXT frame => proccess it
    // m_allocatedBufferAssignedFrameFirst[bufIdx] -> first frame in the buffer (we are using only 1 frame per buffer)
    // m_allocatedBufferReady[bufIdx] -> is already filled by sdk (ready)

    for(bufIdx = 0; bufIdx < PCO_BUFFER_NREVENTS; bufIdx++) {
      if((m_allocBuff.bufferAssignedFrameFirst[bufIdx] == dwFrameIdx) && m_allocBuff.bufferReady[bufIdx]) {
		  // lima frame nr is from 0 .... N-1        
		  lima_buffer_nb = dwFrameIdx -1; // this frame was already readout to the buffer

		DEB_TRACE() << "========================================FOUND " << DEB_VAR3(lima_buffer_nb, dwFrameIdx, bufIdx);

#ifdef COMPILE_PCO_ALLOC_BUFFER
		void * ptrDest = (void *)m_allocBuff.limaAllocBufferPtr[bufIdx];
		void *ptrSrc = (void *) m_allocBuff.pcoAllocBufferPtr[bufIdx];
		size_t size = m_allocBuff.dwPcoAllocBufferSize[bufIdx];

		memcpy(ptrDest, ptrSrc, size);
		
		DEB_TRACE() << "========================================FOUND " << DEB_VAR3(ptrDest, ptrSrc, size);
#endif

		HwFrameInfoType frame_info;
		frame_info.acq_frame_nb = lima_buffer_nb;
		m_buffer_cb_mgr.newFrameReady(frame_info);

		dwFrameIdxLast= m_allocBuff.bufferAssignedFrameLast[bufIdx];
        //----- the image dwFrameIdx is already in the buffer -> callback!
        if(dwFrameFirst2assign <= dwRequestedFrames) {
			if(error = _assignImage2Buffer(dwFrameFirst2assign, dwFrameLast2assign, dwRequestedFrames, bufIdx)) {
				return pcoAcqPcoError;
			}
        }
        goto _WHILE_CONTINUE;
      }
    } // for

// --------------- check if there is some buffer ready
		dwEvent = WaitForMultipleObjects( 
			PCO_BUFFER_NREVENTS,           // number of objects in array
			m_allocBuff.bufferAllocEvent,     // array of objects
			FALSE,       // wait for any object
			5000);       // ms wait

    // The return value indicates which event is signaled

#if PCO_BUFFER_NREVENTS != 2
  #pragma message ("============================================== ABORT - wrong nr of WAIT_OBJECT ")
    DUMMY_FOR_ABORT = 5;
#endif

    switch (dwEvent) { 
        case WAIT_OBJECT_0 + 0: 
			m_allocBuff.bufferReady[0] = 1; 	
			DEB_TRACE() << "========================================WAITOBJ 0  FOUND";
			goto _RETRY;
        case WAIT_OBJECT_0 + 1: 
			m_allocBuff.bufferReady[1] = 1; 
			DEB_TRACE() << "========================================WAITOBJ 1  FOUND";
			goto _RETRY;

        case WAIT_TIMEOUT: 
			maxWaitTimeout--;
			printf("Wait timed out.\n");
			if(maxWaitTimeout > 0 ) goto _RETRY;
				return pcoAcqWaitTimeout;

        default: 
          //dprintf("Wait error: %d\n", GetLastError()); 
			return pcoAcqWaitError;
    }


// --------------- write the next file received in the multi-frames file
//_WRITE_FILE:

_WHILE_CONTINUE:
    dwFrameIdx++;
  } // while(frameIdx ...

	return pcoAcqTransferEnd;

}


//===================================================================================================================
//===================================================================================================================
#define BUFFER_DUMMY_IMG_LEN	(2 * 2016 * 2016)

void BufferCtrlObj::_pcoAllocBuffers() {
	

	DEB_MEMBER_FUNCT();

	int bufIdx;
    
    


	if(!m_allocBuff.createEventsDone){
		for(bufIdx=0; bufIdx < PCO_BUFFER_NREVENTS; bufIdx++) {
		 // Create two event objects
		   m_allocBuff.bufferAllocEvent[bufIdx] = CreateEvent( 
				NULL,   // default security attributes
				FALSE,  // auto-reset event object
				FALSE,  // initial state is nonsignaled
				NULL);  // unnamed object

			if (m_allocBuff.bufferAllocEvent[bufIdx] == NULL) 
			{ 
				THROW_HW_ERROR(NotSupported) << "CreateEvent error";
			} 
		} 
		m_allocBuff.createEventsDone = true;

	}




#ifdef COMPILE_PCO_ALLOC_BUFFER 
    int error = 0;
    char *sErr;
	DWORD dwMaxWidth, dwMaxHeight;
    WORD wBitPerPixel;
    unsigned int bytesPerPixel;

	if(!m_allocBuff.pcoAllocBufferDone){
		m_cam->getMaxWidthHeight(dwMaxWidth, dwMaxHeight);
		m_cam->getBytesPerPixel(bytesPerPixel);
		m_cam->getBitsPerPixel(wBitPerPixel);

		DWORD dwAllocatedBufferSize = dwMaxWidth * dwMaxHeight * (DWORD) bytesPerPixel;


			//-------------- allocate 2 buffers (0,1) and received the handle, mem ptr, events
			for(bufIdx = 0; bufIdx <2 ; bufIdx ++) {
				sErr = m_cam->_PcoCheckError(PCO_AllocateBuffer(m_handle, \
					&m_allocBuff.pcoAllocBufferNr[bufIdx], \
					dwAllocatedBufferSize, \
					&m_allocBuff.pcoAllocBufferPtr[bufIdx], \
					&m_allocBuff.bufferAllocEvent[bufIdx]\
					), error);

				if(error) {
    				DEB_TRACE() << sErr;
    				DEB_TRACE() << DEB_VAR1(dwAllocatedBufferSize);
					THROW_HW_ERROR(NotSupported) << sErr;
				}
				m_allocBuff.dwPcoAllocBufferSize[bufIdx] = dwAllocatedBufferSize;

			}
		m_allocBuff.pcoAllocBufferDone = true;
	}
#endif

}

void BufferCtrlObj::_pcoAllocBuffersFree() {
	

	DEB_MEMBER_FUNCT();

#ifdef COMPILE_PCO_ALLOC_BUFFER 
    int error = 0;
    char *sErr;

	if(m_allocBuff.pcoAllocBufferDone){
	
		//SC2_SDK_FUNC int WINAPI PCO_FreeBuffer(HANDLE ph, SHORT sBufNr)


			//-------------- allocate 2 buffers (0,1) and received the handle, mem ptr, events
			for(int bufIdx = 0; bufIdx <2 ; bufIdx ++) {
				sErr = m_cam->_PcoCheckError(PCO_FreeBuffer(m_handle, m_allocBuff.pcoAllocBufferNr[bufIdx]), error);

				if(error) {
    				DEB_TRACE() << sErr;
					THROW_HW_ERROR(NotSupported) << sErr;
				}
				m_allocBuff.dwPcoAllocBufferSize[bufIdx] = 0;
				m_allocBuff.pcoAllocBufferPtr[bufIdx] = NULL;

			}
		m_allocBuff.pcoAllocBufferDone = false;
	}
#endif

}
