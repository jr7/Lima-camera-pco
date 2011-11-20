#include "Exceptions.h"
#include "PcoBufferCtrlObj.h"
#include "PcoSyncCtrlObj.h"
#include "PcoCamera.h"



#define THROW_LIMA_HW_EXC(e, x)  { \
	printf("========*** LIMA_HW_EXC %s\n", x ); \
			throw LIMA_HW_EXC(e, x); \
} 

using namespace lima;
using namespace lima::Pco;



BufferCtrlObj::BufferCtrlObj(Camera *cam) :
  m_handle(cam->getHandle()),
  m_cam(cam),
  m_status(0)
{
  DEB_CONSTRUCTOR();

  

  //----------------------------------------------- initialization buffers & creating events
  for(int i=0; i < PCO_BUFFER_NREVENTS; i++) {
		m_allocBuff.bufferNr[i] = -1;
		m_allocBuff.bufferPtr[i]	= NULL;

	 // Create two event objects
       m_allocBuff.bufferEvent[i] = CreateEvent( 
            NULL,   // default security attributes
            FALSE,  // auto-reset event object
            FALSE,  // initial state is nonsignaled
            NULL);  // unnamed object

        if (m_allocBuff.bufferEvent[i] == NULL) 
        { 
            THROW_LIMA_HW_EXC(Error, "CreateEvent error")
        } 
    } 
}
void BufferCtrlObj::prepareAcq()
{
  DEB_MEMBER_FUNCT();

  m_acq_frame_nb = -1; // idx of the acquired frame


#ifdef COMPILEIT
  FrameDim dim;
  getFrameDim(dim);
  m_frame[0].ImageBufferSize = m_frame[1].ImageBufferSize = dim.getMemSize();
  
  m_acq_frame_nb = -1;
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

	StdBufferCbMgr& buffer_mgr = m_buffer_cb_mgr;
    buffer_mgr.setStartTimestamp(Timestamp::now());

	m_sync->setExposing(false);

	m_cam->startAcq();
	


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
void BufferCtrlObj::_assignImage2Buffer(DWORD &dwFrameFirst, DWORD &dwFrameLast, DWORD dwRequestedFrames, int bufIdx) {
    DEB_MEMBER_FUNCT();
    int error = 0;
    char *sErr;

		StdBufferCbMgr& buffer_mgr = m_buffer_cb_mgr;
		int buffer_nb, concat_frame_nb;		

    buffer_mgr.setStartTimestamp(Timestamp::now());

    buffer_mgr.acqFrameNb2BufferNb(dwFrameFirst, buffer_nb, concat_frame_nb);
		void *ptr = buffer_mgr.getBufferPtr(buffer_nb, concat_frame_nb);
  // void *ptr = buffer_mgr.getFramePtr(buffer_nb, concat_frame_nb);

    m_allocBuff.bufferNr[bufIdx] = bufIdx;
    m_allocBuff.bufferPtr[bufIdx] = (WORD *) ptr;
    
    DWORD dwMaxWidth, dwMaxHeight;
    WORD wArmWidth, wArmHeight;
    WORD wBitPerPixel;

    unsigned int bytesPerPixel;
    
    m_cam->getArmWidthHeight(wArmWidth, wArmHeight);
    m_cam->getMaxWidthHeight(dwMaxWidth, dwMaxHeight);
    m_cam->getBytesPerPixel(bytesPerPixel);
    m_cam->getBitsPerPixel(wBitPerPixel);

    DWORD dwAllocatedBufferSize = dwMaxWidth * dwMaxHeight * (DWORD) bytesPerPixel;


/**** NOT USED 
    _PcoCheckError(PCO_AllocateBuffer(m_handle, &(m_allocatedBufferNr[bufIdx]), dwAllocatedBufferSize, 

    PCO_AddBufferEx frames[%ld]-[%ld] bufIdx[%d] x[%d] y[%d] bits[%d]", dwFrameFirst, frameLast, bufIdx, ds->ccd.size.xarm, ds->ccd.size.yarm, ds->ccd.size.bits
        sErr = _PcoCheckError(PCO_AddBufferEx(m_handle, dwFrameFirst, dwFrameLast, m_allocatedBufferNr[bufIdx], \
						wArmWidth, wArmHeight, wBitPerPixel));
****/
      // ---------- NEW function with our assigned buffers
      //SC2_SDK_FUNC int WINAPI PCO_AddBufferExtern(HANDLE ph, HANDLE hEvent, DWORD dw1stImage,
      //        DWORD dwLastImage, DWORD dwSynch, void* pBuf, DWORD dwLen, DWORD* dwStatus)

      DWORD dwSynch = 0;  // must be 0
      DWORD dwStatus = 0;
      DWORD dwLen = 1;    // --------- TODO
      void *myBuffer = 0;  //-------------------------- buffer received from lima TODO
      HANDLE hEvent = m_allocBuff.bufferEvent[bufIdx];   // assigned in the constructor of  BufferCtrlObj

      WORD wActSeg = 0;
      sErr = _PcoCheckError(PCO_GetActiveRamSegment(m_handle, &wActSeg));
        _PCO_TRACE("PCO_GetActiveRamSegment", sErr) ;

    sErr = _PcoCheckError(PCO_AddBufferExtern(m_handle, hEvent,wActSeg,dwFrameFirst, dwFrameLast, dwSynch, myBuffer, \
	            dwLen, &dwStatus));
        _PCO_TRACE("PCO_AddBufferExtern", sErr) ;


	m_allocBuff.bufferAssignedFrameFirst[bufIdx] = dwFrameFirst;
	m_allocBuff.bufferAssignedFrameLast[bufIdx] = dwFrameLast;
	m_allocBuff.bufferReady[bufIdx] = 0;

     //----- prepartion of dwFrameFirst2assign & dwFrameLast2assign for the NEXT call to addBuffer
  dwFrameFirst = dwFrameLast + 1;
	dwFrameLast = dwFrameFirst + m_cam->pcoGetFramesPerBuffer() - 1;
	if(dwFrameLast > dwRequestedFrames) dwFrameLast = dwRequestedFrames;
}

//===================================================================================================================
//===================================================================================================================
// cloned from   Write file to disk in a separate thread

int BufferCtrlObj::_xferImag()
{
	DEB_MEMBER_FUNCT();

	DWORD dwFrameIdx, dwFrameIdxLast;
	DWORD dwFrameFirst2assign, dwFrameLast2assign;
	DWORD dwEvent;
	long long nr =0;
	long long bytesWritten = 0;
	int bufIdx;

	int maxWaitTimeout = 10;

	
// --------------- get the requested nr of images 
	int requested_nb_frames;
	DWORD dwFramesPerBuffer, dwRequestedFrames;

	m_sync->getAcqFrames(requested_nb_frames);
	dwRequestedFrames = (DWORD) requested_nb_frames;
	dwFramesPerBuffer = m_cam->pcoGetFramesPerBuffer();


// --------------- prepare the first buffer 
  // ------- in PCO DIMAX only 1 image can be retreived
  //         (dwFramesPerBuffer = 1) ====> (dwFrameLast2assign = dwFrameFirst2assign)
	dwFrameFirst2assign = 1;
	dwFrameLast2assign = dwFrameFirst2assign + dwFramesPerBuffer - 1;
	if(dwFrameLast2assign > dwRequestedFrames) dwFrameLast2assign = dwRequestedFrames;

	bufIdx = 0;
	_assignImage2Buffer(dwFrameFirst2assign, dwFrameLast2assign, dwRequestedFrames, bufIdx);

// --------------- if needed prepare the 2nd buffer 
	if(dwFrameFirst2assign <= dwRequestedFrames) {
		bufIdx = 1;
		_assignImage2Buffer(dwFrameFirst2assign, dwFrameLast2assign, dwRequestedFrames, bufIdx);
	}

  // --------------- loop - process the N frames
	dwFrameIdx = 1;
	while(dwFrameIdx <= dwRequestedFrames) {

_RETRY:

// --------------- look if one of buffer is READY and has the NEXT frame => proccess it
    // m_allocatedBufferAssignedFrameFirst[bufIdx] -> first frame in the buffer (we are using only 1 frame per buffer)
    // m_allocatedBufferReady[bufIdx] -> is already filled by sdk (ready)

    for(bufIdx = 0; bufIdx < PCO_BUFFER_NREVENTS; bufIdx++) {
      if((m_allocBuff.bufferAssignedFrameFirst[bufIdx] == dwFrameIdx) && m_allocBuff.bufferReady[bufIdx]) {
        m_acq_frame_nb = dwFrameIdx; // this frame was already readout to the buffer

		  HwFrameInfoType frame_info;
		  frame_info.acq_frame_nb = m_acq_frame_nb;
		  m_buffer_cb_mgr.newFrameReady(frame_info);



		dwFrameIdxLast= m_allocBuff.bufferAssignedFrameLast[bufIdx];
        //----- the image dwFrameIdx is already in the buffer -> callback!
        if(dwFrameFirst2assign <= dwRequestedFrames) {
          _assignImage2Buffer(dwFrameFirst2assign, dwFrameLast2assign, dwRequestedFrames, bufIdx);
        }
        goto _WHILE_CONTINUE;
      }
    } // for

// --------------- check if there is some buffer ready
		dwEvent = WaitForMultipleObjects( 
			PCO_BUFFER_NREVENTS,           // number of objects in array
			m_allocBuff.bufferEvent,     // array of objects
			FALSE,       // wait for any object
			5000);       // ms wait

    // The return value indicates which event is signaled

#if PCO_BUFFER_NREVENTS != 2
  #pragma message ("============================================== ABORT - wrong nr of WAIT_OBJECT ")
    DUMMY_FOR_ABORT = 5;
#endif

    switch (dwEvent) { 
        case WAIT_OBJECT_0 + 0: m_allocBuff.bufferReady[0] = 1; 	goto _RETRY;
        case WAIT_OBJECT_0 + 1: m_allocBuff.bufferReady[1] = 1; goto _RETRY;

        case WAIT_TIMEOUT: 
			maxWaitTimeout--;
			printf("Wait timed out.\n");
			if(maxWaitTimeout > 0 ) goto _RETRY;
			return 1;

        default: 
          //dprintf("Wait error: %d\n", GetLastError()); 
			return 1;
    }


// --------------- write the next file received in the multi-frames file
//_WRITE_FILE:

_WHILE_CONTINUE:
    ;
  } // while(frameIdx ...

	return 0;

}

