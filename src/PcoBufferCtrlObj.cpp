#include "Exceptions.h"
#include "PcoBufferCtrlObj.h"
#include "PcoSyncCtrlObj.h"
#include "PcoCamera.h"

#define BUFFER_NR_EVENTS 2

#define THROW_LIMA_HW_EXC(e, x)  { \
	printf("========*** LIMA_HW_EXC %s\n", x ); \
			throw LIMA_HW_EXC(e, x); \
} 

using namespace lima;
using namespace lima::Pco;



BufferCtrlObj::BufferCtrlObj(Camera *cam) :
  m_handle(cam->getHandle()),
  //m_status(ePvErrSuccess),
  m_exposing(false)
{
  DEB_CONSTRUCTOR();

  //m_frame[0].Context[0] = this;
  //m_frame[1].Context[0] = this;

  m_frames_per_buffer = 1;  // PCO DSK allows only one frame/buffer

  //----------------------------------------------- initialization buffers & creating events
  for(int i=0; i < BUFFER_NR_EVENTS; i++) {
		m_allocatedBufferNr[i] = -1;
		m_allocatedBufferPtr[i]	= NULL;

	 // Create two event objects
        m_allocatedBufferEvent[i] = CreateEvent( 
            NULL,   // default security attributes
            FALSE,  // auto-reset event object
            FALSE,  // initial state is nonsignaled
            NULL);  // unnamed object

        if (m_allocatedBufferEvent[i] == NULL) 
        { 
            THROW_LIMA_HW_EXC(Error, "CreateEvent error")
        } 
    } 


}
void BufferCtrlObj::prepareAcq()
{
  DEB_MEMBER_FUNCT();
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



#ifdef COMPILEIT
  m_exposing = true;
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

void BufferCtrlObj::_newFrame(tPvFrame* aFrame)
{
  DEB_STATIC_FUNCT();

#ifdef COMPILEIT

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

#endif

}



//===================================================================================================================
//===================================================================================================================
void BufferCtrlObj::assignImage2Buffer(DWORD &dwFrameFirst, DWORD &dwFrameLast, DWORD dwRequestedFrames, int bufIdx) {
    DEB_MEMBER_FUNCT();
    int error;
    char *sErr;

		StdBufferCbMgr& buffer_mgr = m_buffer_cb_mgr;
		int buffer_nb, concat_frame_nb;		

    buffer_mgr.setStartTimestamp(Timestamp::now());

    buffer_mgr.acqFrameNb2BufferNb(dwFrameFirst, buffer_nb, concat_frame_nb);
		void *ptr = buffer_mgr.getBufferPtr(buffer_nb,   concat_frame_nb);

    m_allocatedBufferNr[bufIdx] = bufIdx;
    m_allocatedBufferPtr[bufIdx] = (WORD *) ptr;
    
    /************************************************************************************************
		Allocates a buffer to receive the transferred images. There is a maximum of 8 buffers. This
		function is needed to create, or to attach buffers for the image transfer. The buffers are attached to
		the previously opened camera. Using two buffers in an alternating manner is sufficient for most
		applications. If you use more than one camera, you will get the same buffer numbers 0 and 1 for
		each camera while allocating e.g. two buffers.

		SC2_SDK_FUNC int WINAPI PCO_AllocateBuffer(HANDLE ph, SHORT* sBufNr, DWORD dwSize,
		WORD** wBuf, HANDLE* hEvent)
		· HANDLE ph: Handle to a previously opened camera device.
		· SHORT* sBufNr: Address of a SHORT pointer to get the current number of the buffer.
		· DWORD dwSize: DWORD to set the buffer size.
		· WORD** wBuf: Address of a WORD* to get the buffer pointer.
		· HANDLE* hEvent: Address of a HANDLE to get the event which will be fired in case of 
		a previously arrived image.

		The input data should be filled with the following parameters:
		· *sBufNr = -1 to allocate a new buffer, 0 … 7, to change a previously allocated buffer.
		· dwSize = size of the buffer in byte (normally: Xres * Yres * 2).
		· **wBuf = must be the address of a WORD*.
		·*hEvent = 0 to create a ne

		m_bufferNrM -> m_allocatedBufferNr
		m_bufferM -> m_allocatedBufferPtr
		m_bufferM_events -> m_allocatedBufferEvent
		m_bufsize -> m_allocatedBufferSize
		m_bufsize_max -> m_allocatedBufferSizeMax
		***************************************************************************************************/
    DWORD dwMaxWidth;
    DWORD dwMaxHeight;
    WORD wArmWidth;
    WORD wArmHeight;
    WORD wBitPerPixel;
    
    unsigned int pixbytes;
    
    m_cam->getArmWidthHeight(wArmWidth, wArmHeight);
    m_cam->getMaxWidthHeight(dwMaxWidth, dwMaxHeight);
    m_cam->getBytesPerPixel(pixbytes);
    m_cam->getBitsPerPixel(wBitPerPixel);

    DWORD dwAllocatedBufferSize = dwMaxWidth * dwMaxHeight * (DWORD) pixbytes;

			sErr = _PcoCheckError(PCO_AllocateBuffer(m_handle, &(m_allocatedBufferNr[bufIdx]), dwAllocatedBufferSize, 
										&(m_allocatedBufferPtr[bufIdx]), &(m_allocatedBufferEvent[bufIdx])));
            _PCO_TRACE("PCO_AllocateBuffer", sErr) ;

    /********************************************************************************************************
    Adds a buffer to the driver queue. This function returns immediately. If the desired image is
    transferred to the buffer the buffer event will be fired. The user can start a thread, which can wait
    for the event of the buffer (WaitFor(Single/Multiple)Objects). This function can be used to view
    images while the recording is enabled (the user must set dw1stImage=dwLastImage=0).

    SC2_SDK_FUNC int WINAPI PCO_AddBuffer(HANDLE ph, DWORD dw1stImage, DWORD dwLastImage,
    SHORT sBufNr, WORD wXRes, WORD wYRes, WORD wBitPerPixel)
    · HANDLE ph: Handle to a previously opened camera device.
    · DWORD dw1stImage: Set dw1stImage=dwLastImage=0 during record for actual image
    · DWORD dwLastImage: Set dw1stImage=dwLastImage=x after record for desired image
    · SHORT sBufNr: SHORT to set the buffer number to fill with.
    · WORD wXRes: x-Resolution.
    · WORD wYRes: y-Resolution.
    · WORD wBitPerPixel: BitResolution of one Pixel (e.g. 14bit).

    The input data should be filled with the following parameter:
    · dw1stImage = set to 0 for live view mode(?live view? transfers the most recent image to the
    PC for viewing / monitoring)
    - 0 = live view mode. x = set to the same value as dwLastImage. Has to be a valid image
    number (see PCO_GetNumberOfImagesInSegment, 1?max available).
    · dwLastImage = set to 0 in preview mode.
    - 0 = live view mode. x = set to the same value as dw1stImage. Has to be a valid image
    number (see PCO_GetNumberOfImagesInSegment, 1?max available).
    · sBufNr: 0 ? 7:.number of desired buffer. A buffer can be reused after the event is fired.
    · wXRes: Actual x-Resolution of the image which should be transferred.
    · wYRes: Actual y-Resolution of the image which should be transferred.
    · WBitPerPixel: BitResolution of the image which should be transferred.
    ********************************************************************************************************/

  //... PCO_AddBufferEx frames[%ld]-[%ld] bufIdx[%d] x[%d] y[%d] bits[%d]", dwFrameFirst, frameLast, bufIdx, ds->ccd.size.xarm, ds->ccd.size.yarm, ds->ccd.size.bits
        sErr = _PcoCheckError(PCO_AddBufferEx(m_handle, dwFrameFirst, dwFrameLast, m_allocatedBufferNr[bufIdx], \
						wArmWidth, wArmHeight, wBitPerPixel));
        _PCO_TRACE("PCO_AddBufferEx", sErr) ;

	m_allocatedBufferAssignedFrameFirst[bufIdx] = dwFrameFirst;
	m_allocatedBufferAssignedFrameLast[bufIdx] = dwFrameLast;
	m_allocatedBufferReady[bufIdx] = 0;

     //----- prepartion of dwFrameFirst2assign & dwFrameLast2assign for the NEXT call to addBuffer
  dwFrameFirst = dwFrameLast + 1;
	dwFrameLast = dwFrameFirst + ((DWORD) m_frames_per_buffer) - 1;
	if(dwFrameLast > dwRequestedFrames) dwFrameLast = dwRequestedFrames;

}


/*---------------------------------------------------------------------------
        Purpose:  Write file to disk in a separate thread
		Arguments:	argin = ccd ds struct
        Returns:    nothing
*---------------------------------------------------------------------------*/
void BufferCtrlObj::xferImag()
{
    DEB_MEMBER_FUNCT();

	DWORD dwFrameIdx, dwFrameIdxLast;
  DWORD dwFrameFirst2assign, dwFrameLast2assign;
	DWORD dwEvent;
	long error;
  long long nr =0;
	long long bytesWritten = 0;
	int bufIdx;
	char *data;
	unsigned int bufptr;
	
// --------------- get the requested nr of images 
	int requested_nb_frames;
	DWORD dwFramesPerBuffer, dwRequestedFrames;

  m_sync->getNbFrames(requested_nb_frames);
  dwRequestedFrames = (DWORD) requested_nb_frames;
	dwFramesPerBuffer = m_frames_per_buffer;

// --------------- prepare the first buffer 
  // ------- in PCO DIMAX only 1 image can be retreived
  //         (dwFramesPerBuffer = 1) ====> (dwFrameLast2assign = dwFrameFirst2assign)
	dwFrameFirst2assign = 1;
	dwFrameLast2assign = dwFrameFirst2assign + dwFramesPerBuffer - 1;
	if(dwFrameLast2assign > dwRequestedFrames) dwFrameLast2assign = dwRequestedFrames;

	bufIdx = 0;
	 
  assignImage2Buffer(dwFrameFirst2assign, dwFrameLast2assign, dwRequestedFrames, bufIdx);

/******************
  error = PcoCheckError(PCO_AddBufferEx(m_handle, dwFrameFirst2assign, dwFrameLast2assign, m_allocatedBufferNr[bufIdx], \
						m_size.armwidth, m_size.armheight, m_size.pixbits));
  PCO_TRACE("PCO_AddBufferEx") ;

 	m_allocatedBufferAssignedFrameFirst[bufIdx] = dwFrameFirst2assign;
	m_allocatedBufferAssignedFrameLast[bufIdx] =dwFrameLast2assign;
	m_allocatedBufferReady[bufIdx] = 0;
 
      //----- prepartion of dwFrameFirst2assign & dwFrameLast2assign for the NEXT call to addBuffer
  dwFrameFirst2assign = dwFrameLast2assign + 1;
	dwFrameLast2assign = dwFrameFirst2assign + dwFramesPerBuffer - 1;
	if(dwFrameLast2assign > dwRequestedFrames) dwFrameLast2assign = dwRequestedFrames;
************/

	bufIdx = 1;

// --------------- if needed prepare the 2nd buffer 
	if(dwFrameFirst2assign <= dwRequestedFrames) {
    assignImage2Buffer(dwFrameFirst2assign, dwFrameLast2assign, dwRequestedFrames, bufIdx);
	}


  // --------------- loop - process the N frames
	dwFrameIdx = 1;
	while(dwFrameIdx <= dwRequestedFrames) {
		//dprintf3("... loop frame [%d] of [%d]", frameIdx, nrFrames);

_RETRY:

		//if(ds->ccd.filesave.saving == ACQ_SAVING_ABORTED){dprintf("<%s> Stop requested\n", fnId);	break;  // exit from while loop	}

// --------------- look if one buffer has already the next frame & write it 
		for(bufIdx = 0; bufIdx < BUFFER_NR_EVENTS; bufIdx++) {
			if((m_allocatedBufferAssignedFrameFirst[bufIdx] == dwFrameIdx) && m_allocatedBufferReady[bufIdx]) {
				dwFrameIdxLast= m_allocatedBufferAssignedFrameLast[bufIdx];
				goto _WRITE_FILE;
			}
		}

// --------------- check if there is some buffer ready
		dwEvent = WaitForMultipleObjects( 
			BUFFER_NR_EVENTS,           // number of objects in array
			m_allocatedBufferEvent,     // array of objects
			FALSE,       // wait for any object
			5000);       // ms wait

    // The return value indicates which event is signaled

#if BUFFER_NR_EVENTS != 2
  #pragma message ("============================================== ABORT - wrong nr of WAIT_OBJECT ")
    DUMMY_FOR_ABORT = 5;
#endif

    switch (dwEvent) { 
        case WAIT_OBJECT_0 + 0: m_allocatedBufferReady[0] = 1; 	goto _RETRY;
        case WAIT_OBJECT_0 + 1: m_allocatedBufferReady[1] = 1; goto _RETRY;

        case WAIT_TIMEOUT: 
			printf("Wait timed out.\n");
			goto _RETRY;
        
        default: 
          //dprintf("Wait error: %d\n", GetLastError()); 
          goto _EXIT_ERROR ;
    }


// --------------- write the next file received in the multi-frames file

_WRITE_FILE:
	  //dprintf3("... writting frames [%d]-[%d] to the file", frameIdx, frameIdxLast);
	  for( bufptr=0; dwFrameIdx <= dwFrameIdxLast; dwFrameIdx++, bufptr += m_imgsizeBytes){
      data = ((char *) m_allocatedBufferPtr[bufIdx]) + bufptr;
      //dprintf4("<%s>: Writing data size[%d] frame[%d] bufIdx[%d]\n", fnId, ds->ccd.filesave.datasize, frameIdx, bufIdx);
      // if ((nr = _write(fp, data, ds->ccd.filesave.datasize)) != ds->ccd.filesave.datasize) {
      // --------------- if needed assign the next frame to this buffer  
      if(dwFrameFirst2assign <= dwRequestedFrames) {
        assignImage2Buffer(dwFrameFirst2assign, dwFrameLast2assign, dwRequestedFrames, bufIdx);
      }
	  } // for frameIdx writting ONE buffer with N images
	} // while(frameIdx ...


	// if stop is requested the while loop is break to this point
	
	
	
	// --------------- close the multi-frames file  
	// --------------- close the sinogram file  
	// --------------- close the sinogram file [end]  

	//_endthread();


_EXIT_ERROR:
//	dprintf("<%s> ERROR & EXIT[%s]", fnId, ds->ccd.filesave.filename);

//	_endthread();
;
}

