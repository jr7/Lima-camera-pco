#ifndef PCOBUFFERCTRLOBJ_H
#define PCOBUFFERCTRLOBJ_H

#include "Pco.h"

#include "HwBufferMgr.h"



struct stcAllocBuff {
		SHORT	bufferNr[8];				// bufnrM buffer number allocated by PCO_AllocateBuffer
        WORD	*bufferPtr[8];			// buffer allocated by PCO_AllocateBuffer
        HANDLE bufferEvent[8];

        DWORD bufferAssignedFrameFirst[8];
        DWORD bufferAssignedFrameLast[8];
        int bufferReady[8];

};

namespace lima
{
  namespace Pco
  {
    class Camera;
    class SyncCtrlObj;
    class Interface;

    class DLL_EXPORT BufferCtrlObj : public SoftBufferCtrlMgr
    {
      friend class Interface;
      DEB_CLASS_NAMESPC(DebModCamera,"BufferCtrlObj","Pco");
    
	public:
      BufferCtrlObj(Camera *cam);
      void prepareAcq();
      void startAcq();
      void getStatus(int &err,bool& exposing) {err = m_status,exposing = m_exposing;}

        //-------------------------------------------------------------  moved from taco
        
	  
        int _xferImag();
		  
	
	private:
      //static void _newFrame(tPvFrame*);
		void _assignImage2Buffer(DWORD &dwFrameFirst, DWORD &dwFrameLast, DWORD dwRequestedFrames, int bufIdx);

		struct stcAllocBuff m_allocBuff;
       	unsigned long	m_frames_per_buffer;
        //-------------------------------------------------------------

      HANDLE&      	m_handle;
      int        	m_frame[2];
      Camera* m_cam;
      SyncCtrlObj* 	m_sync;
      int		m_status;
      bool		m_exposing;
    };
  }
}
#endif
