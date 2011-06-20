#ifndef PCOBUFFERCTRLOBJ_H
#define PCOBUFFERCTRLOBJ_H

#include "Pco.h"

#include "HwBufferMgr.h"

namespace lima
{
  namespace Pco
  {
    class Camera;
    class SyncCtrlObj;
    class Interface;

    class BufferCtrlObj : public SoftBufferCtrlMgr
    {
      friend class Interface;
      DEB_CLASS_NAMESPC(DebModCamera,"BufferCtrlObj","Pco");
    public:
      BufferCtrlObj(Camera *cam);
      void prepareAcq();
      void startAcq();
      void getStatus(tPvErr &err,bool& exposing) {err = m_status,exposing = m_exposing;}
    private:
      static void _newFrame(tPvFrame*);
      
      tPvHandle&      	m_handle;
      tPvFrame        	m_frame[2];
      SyncCtrlObj* 	m_sync;
      tPvErr		m_status;
      bool		m_exposing;
    };
  }
}
#endif
