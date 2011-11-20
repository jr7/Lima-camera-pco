#ifndef PCOINTERFACE_H
#define PCOINTERFACE_H



#include "LimaCompatibility.h"

#include "Debug.h"
#include "HwInterface.h"
#include "PcoCamera.h"



namespace lima
{
  namespace Pco
  {
    class Camera;
    class DetInfoCtrlObj;
    class BufferCtrlObj;
    class SyncCtrlObj;

    class  DLL_EXPORT Interface : public HwInterface
    {
      DEB_CLASS_NAMESPC(DebModCamera, "Interface", "Pco");

    public:
      Interface(Camera*);
      virtual ~Interface();

      virtual void getCapList(CapList &) const;

      virtual void reset(ResetLevel reset_level);
      virtual void prepareAcq();
      virtual void startAcq();
      virtual void stopAcq();
      virtual void getStatus(StatusType& status);

      virtual int getNbAcquiredFrames();
      virtual int getNbHwAcquiredFrames();

      double getCocRunTime(){ return m_cam->pcoGetCocRunTime() ;};
      double getFrameRate(){ return m_cam->pcoGetFrameRate() ;};


    private:
      Camera* 		m_cam;
      DetInfoCtrlObj* 	m_det_info;
      BufferCtrlObj* 	m_buffer;
      SyncCtrlObj* 	m_sync;
    };

  } // namespace Pco

} // namespace lima

#endif // PCOINTERFACE_H
