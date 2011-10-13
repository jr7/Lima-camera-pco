#ifndef PCOBUFFERCTRLOBJ_H
#define PCOBUFFERCTRLOBJ_H

#include "Pco.h"

#include "HwBufferMgr.h"

typedef struct
{
    //----- In -----
    void*               ImageBuffer;        // Your image buffer
    unsigned long       ImageBufferSize;    // Size of your image buffer in bytes

    void*               AncillaryBuffer;    // Your buffer to capture associated
                                            //   header & trailer data for this image.
    unsigned long       AncillaryBufferSize;// Size of your ancillary buffer in bytes
                                            //   (can be 0 for no buffer).

    void*               Context[4];         // For your use (valuable for your
                                            //   frame-done callback).
    unsigned long       _reserved1[8];

    //----- Out -----

    tPvErr              Status;             // Status of this frame

    unsigned long       ImageSize;          // Image size, in bytes
    unsigned long       AncillarySize;      // Ancillary data size, in bytes

    unsigned long       Width;              // Image width
    unsigned long       Height;             // Image height
    unsigned long       RegionX;            // Start of readout region (left)
    unsigned long       RegionY;            // Start of readout region (top)
    //tPvImageFormat      Format;             // Image format
    unsigned long       BitDepth;           // Number of significant bits
    //tPvBayerPattern     BayerPattern;       // Bayer pattern, if bayer format

    unsigned long       FrameCount;         // Rolling frame counter
    unsigned long       TimestampLo;        // Time stamp, lower 32-bits
    unsigned long       TimestampHi;        // Time stamp, upper 32-bits

    unsigned long       _reserved2[32];

} tPvFrame;



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
    private:
      static void _newFrame(tPvFrame*);

        //-------------------------------------------------------------  moved from taco
        void assignImage2Buffer(DWORD &dwFrameFirst, DWORD &dwFrameLast, DWORD dwRequestedFrames, int bufIdx);
        void xferImag();

        SHORT	m_allocatedBufferNr[8];				// bufnrM buffer number allocated by PCO_AllocateBuffer
        WORD	*m_allocatedBufferPtr[8];			// buffer allocated by PCO_AllocateBuffer
        HANDLE m_allocatedBufferEvent[8];

        DWORD m_allocatedBufferAssignedFrameFirst[8];
        DWORD m_allocatedBufferAssignedFrameLast[8];
        int m_allocatedBufferReady[8];

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
