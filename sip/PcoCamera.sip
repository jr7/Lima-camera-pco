namespace Pco
{
  class Camera
  {
%TypeHeaderCode
#include "lima/HwInterface.h"
#include <PcoCamera.h>
%End
  public:

        Camera(const char*);
        ~Camera();


        void 	startAcq();
        void	reset();




        void getMaxWidthHeight(unsigned long& width,unsigned long& height /Out/);
        void getBytesPerPixel(unsigned int& pixbytes /Out/);
        int getNbAcquiredFrames();
        char *talk(char *cmd);       

  };
};


