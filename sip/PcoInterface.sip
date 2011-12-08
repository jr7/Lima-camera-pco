namespace Pco
{
  class Interface : HwInterface
  {
%TypeHeaderCode
#include <PcoInterface.h>
%End
  
  public:

    Interface(Pco::Camera*);
    virtual ~Interface();

    virtual void	getCapList(std::vector<HwCap> &cap_list /Out/) const;
    virtual void	reset(ResetLevel reset_level);
    virtual void 	prepareAcq();
    virtual void 	startAcq();
    virtual void 	stopAcq();

    virtual void getStatus(StatusType& status /Out/);

    virtual int 	getNbAcquiredFrames();
    virtual int 	getNbHwAcquiredFrames();


    //void _getCocRunTime(double& coc_run_time /Out/);
        //void _getCocRunTime(double& coc_run_time){ coc_run_time = m_cam->m_cocRunTime ;};

    //void _getFrameRate(double& frame_rate /Out/);
        //void _getFrameRate(double& frame_rate){ frame_rate = m_cam->m_frameRate ;};

  };
};



