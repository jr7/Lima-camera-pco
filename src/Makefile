pco-objs =  \
            PcoHwEventCtrlObj.o  \
            PcoDetInfoCtrlObj.o  \
            PcoInterface.o  \
            PcoRoiCtrlObj.o  \
            PcoSyncCtrlObj.o  \
            PcoCameraUtils.o  \
            PcoBufferCtrlObj.o  \
            PcoCamera.o  \



SRCS = $(pco-objs:.o=.cpp) 

CXXFLAGS += -I../include \
            -I../../../hardware/include \
            -I../../../common/include \
            -I../sdkPco/include \
            -I../../../third-party/Processlib/core/include \
            -pthread -fPIC -g \
            -Wall\

all:	Pco.o

Pco.o:	$(pco-objs)
	$(LD) -o $@ -r $+

clean:
	rm -f *.o *.P

%.o : %.cpp
	$(COMPILE.cpp) -MD $(CXXFLAGS) -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d

-include $(SRCS:.cpp=.P)
