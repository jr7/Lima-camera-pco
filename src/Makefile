############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2011
# European Synchrotron Radiation Facility
# BP 220, Grenoble 38043
# FRANCE
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################

pco-objs :=  \
            PcoHwEventCtrlObj.o  \
            PcoDetInfoCtrlObj.o  \
            PcoInterface.o  \
            PcoRoiCtrlObj.o  \
            PcoSyncCtrlObj.o  \
            PcoCameraUtils.o  \
            PcoCamera.o  \
            PcoBufferCtrlObj.o  \


SRCS = $(pco-objs:.o=.cpp) 

INCLUDES  = -I../include \
            -I../../../hardware/include \
            -I../../../common/include \
            -I../sdkPco/include \
            -I../../../third-party/Processlib/core/include \


CXXFLAGS += $(INCLUDES) -fPIC -Wall -pthread -g


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
