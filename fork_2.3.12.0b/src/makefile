INCLUDEDIR=$(SBX_ROOT)/workspace/linux-pcn1001/include
CROSS=/var/lib/sandbox/toolchains/gcc-4.0.2-glibc-2.3.2/arm-eurotech-linux/bin/
CC=$(CROSS)arm-eurotech-linux-g++
SYSTEM=PCN1001

CFLAGS=  -O2 -DNDEBUG -DPCN_VERSION -D_THREAD_SAFE -Wall -Wno-deprecated -mcpu=xscale 
LDFLAGS= -lpthread -lm

TARGET = imgserver
PCNLIB = libpcn.a
SRC = imgserver.cpp 
LIB = peopledetection.cpp peopledetection.h peopletrack.cpp peopletrack.h directives.h \
      blob_detection.cpp blob_detection.h blob_tracking.cpp blob_tracking.h \
      hungarian_method.cpp hungarian_method.h record_utils.cpp record_utils.h \
      BPmodeling.cpp BPmodeling.h OutOfRangeManager.cpp  OutOfRangeManager.h\
      morphology.cpp morphology.h
PUBLICSRC = imgserver.cpp imgserver.h calib_io.cpp commands.cpp default_parms.h images_fpga.cpp \
	    io.cpp loops.cpp serial_port.cpp socket.cpp directives.h


daemon : $(SRC:.cpp=.o)
		$(CC) $^ $(LDFLAGS) $(PCNLIB) -o $(TARGET)
		$(CROSS)arm-eurotech-linux-strip $(TARGET) 

lib : $(LIB:.cpp=.o)
		rm -f $(PCNLIB)
		$(CROSS)arm-eurotech-linux-ar rs $(PCNLIB) $^
		
pkg : 
		tar cfj imgserver-src.tar.bz2 $(PCNLIB) $(PUBLICSRC)
		
.cpp.o:
		$(CC) -D$(SYSTEM) -c $(CFLAGS) -I $(INCLUDEDIR) $<
		
all : lib daemon

clean:
		rm -f imgserver *.o *~ core 

