#==========================================================================================
# + + +   This Software is released under the "Simplified BSD License"  + + +
# Copyright 2014 F4GKR Sylvain AZARIAN . All rights reserved.
#
#Redistribution and use in source and binary forms, with or without modification, are
#permitted provided that the following conditions are met:
#
#   1. Redistributions of source code must retain the above copyright notice, this list of
#	  conditions and the following disclaimer.
#
#   2. Redistributions in binary form must reproduce the above copyright notice, this list
#	  of conditions and the following disclaimer in the documentation and/or other materials
#	  provided with the distribution.
#
#THIS SOFTWARE IS PROVIDED BY Sylvain AZARIAN F4GKR ``AS IS'' AND ANY EXPRESS OR IMPLIED
#WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
#FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Sylvain AZARIAN OR
#CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#The views and conclusions contained in the software and documentation are those of the
#authors and should not be interpreted as representing official policies, either expressed
#or implied, of Sylvain AZARIAN F4GKR.
#
# Adds PerseusSDR capability to SDRNode
#==========================================================================================


QT       -= core gui

TARGET = CloudSDR_Perseus
TEMPLATE = lib
 
 
win32 {
    DESTDIR = C:/SDRNode/addons
    LIBS += -lusb-1.0
    RC_FILE = resources.rc
    LIBS += -lusb-1.0 -lpthread
    QMAKE_CFLAGS =  -mno-ms-bitfields    
}

unix {
    DESTDIR = /opt/sdrnode/addons
    QMAKE_CFLAGS  += -mno-ms-bitfields
    LIBS += -lusb-1.0 -lpthread
}

SOURCES += \
    entrypoint.cpp \  
    perseus/fpga_code.c \
    perseus/perseus24v41_512.c \
    perseus/perseuserr.c \
    perseus/perseusfx2.c \
    perseus/perseus-in.c \
    perseus/perseus-sdr.c

HEADERS +=\
    entrypoint.h \ 
    driver_version.h \
    perseus/fpga_data.h \
    perseus/perseuserr.h \
    perseus/perseusfx2.h \
    perseus/perseus-in.h \
    perseus/perseus-sdr.h

OTHER_FILES += \
    resources.rc

unix {
    #target.path = /usr/lib
    #INSTALLS += target
}
