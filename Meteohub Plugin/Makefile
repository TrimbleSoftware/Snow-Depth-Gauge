# make file for meteohub/meteoplug plug-in for the Trimble Snow Depth Gauge USB to serial XBee interface
# for both meteohub on Iomega iConnect hardware running Debian 4 Linux AND meteoplug on TP-LINK MR3020 hardware running OpenWRT Linux
# FTRIMBLE 7-May-2014

CC = $(CC)
LD = $(LD)
CFLAGS = $(CFLAGS)
LDFLAGS = $(LDFLAGS)

# get machine name from OS
UNAME := $(shell uname -m)

# OpenWRT centric make file settings for TP-LINK MR-3020 Trimble Snow Depth Gauge plug-in
# requires OpenWRT SDK to be installed/built on the host OS, in this case debian 6 running in a virtualbox VM on Win7 host OS.
# Assumes that the OpenWRT gcc MIPS crosscompiler/linker is located on the PATH

# gcc settings for OpenWRT MIPS gcc crosscompiler/linker running on VMBox with Debian 6
ifeq ($(UNAME),i686)
	# override CC and LD with OpenWRT MIPS specific cross complier
	CC	= mips-openwrt-linux-uclibc-gcc
	LD	= mips-openwrt-linux-uclibc-gcc
	# override CFLAGS and LDFLAGS to use OpenWRT includes and libs
	CFLAGS  = -s -Wall -O2 -U DEBUG -I /home/meteoplug/openwrt/trunk/staging_dir/toolchain-mips_r2_gcc-4.6-linaro_uClibc-0.9.33.2/usr/include
	LDFLAGS = -s -L /home/meteoplug/openwrt/trunk/staging_dir/toolchain-mips_r2_gcc-4.6-linaro_uClibc-0.9.33.2/usr/lib
endif

# gcc settings for gcc running on Debian 4 in meteohub iConnect kernel
ifeq ($(UNAME),armv5tel)
	# user standard gcc
	CC	= gcc 
	LD	= gcc
	# c and linker flags
	CFLAGS  = -Wall -O2 -U DEBUG
        DEBUGCFLAGS     = -Wall -g3 -D DEBUG
	LDFLAGS = -s -lm
        DEBUGLDFLAGS = -lm
endif

#gcc settings for gcc running on raspbery pi meteohub kernel
ifeq ($(UNAME),armv6l)
	# user standard gcc
	CC 	= gcc
	LD 	= gcc
	# c and linker flags
	CFLAGS	= -Wall -O2 -U DEBUG
	LDFLAGS	= -s -lm
endif

#gcc settings for gcc running on raspbery pi3 meteohub kernel
ifeq ($(UNAME),armv7l)
	# user standard gcc
	CC 	= gcc
	LD 	= gcc
	# c and linker flags
	CFLAGS	= -Wall -O2 -U DEBUG
	DEBUGCFLAGS	= -Wall -g3 -D DEBUG
	LDFLAGS	= -s -lm
	DEBUGLDFLAGS = -lm
endif

all:	mhsdpi

debug: clean debug_compile mhsdpi

gdb: clean gdb_compile mhsdpigdb

mhsdpi:	mhsdpi.o config.o fdget.o
	$(LD) mhsdpi.o config.o fdget.o -o mhsdpi $(LDFLAGS)

mhsdpigdb:	mhsdpi.o config.o fdget.o
	$(LD) mhsdpi.o config.o fdget.o -o mhsdpi $(DEBUGLDFLAGS)

static:	mhsdpi.o config.o fdget.o
	$(LD) -static -o mhsdpi mhsdpi.o config.o fdget.o $(LDFLAGS)

staticgdb:	mhsdpi.o config.o fdget.o
	$(LD) -static -o mhsdpi mhsdpi.o config.o fdget.o $(DEBUGLDFLAGS)

debug_compile:	config.c mhsdpi.c mhsdpi.h fdget.h
	$(CC) $(DEBUGCFLAGS) -c mhsdpi.c -c config.c -c fdget.c

gdb_compile:	config.c mhsdpi.c mhsdpi.h fdget.h
	$(CC) $(DEBUGCFLAGS) -U DEBUG -c mhsdpi.c -c config.c -c fdget.c

mhsdpi.o:	config.c mhsdpi.c mhsdpi.h fdget.c fdget.h
	$(CC) $(CFLAGS) -c mhsdpi.c -o mhsdpi.o

config.o:	config.c mhsdpi.h
	$(CC) $(CFLAGS) -c config.c -o config.o

fdget.o:	fdget.c fdget.h
	$(CC) $(CFLAGS) -c fdget.c -o fdget.o

clean:
	rm -rf mhsdpi *.o *~
