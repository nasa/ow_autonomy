# This Makefile supports a "standalone" build of the PLEXIL adapter in this
# directory.  It uses the host's default C++ compiler and the PLEXIL installaton
# indicated by $PLEXIL_HOME.  It installs its product, a dynamic library, in
# $PLEXIL_HOME/lib.

# Note that this Makefile does not support the deployment version of the PLEXIL
# application, in which PLEXIL will be integrated using the Robot Operating
# System (ROS).

include $(PLEXIL_HOME)/makeinclude/standard-defs.make

OSTYPE   ?= $(shell uname -s)
MACHTYPE ?= $(shell uname -p)
OS_ARCH = $(OSTYPE)-$(MACHTYPE)

LIBRARY = ow_adapter

SRC = subscriber.cc OwAdapterNative.cc OwAdapterROS.cc OwSimProxy.cc

INC = subscriber.hh OwAdapter.hh OwSimProxy.hh

# Add our own (not PLEXIL) include directories here.
INC_DIRS +=

# Add our own (not PLEXIL) libraries here.
LIBS +=

# Add our own (not PLEXIL) library flags here.
LIB_FLAGS +=

# Customize the target library name/location here, if needed.
#LIB_TARGET =

# Use c++11
# NOTE: Before doing this, PLEXIL needs to built for C++ 11.  The procedure for
# doing this is not documented, but should amount to adding the '-std=c++11'
# flag to INITIAL_CXXFLAGS in $PLEXIL_HOME/makeinclude/standard-defs.make.
# Those of us on the CHAP-E project can also take a look at the plexilz make
# file.
#STANDARD_CXXFLAGS += -std=c++11

include $(PLEXIL_HOME)/makeinclude/standard-targets.make


