include $(PLEXIL_HOME)/makeinclude/standard-defs.make

OSTYPE   ?= $(shell uname -s)
MACHTYPE ?= $(shell uname -p)
OS_ARCH = $(OSTYPE)-$(MACHTYPE)

LIBRARY = OwAdapter

SRC = subscriber.cc OwAdapter.cc

INC = subscriber.hh OwAdapter.hh

INC_DIRS += $(PLEXIL_HOME)/src/third-party/pugixml/src $(PLEXIL_HOME)/src/utils \
 $(PLEXIL_HOME)/src/value $(PLEXIL_HOME)/src/expr $(PLEXIL_HOME)/src/intfc \
 $(PLEXIL_HOME)/src/exec $(PLEXIL_HOME)/src/app-framework

LIBS += pugixml PlexilAppFramework PlexilExec PlexilIntfc PlexilExpr \
        PlexilValue PlexilUtils

LIB_FLAGS += -L $(PLEXIL_HOME)/lib

LIB_TARGET = lib$(LIB_NAME)$(LIB_EXT)

# Use c++11
STANDARD_CXXFLAGS += -std=c++11

include $(PLEXIL_HOME)/makeinclude/standard-targets.make

# DO NOT DELETE
