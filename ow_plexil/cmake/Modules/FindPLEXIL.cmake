# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

######################################################################
# Find script for PLEXIL.
#
# The PLEXIL directory structure is pretty wacky, so we
# don't try to define _INCLUDE_DIR or some of the other
# standard variables.
#
# Output Variables:
# -----------------
# PLEXIL_FOUND               : TRUE if search succeded
# PLEXIL_COMPILER            : full path to plexilc


######################################################################
message(STATUS "Looking for PLEXIL...")

include(GetLibraryList)

find_path( PLEXIL_INCLUDE_DIR
           NAMES PlexilExec.hh
           PATH_SUFFIXES include
           HINTS ENV PLEXIL_HOME )

if( PLEXIL_INCLUDE_DIR )
  string(REGEX REPLACE "/[^/]*$" "" PLEXIL_ROOT ${PLEXIL_INCLUDE_DIR} )
  set( PLEXIL_LIBRARY_DIR ${PLEXIL_ROOT}/lib )

  find_program( PLEXIL_COMPILER
                NAMES plexilc
                PATH_SUFFIXES scripts bin
                PATHS ${PLEXIL_ROOT}
                NO_DEFAULT_PATH )

  if( PLEXIL_COMPILER )
    include( plexil_functions )
  endif()

  set( PLEXIL_LIBS
        GanttListener
#        IpcAdapter
#        IpcUtils
        Launcher
#        LuvListener
        PlanDebugListener
        PlexilAppFramework
        PlexilExec
        PlexilExpr
        PlexilIntfc
        PlexilSockets
        PlexilUtils
        PlexilValue
        PlexilXmlParser
        pugixml
#        SampleAdapter
#        standalonesimulator
#        UdpAdapter
#        UdpUtils )
       )

  get_library_list(PLEXIL ${PLEXIL_LIBRARY_DIR} "d" "${PLEXIL_LIBS}")

  #message(STATUS "PLEXIL_INCLUDE_DIR = ${PLEXIL_INCLUDE_DIR}")
  #message(STATUS "       PLEXIL_ROOT = ${PLEXIL_ROOT}")
  #message(STATUS "   PLEXIL_COMPILER = ${PLEXIL_COMPILER}")

endif()

if( PLEXIL_COMPILER )
  message( STATUS "  PLEXIL found in ${PLEXIL_ROOT}." )
  set( PLEXIL_FOUND TRUE )
else()
  message( STATUS "  PLEXIL NOT found!" )
  set( PLEXIL_FOUND FALSE )
  if( PLEXIL_FIND_REQUIRED )
    message( FATAL_ERROR "Could not find PLEXIL but it is marked as REQUIRED" )
  endif()
endif()
