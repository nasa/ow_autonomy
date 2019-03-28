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
# PLEXIL_EXEC_LIBRARY        : full path to Exec_g
# PLEXIL_UTILS_LIBRARY       : full path to PlexilUtils_g
# PLEXIL_TINYXML_LIBRARY     : full path to PlexilTinyXml_g
# PLEXIL_SOCKETS_LIBRARY     : full path to Sockets_g
# PLEXIL_INTERFACE_LIBRARY   : full path to PlexilInterface_g
# PLEXIL_CORBA_UTILS_LIBRARY : full path to PlexilCorbaUtils_g
# PLEXIL_EVENT_CHANNEL_REPORTER_LIBRARY : full path to EventChannelReporter_g
#
######################################################################
message(STATUS "Looking for PLEXIL")

include(GetReleaseSearchPath)

set(     PACKAGE PLEXIL )
set( PACKAGE_DIR plexil_exec )

get_release_search_path( "${PACKAGE}" "${PACKAGE_DIR}" PLEXIL_ROOT_DIR PLEXIL_ROOT )

set( UNIVERSAL_LIB_DIR "lib" )
find_library( PLEXIL_EXEC_LIBRARY PlexilExec
  ${RELEASE_SEARCH_PATH}/${UNIVERSAL_LIB_DIR}
  "${PACKAGE} PlexilExec library"
)
#
# set(  INTERFACE_LIB_DIR "interfaces/lib" )
# find_library( PLEXIL_INTERFACE_LIBRARY PlexilInterface_g
#   ${RELEASE_SEARCH_PATH}/${INTERFACE_LIB_DIR}
#   "${PACKAGE} PlexilInterface_g library"
# )

#if( PLEXIL_EXEC_LIBRARY AND PLEXIL_INTERFACE_LIBRARY )

  # Set the root to 1 directories above base library.
  #-----------------------------------------
  string(REGEX REPLACE "/[^/]*/[^/]*$" "" PLEXIL_ROOT_DIR ${PLEXIL_EXEC_LIBRARY} )


  message(STATUS "  Found PLEXIL in ${PLEXIL_ROOT_DIR}")
  set( PLEXIL_FOUND TRUE )
  set( PLEXIL_INCLUDE_DIR ${PLEXIL_ROOT_DIR}/include CACHE PATH "Plexil include directory" )
  set( PLEXIL_LIBRARY_DIR ${PLEXIL_ROOT_DIR}/lib     CACHE PATH "Plexil library directory" )
  mark_as_advanced(PLEXIL_INCLUDE_DIR)
  mark_as_advanced(PLEXIL_LIBRARY_DIR)
  message(STATUS "  Found PLEXIL include ${PLEXIL_INCLUDE_DIR}")

  
#   find_library( PLEXIL_UTILS_LIBRARY   PlexilUtils_g   ${RELEASE_SEARCH_PATH}/${UNIVERSAL_LIB_DIR} )
#   find_library( PLEXIL_TINYXML_LIBRARY PlexilTinyXml_g ${RELEASE_SEARCH_PATH}/${UNIVERSAL_LIB_DIR} )
#   find_library( PLEXIL_SOCKETS_LIBRARY Sockets_g       ${RELEASE_SEARCH_PATH}/${UNIVERSAL_LIB_DIR} )
#   mark_as_advanced( PLEXIL_EXEC_LIBRARY    )
#   mark_as_advanced( PLEXIL_UTILS_LIBRARY   )
#   mark_as_advanced( PLEXIL_TINYXML_LIBRARY )
#   mark_as_advanced( PLEXIL_SOCKETS_LIBRARY )
#
#   find_library( PLEXIL_CORBA_UTILS_LIBRARY PlexilCorbaUtils_g ${RELEASE_SEARCH_PATH}/${INTERFACE_LIB_DIR} )
#   find_library( PLEXIL_EVENT_CHANNEL_REPORTER_LIBRARY EventChannelReporter_g ${RELEASE_SEARCH_PATH}/${INTERFACE_LIB_DIR} )
#   mark_as_advanced( PLEXIL_INTERFACE_LIBRARY  )
#   mark_as_advanced( PLEXIL_CORBA_UTILS_LIBRARY )
#   mark_as_advanced( PLEXIL_EVENT_CHANNEL_REPORTER_LIBRARY )
  
#else( PLEXIL_EXEC_LIBRARY AND PLEXIL_INTERFACE_LIBRARY )

#   set( PLEXIL_FOUND FALSE )
#   message(STATUS ${RELEASE_SEARCH_ERROR_MESSAGE})

#endif( PLEXIL_EXEC_LIBRARY AND PLEXIL_INTERFACE_LIBRARY )

