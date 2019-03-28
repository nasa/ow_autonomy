include( SetIrgPaths )

# get_release_path( PKG_DIR_NAME PKG_ROOT_DIR (PKG_ENV_VAR) )
######################################################################
# 
# This maro puts the most likely path for the release package
# into the RELEASE_SEARCH_PATH variable
#
# Note that this differs from the similar package search path 
# in that it returns the root, not where the libraries should be for
# two reasons:
# 1. This call will always return a single directory
# 2. The releases packages have libraries in inconsistent places
#
# The arguments are
# PKG_NAME      - name of the package directory (e.g. "CSpice")
# PKG_DIR_NAME  - name of the package directory (e.g. "cspice-1.0")
# PKG_ROOT_DIR  - the CMake root var name (e.g. "CSPICE_ROOT_DIR")
# PKG_ENV_VAR   - (optional) the env var that (may) contain the root path
#
# If PKG_ROOT_DIR is defined before this macro is called, 
# NO OTHER PATHS ARE SEARCHED. This is a way of "forcing" 
# the build to use a specified path.
# 
# If PKG_ENV_VAR contains a value, 
# NO OTHER PATHS ARE SEARCHED. 
# 
# Otherwise, the search path will be the default
#
# useage:
# get_release_lib_search_path( CSpice cspice-1.0 CSPICE_ROOT_DIR CSPICE_ROOT )
#
# Output Variables:
# -----------------
# RELEASE_SEARCH_PATH : the path that should be searched for a library or binary
# RELEASE_SEARCH_ERROR_MESSAGE : a useful message to print if the search fails
#
######################################################################

macro( get_release_search_path PKG_NAME PKG_DIR_NAME PKG_ROOT_DIR )

set(RELEASE_SEARCH_PATH "" ) # make sure we've got a clean variable

## If PKG_ROOT_DIR is defined, set 
## to that value and return
##########################################
if( ${PKG_ROOT_DIR} )

  set( RELEASE_SEARCH_PATH ${${PKG_ROOT_DIR}} )
  set( RELEASE_SEARCH_ERROR_MESSAGE "${PKG_NAME} root NOT found!!! ${PKG_ROOT_DIR} is set to \"${${PKG_ROOT_DIR}}\", but library test failed.")
  
else( ${PKG_ROOT_DIR}  )
  
  ##
  ## System search path
  ##########################################
  set( RELEASE_SEARCH_PATH ${RELEASE_SEARCH_PATH}
    ${IRG_RELEASES_DIR}/${PKG_DIR_NAME}
  )
  set( RELEASE_SEARCH_ERROR_MESSAGE "${PKG_NAME} root NOT found!!! Try passing -D${PKG_ROOT_DIR}:PATH=<path> to cmake command")
  
  ##
  ## the fourth argument is the (optional) 
  ## environment variable name
  ##
  ## if the environment variable is set, 
  ## look only in that directory.
  ## (overwrite the default path set above)
  ##########################################
  if( ${ARGC} EQUAL 4 ) 
    
    set( ENV_VAR_VALUE $ENV{${ARGV3}} )
    
    if( ENV_VAR_VALUE )
    
      message( STATUS " env var ${ARGV3}=${ENV_VAR_VALUE}" )
     
      set( RELEASE_SEARCH_PATH ${ENV_VAR_VALUE} )
      set( RELEASE_SEARCH_ERROR_MESSAGE "${PKG_NAME} root NOT found!!! ${ARGV3} env var is set to \"${ENV_VAR_VALUE}\", but library test failed.")
      
    else( ENV_VAR_VALUE )
    
      set( RELEASE_SEARCH_ERROR_MESSAGE "${RELEASE_SEARCH_ERROR_MESSAGE}, or setting the ${ARGV3} environment variable")
    
    endif( ENV_VAR_VALUE )

  endif( ${ARGC} EQUAL 4 ) 
     
endif( ${PKG_ROOT_DIR}  )

endmacro( get_release_search_path )









