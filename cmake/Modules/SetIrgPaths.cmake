if(NOT ARCHITECTURE)
  include(SetArchitecture)
endif(NOT ARCHITECTURE)

# We need to find a good way to switch IRG_PREFIX from the top level
# For now, you can call cmake using -DIRG_PREFIX=/irg for NFS build
#-------------------------------------------------
if( NOT IRG_PREFIX )

  set( IRG_PREFIX /usr/local/irg )
  if(WIN32) 
    set( IRG_PREFIX c:/devel )
  endif(WIN32)
  
endif( NOT IRG_PREFIX )

if( NOT IRG_PACKAGES_DIR )
  set( IRG_PACKAGES_DIR ${IRG_PREFIX}/packages/${ARCHITECTURE} )
endif( NOT IRG_PACKAGES_DIR )

if( NOT IRG_RELEASES_DIR )
  set( IRG_RELEASES_DIR ${IRG_PREFIX}/releases )
endif( NOT IRG_RELEASES_DIR )
