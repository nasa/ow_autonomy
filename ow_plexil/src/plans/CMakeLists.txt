# Builds every .plp file in this directory.

file( GLOB PLEXIL_SRC_PLANS "${CMAKE_CURRENT_SOURCE_DIR}/*.plp" )

set( PLEXIL_CONFIG_FILES "ow-config.xml" "owlat-config.xml" "plexil-debug.cfg")
set( PLEXIL_PLAN_DIR etc/plexil )

add_plexil_plans( ${PLEXIL_SRC_PLANS} )
add_plexil_configs( "${PLEXIL_CONFIG_FILES}" )
