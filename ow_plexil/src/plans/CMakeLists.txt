# Builds every .plp file in this directory.

if(OWLAT)
  file( GLOB PLEXIL_SRC_PLANS
    "${CMAKE_CURRENT_SOURCE_DIR}/owlat/*.plp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/*.plp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.plp")
  set( PLEXIL_CONFIG_FILES "owlat/owlat-config.xml" "plexil-debug.cfg")
else()
  file( GLOB PLEXIL_SRC_PLANS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.plp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/*.plp")
  set( PLEXIL_CONFIG_FILES "ow-config.xml" "plexil-debug.cfg")
endif()

set( PLEXIL_PLAN_DIR etc/plexil )
add_plexil_plans( ${PLEXIL_SRC_PLANS} )
add_plexil_configs( "${PLEXIL_CONFIG_FILES}" )
