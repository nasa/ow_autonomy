# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

# Add plexil plans to be compiled. It seems that the plexil
# compiler cannot compile plans in parallel safely. This function
# will serialize compilation for plans in this directory ONLY,
# so if there are multiple directories in the project(s) that are
# being compiled in parallel, there may be build failures
#####################################################################
function(add_plexil_plans PLAN_SRC_FILES)

  set(PLAN_LIST)

  # it appears that PLEXILC has problems with parallel builds. Therefore,
  # we set up the plans to be built in a serial manner by specifying
  # targets for each plan and each plan depends on the compiled output of
  # the plan preceeding it
  foreach(PLAN ${PLAN_SRC_FILES} ${ARGN})
    # Get name without extension (NAME_WE) and append to list of tables
    get_filename_component(PLANWE ${PLAN} NAME_WE)
    get_filename_component(PLANEX ${PLAN} EXT)
    get_filename_component(PLANRD ${PLAN} DIRECTORY)

    set(PLEXILC_OUTPUT ${PLANWE}.plx)
    add_custom_target( ${PLEXILC_OUTPUT} ALL DEPENDS ${PLEXILC_OUTPUTS} )

    add_custom_command(
      TARGET  ${PLEXILC_OUTPUT}
      COMMAND ${PLEXIL_COMPILER} -o ${PLEXILC_OUTPUT} ${PLANRD}/${PLANWE}${PLANEX}
      DEPENDS ${PLAN}
    )
    list(APPEND PLAN_LIST ${PLEXILC_OUTPUT})
    set( PLEXILC_OUTPUTS ${PLEXILC_OUTPUTS} ${PLEXILC_OUTPUT} )

  endforeach(PLAN ${PLAN_SRC_FILES} ${ARGN})

  # install the compiled plans into install or devel space
  # We need to explicitly copy the compiled plans into devel space
  # because of the way catkin does things
  if( NOT PLEXIL_PLAN_DIR )
    set(PLEXIL_PLAN_DIR data/plx/plans)
  endif()

  foreach(PLAN_FILE ${PLAN_LIST})
    install(
      FILES ${CMAKE_CURRENT_BINARY_DIR}/${PLAN_FILE}
      DESTINATION ${PLEXIL_PLAN_DIR}
      )

    if( catkin_FOUND )
      set( PLEXILC_TARGET PLEXILC_${PLAN_FILE} )

      add_custom_target( ${PLEXILC_TARGET} ALL DEPENDS ${PLAN_LIST})

      add_custom_command(
        TARGET ${PLEXILC_TARGET}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/${PLAN_FILE}
        ${CATKIN_DEVEL_PREFIX}/${PLEXIL_PLAN_DIR}/${PLAN_FILE})
    endif()

  endforeach(PLAN_FILE ${PLAN_LIST})

endfunction(add_plexil_plans)


# copy config file into compiled plan dir
####################################################################
function(add_plexil_configs PLAN_CFG_FILES)
  foreach(PLAN_CFG_FILE ${PLAN_CFG_FILES})
    install(FILES ${PLAN_CFG_FILE} DESTINATION ${PLEXIL_PLAN_DIR})
    if( catkin_FOUND AND CATKIN_DEVEL_PREFIX)
      exec_program("${CMAKE_COMMAND}" ARGS
        -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/${PLAN_CFG_FILE}"
        "${CATKIN_DEVEL_PREFIX}/${PLEXIL_PLAN_DIR}/${PLAN_CFG_FILE}"
      )
    endif()
  endforeach()
endfunction(add_plexil_configs)
