set(LIB_NAME ow_adapter)

OPTION(OWLAT "Build to also support JPL's OWLAT simulator" OFF)

# OceanWATERS code (required)

set (HEADERS
  joint_support.h
  subscriber.h
  action_support.h
  adapter_support.h
  PlexilInterface.h
  OwExecutive.h
  OwInterface.h
  CommonAdapter.h
  OwAdapter.h
  PlexilPlanSelection.h
  TerminalPlanSelection.h
)

set (SOURCES
  subscriber.cpp
  action_support.cpp
  adapter_support.cpp
  PlexilInterface.cpp
  OwExecutive.cpp
  OwInterface.cpp
  CommonAdapter.cpp
  OwAdapter.cpp
  PlexilPlanSelection.cpp
  TerminalPlanSelection.cpp
)

# OWLAT code (optional)

if(OWLAT)
  set (HEADERS ${HEADERS} OwlatAdapter.h OwlatInterface.h)
  set (SOURCES ${SOURCES} OwlatAdapter.cpp OwlatInterface.cpp)
endif()

add_definitions(-DUSING_ROS)

include_directories(
  ${PLEXIL_INCLUDE_DIR}
)

add_library(${LIB_NAME} SHARED
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${LIB_NAME}
  ${catkin_LIBRARIES}
  ${PLEXIL_LIBRARIES}
  )

add_dependencies(${LIB_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_executable(ow_exec_node ow_exec_node.cpp)
add_executable(terminal_selection_node terminal_selection_node.cpp)

target_link_libraries(ow_exec_node
  ${catkin_LIBRARIES}
  ${PLEXIL_LIBRARIES}
  ${LIB_NAME})

target_link_libraries(terminal_selection_node
  ${catkin_LIBRARIES}
  ${PLEXIL_LIBRARIES}
  ${LIB_NAME})

install(TARGETS ow_exec_node terminal_selection_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(OWLAT)
  add_executable(owlat_exec_node owlat_exec_node.cpp)

  target_link_libraries(owlat_exec_node
    ${catkin_LIBRARIES}
    ${PLEXIL_LIBRARIES}
    ${LIB_NAME})

  install(TARGETS owlat_exec_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
