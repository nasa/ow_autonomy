set(LIB_NAME ow_adapter)

# Headers and source files used by both OceanWATERS and OWLAT

set (HEADERS
  subscriber.h
  action_support.h
  adapter_support.h
  PlexilInterface.h
  LanderInterface.h
  OwExecutive.h
  PlexilAdapter.h
  LanderAdapter.h
  PlexilPlanSelection.h
  TerminalPlanSelection.h
  FaultDependencies.h
)

set (SOURCES
  subscriber.cpp
  action_support.cpp
  adapter_support.cpp
  PlexilInterface.cpp
  LanderInterface.cpp
  OwExecutive.cpp
  PlexilAdapter.cpp
  LanderAdapter.cpp
  PlexilPlanSelection.cpp
  TerminalPlanSelection.cpp
  FaultDependencies.cpp
)

# Executive-specific code
if(OWLAT)
  set (HEADERS ${HEADERS} OwlatAdapter.h OwlatInterface.h)
  set (SOURCES ${SOURCES} OwlatAdapter.cpp OwlatInterface.cpp)
else()
  set (HEADERS ${HEADERS} OwAdapter.h OwInterface.h)
  set (SOURCES ${SOURCES} OwAdapter.cpp OwInterface.cpp)
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

add_executable(terminal_selection_node terminal_selection_node.cpp)
target_link_libraries(terminal_selection_node
  ${catkin_LIBRARIES}
  ${PLEXIL_LIBRARIES}
  ${LIB_NAME})

install(TARGETS terminal_selection_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(OWLAT)
  add_executable(owlat_exec_node owlat_exec_node.cpp)
  target_link_libraries(owlat_exec_node
    ${catkin_LIBRARIES}
    ${PLEXIL_LIBRARIES}
    ${LIB_NAME})
  install(TARGETS owlat_exec_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else()
  add_executable(ow_exec_node ow_exec_node.cpp)
  target_link_libraries(ow_exec_node
    ${catkin_LIBRARIES}
    ${PLEXIL_LIBRARIES}
    ${LIB_NAME})
  install(TARGETS ow_exec_node
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
