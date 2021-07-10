set(LIB_NAME ow_adapter)

set (HEADERS
  OwExecutive.h
  OwInterface.h
  OwAdapter.h
  joint_support.h
  subscriber.h
  PlexilPlanSelection.h
  TerminalPlanSelection.h
)

set (SOURCES
  OwExecutive.cpp
  OwInterface.cpp
  OwAdapter.cpp
  subscriber.cpp
  PlexilPlanSelection.cpp
  TerminalPlanSelection.cpp
)

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

add_executable(plexil_node plexil_node.cpp)
add_executable(plexil_plan_selection_node plexil_plan_selection_node.cpp)

target_link_libraries(plexil_node
  ${catkin_LIBRARIES}
  ${PLEXIL_LIBRARIES}
  ${LIB_NAME})

target_link_libraries(plexil_plan_selection_node
  ${catkin_LIBRARIES}
  ${PLEXIL_LIBRARIES}
  ${LIB_NAME})

install(TARGETS plexil_node plexil_plan_selection_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

