set(LIB_NAME ow_adapter)

set (HEADERS
  joint_support.h
  subscriber.h
  action_support.h
  PlexilInterface.h
  OwExecutive.h
  OwInterface.h
  OWLATSimInterface.h
  OwAdapter.h
)

set (SOURCES
  subscriber.cpp
	action_support.cpp
  PlexilInterface.cpp
  OwExecutive.cpp
  OwInterface.cpp
  OWLATSimInterface.cpp
  OwAdapter.cpp
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

target_link_libraries(plexil_node
  ${catkin_LIBRARIES}
  ${PLEXIL_LIBRARIES}
  ${LIB_NAME})

install(TARGETS plexil_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
