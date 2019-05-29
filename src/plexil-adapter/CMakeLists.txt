set(LIB_NAME ow_adapter)

set (HEADERS
  OwAdapter.hh
  OwSimInterface.hh
  OwSimProxy.hh
  subscriber.hh
)

set (SOURCES
  OwAdapter.cc
  OwSimInterfaceROS.cc
  OwSimProxy.cc
  subscriber.cc
#	plexil_ros_app.cc
)

add_definitions(-DUSING_ROS)

include_directories(
  ${PLEXIL_INCLUDE_DIR}
)

# message(STATUS "PLEXIL_INCLUDE_DIR = ${PLEXIL_INCLUDE_DIR}")

add_library(${LIB_NAME} SHARED
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${LIB_NAME}
  ${catkin_LIBRARIES}
  ${PLEXIL_LIBRARIES}
	)

add_executable(autonomy_node autonomy_node.cc)
target_link_libraries(autonomy_node ${catkin_LIBRARIES})
install(TARGETS autonomy_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
