set(LIB_NAME ow_adapter)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PLEXIL_INCLUDE_DIR}
)



#add_dependencies(${LIB_NAME} ow_lander)

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
#	autonomy_node.cc
)

add_definitions(-DUSING_ROS)

add_library(${LIB_NAME} SHARED
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${LIB_NAME}
  ${catkin_LIBRARIES}
  ${PLEXIL_LIBRARIES}
)
