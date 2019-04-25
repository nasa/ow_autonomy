set(LIB_NAME ow_adapter)

set (HEADERS
  OwAdapter.hh
  OwSimInterface.hh
  subscriber.hh
)

set (SOURCES
  OwAdapter.cc
  OwSimInterfaceNative.cc
  subscriber.cc
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
