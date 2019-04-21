set(LIB_NAME ow_adapter)

set (HEADERS
  OwAdapter.hh
  OwSimProxy.hh
  subscriber.hh
)

set (SOURCES
  OwAdapterNative.cc
  OwAdapterROS.cc
  OwSimProxy.cc
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
