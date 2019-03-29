set(LIB_NAME ow_adapter)

set (HEADERS
  OwAdapter.hh
  OwSimProxy.hh
  subscriber.hh
)

set (SOURCES
  OwAdapter.cc
  OwSimProxy.cc
  subscriber.cc
)


add_library(${LIB_NAME} SHARED
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${LIB_NAME}
  ${catkin_LIBRARIES}
  ${PLEXIL_LIBRARIES}
)
