set(LIB_NAME ow_adapter)

set (HEADERS
  OwExecutive.h
  OwInterface.h
  OwAdapter.h
  OwSimInterface.h
  OwSimProxy.h
  subscriber.h
  support.h
)

set (SOURCES
  OwExecutive.cpp
  OwInterface.cpp
  OwAdapter.cpp
  OwSimInterfaceROS.cpp
  OwSimProxy.cpp
  subscriber.cpp
)

add_definitions(-DUSING_ROS)

include_directories(
  ${PLEXIL_INCLUDE_DIR}
)

# For debugging
# message(STATUS "PLEXIL_INCLUDE_DIR = ${PLEXIL_INCLUDE_DIR}")

add_library(${LIB_NAME} SHARED
  ${HEADERS}
  ${SOURCES}
)

target_link_libraries(${LIB_NAME}
  ${catkin_LIBRARIES}
  ${PLEXIL_LIBRARIES}
  )

add_executable(autonomy_node autonomy_node.cpp)

target_link_libraries(autonomy_node
  ${catkin_LIBRARIES}
  ${PLEXIL_LIBRARIES}
  ${LIB_NAME})

install(TARGETS autonomy_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
