set(LIB_NAME ow_adapter)

set (HEADERS
  OwExecutive.h
  OwInterface.h
  OwAdapter.h
  OwSimInterface.h
  OwSimProxy.h
	joint_support.h
  subscriber.h
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

add_dependencies(${LIB_NAME}
  ${${LIB_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_executable(autonomy_node autonomy_node.cpp)
add_executable(MoveGuardedServer MoveGuardedServer.cpp)
add_executable(MoveGuardedClient MoveGuardedClient.cpp)

target_link_libraries(autonomy_node
  ${catkin_LIBRARIES}
  ${PLEXIL_LIBRARIES}
  ${LIB_NAME})

target_link_libraries(MoveGuardedServer
  ${catkin_LIBRARIES})

add_dependencies(MoveGuardedServer
  ${MoveGuardedServer_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(MoveGuardedClient
  ${catkin_LIBRARIES})

add_dependencies(MoveGuardedClient
  ${MoveGuardedClient_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

install(TARGETS autonomy_node MoveGuardedServer MoveGuardedClient
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
