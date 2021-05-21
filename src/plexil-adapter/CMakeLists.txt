set(LIB_NAME ow_adapter)

set (HEADERS
  OwExecutive.h
  OwInterface.h
  OwAdapter.h
  joint_support.h
  subscriber.h
)

set (SOURCES
  OwExecutive.cpp
  OwInterface.cpp
  OwAdapter.cpp
  subscriber.cpp
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

add_executable(autonomy_node autonomy_node.cpp)

target_link_libraries(autonomy_node
  ${catkin_LIBRARIES}
  ${PLEXIL_LIBRARIES}
  ${LIB_NAME})

install(TARGETS autonomy_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
