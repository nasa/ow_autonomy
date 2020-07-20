cmake_minimum_required(VERSION 2.8.3)
project(ow_autonomy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  actionlib
  actionlib_msgs
  ow_lander
)

list(INSERT CMAKE_MODULE_PATH 0
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

find_package(PLEXIL REQUIRED)

add_action_files(
  DIRECTORY action
  FILES GuardedMove.action
	)

generate_messages(
  DEPENDENCIES actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp roslib ow_lander actionlib_msgs
  CFG_EXTRAS ow_autonomy-extras.cmake
)

catkin_add_env_hooks(60.ow_autonomy
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)


###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PLEXIL_INCLUDE_DIR}
)

add_subdirectory(src)
