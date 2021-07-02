cmake_minimum_required(VERSION 2.8.3)
project(ow_autonomy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  actionlib
  actionlib_msgs
  geometry_msgs
  ow_lander
  ow_faults
  rqt_gui_py
)

list(INSERT CMAKE_MODULE_PATH 0
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

find_package(PLEXIL REQUIRED)

#generate_messages(
#  DEPENDENCIES actionlib_msgs geometry_msgs
#)


###################################
## catkin specific configuration ##
###################################

## for GUI
catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy roslib ow_lander actionlib_msgs geometry_msgs ow_faults
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

#############
## INSTALL ##
#############

## Mark executable python script for installation
install(PROGRAMS
  scripts/rqt_plexil_planner
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark resource package for installation
install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
## Mark plugin.xml file for installation
install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

