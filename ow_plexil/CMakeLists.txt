cmake_minimum_required(VERSION 3.0.2)
project(ow_plexil)

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
  ow_faults_detection
  rqt_gui_py
  message_generation
  std_msgs
)

list(INSERT CMAKE_MODULE_PATH 0
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

find_package(PLEXIL REQUIRED)

add_service_files(
  FILES
  PlanSelection.srv
)

add_action_files(
  FILES
  IdentifyLocation.action
)

## for GUI
catkin_python_setup()

generate_messages(
  DEPENDENCIES 
  geometry_msgs
  std_msgs
  actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib rospy ow_lander actionlib_msgs geometry_msgs ow_faults_detection
  CFG_EXTRAS ow_plexil-extras.cmake
)

catkin_add_env_hooks(60.ow_plexil
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
  rqt_plexil_plan_selection/scripts/rqt_plexil_plan_selection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark resource package for installation
install(DIRECTORY
  rqt_plexil_plan_selection/resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
## Mark plugin.xml file for installation
install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

