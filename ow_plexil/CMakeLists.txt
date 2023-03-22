cmake_minimum_required(VERSION 3.0.2)

# Top-level build file for the PLEXIL executive, conditioned to build
# one customized for either OceanWATERS or JPL's OWLAT testbed.

project(ow_plexil)

OPTION(OWLAT "Build to support JPL's OWLAT simulator instead of OceanWATERS" OFF)

# Find the packages that are used by both OceanWATERS and OWLAT.

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  actionlib
  actionlib_msgs
  geometry_msgs
  rqt_gui_py
  message_generation
  std_msgs
  )

if(OWLAT)
  # Intended to only make the owl_msgs package be found here.
  list(INSERT CMAKE_PREFIX_PATH 0 "/opt/ros/noetic/share")
else()
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    actionlib
    actionlib_msgs
    geometry_msgs
    rqt_gui_py
    message_generation
    std_msgs
    owl_msgs
    ow_lander
    ow_faults_detection
    )
endif()

# Use OWLAT's owl_msgs package (under /opt/ros) if building OWLAT,
# ours otherwise (ow_simulator/owl_msgs).  The path is set in the
# previous statement.
find_package(owl_msgs)

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

# for GUI
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

if(OWLAT)
  catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp roslib rospy actionlib_msgs geometry_msgs owl_msgs
    CFG_EXTRAS ow_plexil-extras.cmake
    )
else()
  catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp roslib rospy actionlib_msgs geometry_msgs ow_faults_detection ow_lander owl_msgs
    CFG_EXTRAS ow_plexil-extras.cmake
    )
endif()

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
