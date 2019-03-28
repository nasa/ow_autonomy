cmake_minimum_required(VERSION 2.8.3)
project(ow_autonomy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  irg_cmake
  roscpp
)

find_package(PLEXIL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ow_adapter
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PLEXIL_INCLUDE_DIR}
)


add_subdirectory(src)
