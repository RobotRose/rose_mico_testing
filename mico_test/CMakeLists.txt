cmake_minimum_required(VERSION 2.8.3)
project(mico_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    controller_manager
    control_msgs
    ecl_geometry
    hardware_interface
    wpi_jaco_msgs
    jaco_sdk
    roscpp
    sensor_msgs
    std_srvs)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(mico_test src/test.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(mico_test
  ${catkin_LIBRARIES}
)