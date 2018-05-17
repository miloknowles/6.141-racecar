cmake_minimum_required(VERSION 2.8.3)
project(obstacle_avoidance)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  rospy
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
# https://github.com/mehditlili/boost-python-catkin-example
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs REQUIRED) # sets ${PYTHON_INCLUDE_DIRS}

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS PlanningLibC
	LIBRARIES PlanningLibC
  CATKIN_DEPENDS message_runtime rospy sensor_msgs std_msgs roscpp geometry_msgs nav_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(PlanningLibC)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  src/path_planner.py
  src/pure_pursuit.py
  src/trajectory_builder.py
  src/trajectory_loader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Build PlanningLibC.
add_subdirectory(./PlanningLibC)
