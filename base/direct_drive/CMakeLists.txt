cmake_minimum_required(VERSION 2.8.3)
project(direct_drive)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pluginlib
  roscpp
  nav_msgs
  std_msgs
  geometry_msgs
  tf
  ackermann_msgs
  vesc_msgs
  vesc_driver
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    nodelet
    pluginlib
    roscpp
    nav_msgs
    std_msgs
    geometry_msgs
    tf
    ackermann_msgs
    vesc_msgs
    vesc_driver
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# file(GLOB vesc_driver_sources /home/racecar/racecar-ws/src/vesc/vesc_driver/src/*.cpp)

# node executable
add_executable(direct_drive_node src/direct_drive.cpp)
# add_dependencies(direct_drive_node ${catkin_EXPORTED_TARGETS} ${vesc_driver_sources})
add_dependencies(direct_drive_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(direct_drive_node
  vesc_driver_nodelet
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS direct_drive_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############

# TODO