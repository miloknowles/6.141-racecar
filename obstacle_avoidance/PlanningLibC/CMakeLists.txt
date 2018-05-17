set(PLC_DIR ${CMAKE_CURRENT_LIST_DIR})

# Add any source files here.
SET(PlanningLibC_SRCS
  src/point2.hpp
  src/pose3.hpp
  src/dubins.hpp
  src/dubins.cpp
  src/utils.cpp
  src/pure_pursuit_controller.cpp
  src/occupancy_grid.cpp
  src/motion_planning_tree.hpp
  src/clrrt_node.cpp
  src/clrrt.cpp
  src/PlanningLibC.cpp
)

# Declare a C++ library
add_library(PlanningLibC SHARED
  ${PlanningLibC_SRCS}
)

# Specify libraries to link a library or executable target against
target_link_libraries(PlanningLibC
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# The last command is important because it will change the destination
# of the compiled library from /catkin_ws/devel/lib/ to /catkin_ws/devel/lib/python2.7/dist-packages/
# so it can be found by python.
set_target_properties(PlanningLibC PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# Mandatory properties.
target_compile_options(PlanningLibC PUBLIC "-std=c++11")

target_include_directories(PlanningLibC PRIVATE ${PLC_DIR}/src)

# Mark executables and/or libraries for installation
install(TARGETS PlanningLibC
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# CLRRTNode Executable.
add_executable(clrrt_node src/clrrt_node.cpp)
set_target_properties(clrrt_node PROPERTIES NO_SYSTEM_FROM_IMPORTED TRUE)
target_compile_options(clrrt_node PUBLIC "-std=c++11")
target_include_directories(clrrt_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS})
target_link_libraries(clrrt_node
  PlanningLibC
  ${catkin_LIBRARIES})
install(TARGETS clrrt_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# RepublishOdomNode Executable.
add_executable(republish_odom_node src/republish_odom_node.cpp)
set_target_properties(republish_odom_node PROPERTIES NO_SYSTEM_FROM_IMPORTED TRUE)
target_compile_options(republish_odom_node PUBLIC "-std=c++11")
target_include_directories(republish_odom_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS})
target_link_libraries(republish_odom_node
  PlanningLibC
  ${catkin_LIBRARIES})
install(TARGETS republish_odom_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# PurePursuitNode Executable.
add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
set_target_properties(pure_pursuit_node PROPERTIES NO_SYSTEM_FROM_IMPORTED TRUE)
target_compile_options(pure_pursuit_node PUBLIC "-std=c++11")
target_include_directories(pure_pursuit_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS})
target_link_libraries(pure_pursuit_node
  PlanningLibC
  ${catkin_LIBRARIES})
install(TARGETS pure_pursuit_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Testing.
enable_testing()

set(TEST_SRCS
  test/run_all_tests.cpp
  test/test_motion_planning_tree.cpp
  test/test_occupancy_grid.cpp
  test/test_clrrt.cpp
  test/test_pure_pursuit.cpp
  test/test_utils.cpp
)

add_executable(PlanningLibC_test ${TEST_SRCS})
set_target_properties(PlanningLibC_test PROPERTIES NO_SYSTEM_FROM_IMPORTED TRUE)
target_include_directories(PlanningLibC_test PRIVATE ${PLC_DIR}/src)
target_include_directories(PlanningLibC_test PRIVATE ${PLC_DIR}/test)
target_link_libraries(PlanningLibC_test PlanningLibC gtest)

add_test(NAME PlanningLibC_test COMMAND PlanningLibC_test --gtest_color=yes)

install(TARGETS PlanningLibC_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
