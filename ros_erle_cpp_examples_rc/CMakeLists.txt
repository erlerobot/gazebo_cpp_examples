cmake_minimum_required(VERSION 2.8.3)
project(ros_erle_cpp_examples_rc)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

#
###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    roscpp
    mavros_msgs
)

###########
## Build ##
###########

add_executable(rc
  src/rc.cpp
)

target_link_libraries(rc ${catkin_LIBRARIES})
add_dependencies(rc rc)

install(TARGETS rc 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)