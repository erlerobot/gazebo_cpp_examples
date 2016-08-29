cmake_minimum_required(VERSION 2.8.3)
project(ros_erle_cpp_examples_change_mode)

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

add_executable(changemode
  src/changemode.cpp
)

target_link_libraries(changemode ${catkin_LIBRARIES})
add_dependencies(changemode changemode)

install(TARGETS changemode 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)