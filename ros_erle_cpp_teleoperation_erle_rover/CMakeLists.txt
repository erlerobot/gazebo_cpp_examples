cmake_minimum_required(VERSION 2.8.3)
project(ros_erle_cpp_teleoperation_erle_rover)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

#
###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    roscpp
    mav_msgs
)

###########
## Build ##
###########

add_executable(teleoperation
  src/main.cpp
)

target_link_libraries(teleoperation ${catkin_LIBRARIES})
add_dependencies(teleoperation teleoperation)

install(TARGETS teleoperation 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)