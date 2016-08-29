cmake_minimum_required(VERSION 2.8.3)
project(ros_erle_cpp_teleoperation_erle_rover)

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

add_executable(teleoperation
  src/main.cpp
)
target_link_libraries(teleoperation ${catkin_LIBRARIES})
add_dependencies(teleoperation teleoperation)

add_executable(teleoperation_automatic
  src/main2.cpp
)
target_link_libraries(teleoperation_automatic ${catkin_LIBRARIES})
add_dependencies(teleoperation_automatic teleoperation_automatic)


install(TARGETS teleoperation teleoperation_automatic
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)