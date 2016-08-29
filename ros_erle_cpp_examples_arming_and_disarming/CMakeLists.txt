cmake_minimum_required(VERSION 2.8.3)
project(ros_erle_cpp_examples_arming_and_disarming)

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

add_executable(arming
  src/arming.cpp
)

target_link_libraries(arming ${catkin_LIBRARIES})
add_dependencies(arming arming)


add_executable(disarming
  src/disarming.cpp
)

target_link_libraries(disarming ${catkin_LIBRARIES})
add_dependencies(disarming disarming)

install(TARGETS arming disarming 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)