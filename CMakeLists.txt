cmake_minimum_required(VERSION 2.8.3)
project(ros_erle_pattern_follower)

set(CMAKE_PREFIX_PATH "/usr/local/lib")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  mavros
 )

find_package(OpenCV REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} 
/usr/local/include/)

#
###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    roscpp
    image_transport
    cv_bridge
    mavros
)


###########
## Build ##
###########

add_executable(${PROJECT_NAME}
  src/image_subscriber.cpp
)
#
#
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} 	${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} /usr/local/lib/libaruco.so )
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME})


