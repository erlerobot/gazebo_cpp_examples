cmake_minimum_required(VERSION 2.8.3)
project(ros_erle_rover_tf)
find_package(catkin REQUIRED COMPONENTS tf nav_msgs )

catkin_package(
               CATKIN_DEPENDS nav_msgs)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_executable(tf_sim src/tf_sim.cpp)
add_dependencies(tf_sim nav_msgs)
target_link_libraries(tf_sim ${catkin_LIBRARIES})

add_executable(tf_real src/tf_real.cpp)
add_dependencies(tf_real nav_msgs)
target_link_libraries(tf_real ${catkin_LIBRARIES})


install(TARGETS tf_sim tf_real
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
