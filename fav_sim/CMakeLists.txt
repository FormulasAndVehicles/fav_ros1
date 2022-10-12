cmake_minimum_required(VERSION 3.0.2)
project(fav_sim)


find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  gazebo_ros
  gazebo_plugins
  fav_msgs
  fav_gazebo_plugins
)

find_package(gazebo REQUIRED)

add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  DEPENDS
      gazebo_ros
      roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(esc_commander src/esc_commander_node.cpp)
target_link_libraries(esc_commander ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(esc_commander fav_msgs_generate_messages_cpp)


