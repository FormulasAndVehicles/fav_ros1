cmake_minimum_required(VERSION 3.0.2)
project(fav_sim)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  fav_msgs
  fav_gazebo_plugins
)

add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  DEPENDS
      gazebo_ros
)
