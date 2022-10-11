cmake_minimum_required(VERSION 3.0.2)
project(fav_gazebo_plugins)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  roscpp
  fav_msgs
)

add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS 
      roscpp 
      gazebo_ros
)

function(add_plugin plugin_name)
  add_library(${plugin_name} SHARED src/${plugin_name}/${plugin_name}.cpp)
  add_dependencies(${plugin_name} ${catkin_EXPORTED_TARGETS} fav_msgs_generate_messages_cpp)
  target_link_libraries(${plugin_name} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
endfunction()

add_plugin(barometer)
add_plugin(range_sensor)
add_plugin(hydrodynamics)
add_plugin(buoyancy)
add_plugin(thruster)

