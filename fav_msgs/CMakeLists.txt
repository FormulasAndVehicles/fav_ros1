cmake_minimum_required(VERSION 3.0.2)
project(fav_msgs)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_message_files(
    FILES
    RangeMeasurement.msg
    RangeMeasurementArray.msg
    ThrusterSetpoint.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    message_runtime
)

