cmake_minimum_required(VERSION 2.8.3)
project(type_changer_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(type_change_sim src/type_change_sim.cpp )
target_link_libraries (type_change_sim ${catkin_LIBRARIES})
add_dependencies(type_change_sim ${catkin_EXPORTED_TARGETS})
