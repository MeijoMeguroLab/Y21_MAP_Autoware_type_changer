cmake_minimum_required(VERSION 2.8.3)
project(type_changer_real)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  sensor_msgs
  velodyne_msgs
)

#
catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs velodyne_msgs
        
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(type_changer_real src/type_changer_real.cpp )
target_link_libraries (type_changer_real ${catkin_LIBRARIES})
add_dependencies(type_changer_real   ${catkin_EXPORTED_TARGETS})

install(TARGETS
type_changer_real
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)