cmake_minimum_required(VERSION 2.8.3)
project(walking_person_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(roscpp REQUIRED)
find_package(gazebo REQUIRED)
## find_package(glog 0.3.4 REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(./include)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(walking_person_plugin SHARED src/ActorPlugin.cc)
target_link_libraries(${PROJECT_NAME}  ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} glog)
set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
