cmake_minimum_required(VERSION 3.10.2)
project(feature_tracker)

#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
#
#find_package(catkin REQUIRED COMPONENTS
#    roscpp
#    std_msgs
#    sensor_msgs
#    cv_bridge
#    camera_model
#    )
#
#find_package(OpenCV REQUIRED)
#
#catkin_package()
#
#include_directories(
#    ${catkin_INCLUDE_DIRS}
#    )
#
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#find_package(Eigen3)
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#  ${EIGEN3_INCLUDE_DIR}
#)

add_library(feature_tracker
#    src/feature_tracker_node.cpp
        src/parameters.cpp
    src/feature_tracker.cpp
    )

target_link_libraries(feature_tracker PUBLIC
    ${jniLibs_DIR}/libopencv_java4.so)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_SOURCE_DIR}/lib-includes
#        ${CMAKE_SOURCE_DIR}/platform
        ${CMAKE_CURRENT_SOURCE_DIR}/../camera_model/include)