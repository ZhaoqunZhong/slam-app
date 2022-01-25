cmake_minimum_required(VERSION 3.10.2)

project(rosbag)

# RosTime
add_library(RosTime SHARED
        src/rostime/rate.cpp
        src/rostime/time.cpp
        src/rostime/duration.cpp
        )

target_include_directories(RosTime
        PUBLIC
        ${CMAKE_SOURCE_DIR}/lib-includes
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        )

# CvBridgeSimple
add_library(CvBridgeSimple SHARED
        src/cv_bridge_simple.cpp)
target_include_directories(CvBridgeSimple
        PUBLIC
        ${CMAKE_SOURCE_DIR}/lib-includes
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        )
target_link_libraries(CvBridgeSimple
        PUBLIC
        ${jniLibs_DIR}/libopencv_java4.so
        )

# RosbagStorage
FILE(GLOB_RECURSE RosbagStorage_SOURCES
        src/RosbagStorage/*.c
        src/RosbagStorage/*.cpp
        )

add_library(RosbagStorage SHARED
        ${RosbagStorage_SOURCES}
        )

target_include_directories(RosbagStorage
        PUBLIC
        ${CMAKE_SOURCE_DIR}/lib-includes
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        )

target_link_libraries(RosbagStorage
        PUBLIC
        RosTime CvBridgeSimple
        ${jniLibs_DIR}/libglog.so ${jniLibs_DIR}/libopencv_java4.so
        )
