cmake_minimum_required(VERSION 3.10.2)

project(rosbag)

FILE(GLOB_RECURSE RosbagStorage_SOURCES
        src/RosbagStorage/*.c
        src/RosbagStorage/*.cpp
        )
add_library(rosbag SHARED
        src/rostime/rate.cpp
        src/rostime/time.cpp
        src/rostime/duration.cpp
        src/cv_bridge_simple.cpp
        ${RosbagStorage_SOURCES}
        )
target_include_directories(rosbag
        PRIVATE
        ${CMAKE_SOURCE_DIR}/lib-includes
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        )
target_link_libraries(rosbag
        PRIVATE
        ${jniLibs_DIR}/libglog.so
        ${jniLibs_DIR}/libopencv_java4.so
        )
