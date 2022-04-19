
cmake_minimum_required(VERSION 3.10.2)

project("slam_algorithm")

add_library(${PROJECT_NAME} SHARED
        algorithm_interface.cpp)

add_subdirectory(modify-vins-mono)

target_link_libraries(${PROJECT_NAME} PRIVATE
#        ${jniLibs_DIR}/libopencv_java4.so
#        ${jniLibs_DIR}/libglog.so
#        ${jniLibs_DIR}/libceres.so
        ${jniLibs_DIR}/librosbag.so
        vins_mono
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_SOURCE_DIR}/lib-includes
        ${CMAKE_SOURCE_DIR}/platform
        ${CMAKE_SOURCE_DIR}/platform/rosbag/include)