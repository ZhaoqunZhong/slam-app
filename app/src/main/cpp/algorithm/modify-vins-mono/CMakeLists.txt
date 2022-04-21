# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.10.2)

project(vins_mono)

add_subdirectory(camera_model)
add_subdirectory(feature_tracker)
add_subdirectory(vins_estimator)

add_library(${PROJECT_NAME} SHARED
        slam_api/obslam_api.cpp
        system.cpp
        #debug
#        ${CMAKE_SOURCE_DIR}/platform/native_debug.cpp
        )

target_link_libraries(${PROJECT_NAME} PUBLIC
        camera_model
        feature_tracker
        vins_estimator
        ${jniLibs_DIR}/libopencv_java4.so
        ${jniLibs_DIR}/libglog.so
        ${jniLibs_DIR}/libceres.so
        #debug
#        log
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_SOURCE_DIR}/lib-includes
        #debug
#        ${CMAKE_SOURCE_DIR}/platform
        )