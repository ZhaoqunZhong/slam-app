# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.10.2)

project(Initializer)

set(opencv_lib "${jniLibs_DIR}/libopencv_java4.so")
set(cere_lib "${jniLibs_DIR}/libceres.so")
set(glog_lib "${jniLibs_DIR}/libglog.so")
set(boost_filesystem_lib "${jniLibs_DIR}/libboost_filesystem-clang-mt-a64-1_74.a")
set(lib_includes "${CMAKE_SOURCE_DIR}/lib-includes")

add_subdirectory(camera_model)
add_subdirectory(feature_tracker)
add_subdirectory(vins_estimator)

add_library(${PROJECT_NAME} SHARED
        slam_api/obslam_api.cpp
        system.cpp
        )

target_link_libraries(${PROJECT_NAME} PUBLIC
        camera_model
        feature_tracker
        vins_estimator
        ${opencv_lib}
        ${cere_lib}
        ${glog_lib}
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        ${lib_includes}
        )