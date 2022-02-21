
# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.10.2)

project("slam_app")

set(jniLibs_DIR ${CMAKE_SOURCE_DIR}/../jniLibs/${ANDROID_ABI})

#----------------------rosbag--------------------------
#add_subdirectory(platform/rosbag)

#----------------------------data record--------------------------------
add_library(data_record SHARED
        data_record.cpp
        platform/cam_publisher.cpp
        platform/ndk_camera_utils/ndk_utils.cpp
        platform/imu_publisher.cpp
        platform/image_preview.cpp
        platform/native_debug.cpp
        platform/perf_monitor.cpp
        platform/data_dumper.cpp
        platform/RosbagPacker.cpp)

target_link_libraries(data_record PRIVATE
        android
        camera2ndk
        log
        mediandk
        ${jniLibs_DIR}/libopencv_java4.so
        ${jniLibs_DIR}/libglog.so
        ${jniLibs_DIR}/librosbag.so
)

target_include_directories(data_record PRIVATE
        lib-includes
        platform/rosbag/include)

