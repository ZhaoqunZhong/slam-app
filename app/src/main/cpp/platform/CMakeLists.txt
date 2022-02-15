cmake_minimum_required(VERSION 3.10.2)

project(platform)

#add_subdirectory(rosbag)

add_library(platform-lib SHARED
        ndk_camera_utils/ndk_utils.cpp
        imu_publisher.cpp
        cam_publisher.cpp
        native_debug.cpp
        perf_monitor.cpp
#        data_dumper.cpp
        image_preview.cpp
        platform_service.cpp
        map_drawer.cpp
#        dataset_player.cpp
#        openvis_simulation/BsplineSE3.cpp
#        openvis_simulation/Simulator.cpp
        camera_model.cpp
#        RosbagPacker.cpp
        )

target_include_directories(platform-lib PUBLIC
        ${CMAKE_SOURCE_DIR}/lib-includes
        )

target_link_libraries(platform-lib
        log
        android
        GLESv3
        camera2ndk
        mediandk
        ${jniLibs_DIR}/libopencv_java4.so
        ${jniLibs_DIR}/libglog.so
#        RosbagStorage
        ${jniLibs_DIR}/libRosbagStorage.so
        ${jniLibs_DIR}/libCvBridgeSimple.so
        ${jniLibs_DIR}/libRosTime.so
        )

#add_custom_command(TARGET platform-lib POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:platform-lib> ${jniLibs_DIR}
#        )



