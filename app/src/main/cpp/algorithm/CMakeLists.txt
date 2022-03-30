
cmake_minimum_required(VERSION 3.10.2)

project("vio")

add_library(${PROJECT_NAME} SHARED
    vio.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${jniLibs_DIR}/libopencv_java4.so
        ${jniLibs_DIR}/libglog.so
        ${jniLibs_DIR}/libceres.so
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        ../lib-includes)