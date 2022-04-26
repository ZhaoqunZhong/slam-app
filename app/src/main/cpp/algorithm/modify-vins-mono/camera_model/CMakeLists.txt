cmake_minimum_required(VERSION 3.10.2)
project(camera_model)

#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

#find_package(catkin REQUIRED COMPONENTS
#    roscpp
#    std_msgs
#    )

#find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
#include_directories(${Boost_INCLUDE_DIRS})

#find_package(OpenCV REQUIRED)

# set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
#find_package(Ceres REQUIRED)
#include_directories(${CERES_INCLUDE_DIRS})


#catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES camera_model
#    CATKIN_DEPENDS roscpp std_msgs
##    DEPENDS system_lib
#    )

#include_directories(
#    ${catkin_INCLUDE_DIRS}
#    )

#include_directories("include")


#add_executable(Calibration
#    src/intrinsic_calib.cc
#    src/chessboard/Chessboard.cc
#    src/calib/CameraCalibration.cc
#    src/camera_models/Camera.cc
#    src/camera_models/CameraFactory.cc
#    src/camera_models/CostFunctionFactory.cc
#    src/camera_models/PinholeCamera.cc
#    src/camera_models/CataCamera.cc
#    src/camera_models/EquidistantCamera.cc
#    src/camera_models/ScaramuzzaCamera.cc
#    src/sparse_graph/Transform.cc
#    src/gpl/gpl.cc
#    src/gpl/EigenQuaternionParameterization.cc)

add_library(${PROJECT_NAME}
#    src/chessboard/Chessboard.cc
#    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

#target_link_libraries(Calibration ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${jniLibs_DIR}/libopencv_java4.so
        ${jniLibs_DIR}/libboost_filesystem-clang-mt-a64-1_74.a
        ${jniLibs_DIR}/libceres.so
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_SOURCE_DIR}/lib-includes
#        ${CMAKE_SOURCE_DIR}/platform
        ${CMAKE_CURRENT_SOURCE_DIR}/include)
