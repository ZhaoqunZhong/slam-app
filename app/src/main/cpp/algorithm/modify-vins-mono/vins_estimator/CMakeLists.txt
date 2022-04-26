cmake_minimum_required(VERSION 3.10.2)
project(vins_estimator)

#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

#find_package(catkin REQUIRED COMPONENTS
#    roscpp
#    std_msgs
#    geometry_msgs
#    nav_msgs
#    tf
#    cv_bridge
#    )

#find_package(OpenCV REQUIRED)

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

#find_package(Ceres REQUIRED)

#include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#find_package(Eigen3)
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#  ${EIGEN3_INCLUDE_DIR}
#)

#catkin_package()

add_library(${PROJECT_NAME}
#    src/estimator_node.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/projection_td_factor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
#    src/utility/visualization.cpp
#    src/utility/CameraPoseVisualization.cpp
    src/utility/perf_monitor.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/motion_only_estimator.cpp
    )


target_link_libraries(${PROJECT_NAME} PUBLIC
        ${jniLibs_DIR}/libopencv_java4.so
        ${jniLibs_DIR}/libceres.so)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_SOURCE_DIR}/lib-includes
#        ${CMAKE_SOURCE_DIR}/platform
        )
