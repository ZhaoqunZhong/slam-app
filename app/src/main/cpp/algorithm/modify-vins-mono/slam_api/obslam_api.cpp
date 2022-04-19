//
// Created by zyn on 2021/11/2.
//


#include "obslam_api.h"
#include "../system.h"

#ifdef __cplusplus
extern "C" {
#endif


void create_slam(SLAM_HANDLE *slam_handle, const char *volcabulary_path, const char *config_path)
{
    *slam_handle = new System(std::string(config_path));
}

void release_slam(SLAM_HANDLE slam_handle)
{
    if(slam_handle)
    {
        delete (System*)slam_handle;
    }
}

int32_t start_slam(SLAM_HANDLE slam_handle)
{
    ((System*)slam_handle)->vi_th_ = std::thread(&System::process, ((System*)slam_handle));
    ((System*)slam_handle)->vi_th_.detach();
    ((System*)slam_handle)->mo_th_ = std::thread(&System::motionOnlyProcess, ((System*)slam_handle));
    ((System*)slam_handle)->mo_th_.detach();
    return 1;
}

int32_t stop_slam(SLAM_HANDLE slam_handle)
{
    ((System*)slam_handle)->bStart_backend = false;
    ((System*)slam_handle)->con.notify_one();
    ((System*)slam_handle)->mo_estimate_start = false;
    ((System*)slam_handle)->mo_buf_con_.notify_one();
    while (!((System*)slam_handle)->process_exited || !((System*)slam_handle)->mo_estimate_exited) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 1;
}

int32_t slam_add_image(SLAM_HANDLE slam_handle, uint64_t timestamp,
                                    uint32_t width, uint32_t height, uint8_t *data)
{
    double imgTimeStamp = timestamp/1e9;
    cv::Mat img(height, width, CV_8UC1, data);
    ((System*)slam_handle)->subImageData(imgTimeStamp, img);
    return 1;
}

int32_t slam_add_imu(SLAM_HANDLE slam_handle, uint64_t timestamp,
                                  double linear_acceleration_x, double linear_acceleration_y, double linear_acceleration_z,
                                  double angular_velocity_x, double angular_velocity_y, double angular_velocity_z)
{
    double imuTimeStamp = timestamp/1e9;
    Eigen::Vector3d gyr{angular_velocity_x, angular_velocity_y, angular_velocity_z};
    Eigen::Vector3d acc{linear_acceleration_x, linear_acceleration_y, linear_acceleration_z};
    ((System*)slam_handle)->subImuData(imuTimeStamp, gyr, acc);
    return 1;
}

int32_t slam_add_accel(SLAM_HANDLE slam_handle, uint64_t timestamp,
                                    double linear_acceleration_x, double linear_acceleration_y, double linear_acceleration_z)
{

    return 1;
}

int32_t slam_add_gyro(SLAM_HANDLE slam_handle, uint64_t timestamp,
                                   double angular_velocity_x, double angular_velocity_y, double angular_velocity_z)
{

    return 1;
}

void slam_register_status_callback(SLAM_HANDLE slam_handle, slam_status_callback cbkfun) {
    ((System*)slam_handle)->estimator.statusCallback_ = cbkfun;
}

void slam_register_pose_callback(SLAM_HANDLE slam_handle, slam_pose_callback cbkfun) {
    ((System*)slam_handle)->poseCallback_ = cbkfun;
}

void slam_register_image_process_callback(SLAM_HANDLE slam_handle, image_process_callback cbkfun) {
    ((System*)slam_handle)->imageProcessCallback_ = cbkfun;
}

void slam_register_keyFrame_callback(SLAM_HANDLE slam_handle, keyFame_callback cbkfun) {

}

#ifdef __cplusplus
}
#endif
