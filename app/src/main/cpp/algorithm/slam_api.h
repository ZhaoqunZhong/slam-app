//
// Created by zhongzhaoqun on 2022/3/2.
//

#ifndef SLAM_API_H
#define SLAM_API_H

#include <string>

/*
 * I think for early stage of development, it's better to utilize popular
 * third-party libs as data interfaces.
 */
#include "opencv2/opencv.hpp"
// #include "Eigen/Core"
#include "Eigen/Eigen"

#define SLAM_HANDLE void*

void slam_create(SLAM_HANDLE slam_handle);

/*
 * Volcabulary only accept .bin file.
 */
void slam_add_volcabulary(SLAM_HANDLE slam_handle, std::string val_path);

typedef enum {
    RAD_TAN = 0,
    FISH_EYE
} slam_camera_type;
void slam_add_camera(SLAM_HANDLE slam_handle, slam_camera_type cameraType, Eigen::Matrix3d K,
                     std::vector<double> D, double readout_time_prior);

void slam_add_imu(SLAM_HANDLE slam_handle, double acc_n, double acc_w, double gyr_n, double gyr_w,
                  Eigen::Vector3d acc_bias_prior, Eigen::Vector3d gyr_bias_prior);

void slam_add_vi_extrinsic_prior(SLAM_HANDLE slam_handle, Eigen::Matrix4d Tic_prior,
                                 double td_delta_cam_prior);

void slam_start(SLAM_HANDLE slam_handle);

void slam_feed_image(SLAM_HANDLE slam_handle, uint64_t ts, cv::Mat img);

void slam_feed_imu(SLAM_HANDLE slam_handle, uint64_t ts, Eigen::Vector3d acc, Eigen::Vector3d gyr);

void slam_feed_acc(SLAM_HANDLE slam_handle, uint64_t ts, Eigen::Vector3d acc);

void slam_feed_gyr(SLAM_HANDLE slam_handle, uint64_t ts, Eigen::Vector3d gyr);

void slam_stop(SLAM_HANDLE slam_handle);

void slam_release(SLAM_HANDLE slam_handle);

typedef enum {
    NOT_STARTED = 0,
    STARTING,
    INITIALIZING,
    TRACKING,
    STOPING
} slam_status;
slam_status slam_check_status(SLAM_HANDLE slam_handle);

typedef enum {
    PRE_ACCURATE = 0,
    ACCURATE
} slam_pose_type;
/*
 * Pose is T_world_to_imu
 */
struct slam_pose {
    uint64_t ts;
    Eigen::Vector3d tran;
    Eigen::Quaterniond rot;
    Eigen::Matrix4d T;
    slam_pose_type type;
};
/*
 * The pose callback is blocking function.
 */
typedef void (*slam_pose_callback)(slam_pose* pose);
void slam_register_pose_callback(SLAM_HANDLE slam_handle, slam_pose_callback);



/*-------------------private API--------------------*/
/*
 * The image process callback is non-blocking function. The client side needs to
 * check the timestamp disorder by themselves. Although the possibility of disorder
 * is pretty small.
 */
struct slam_processed_image {
    uint64_t ts;
    cv::Mat image;
    double processing_time;
};
typedef void (*image_process_callback)(slam_processed_image *image);
void slam_register_image_process_callback(SLAM_HANDLE slam_handle, image_process_callback);

/*
 * Inorder to support this keyFrame interface, the slam module has to maintain a number of image buffers,
 * instead of ditching the images as soon as processing is done.
 *
 * This interface is also non-blocking.
 *
 * This interface will return a frame when it's first categorized as keyFrame,
 * even if it would be culled later.
 */
struct slam_keyFrame {
    uint64_t ts;
    cv::Mat image;
};
typedef void (*keyFame_callback)(slam_keyFrame *frame);
void slam_register_keyFrame_callback(SLAM_HANDLE slam_handle, keyFame_callback);

/*
 * The slam module has total control over the format and content of the config file.
 */
void slam_add_config_path(SLAM_HANDLE slam_handle, std::string config);

void slam_add_result_path(SLAM_HANDLE slam_handle, std::string result_path);

#endif //SLAM_API_H
