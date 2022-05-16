//
// Created by zhong on 2021/11/5.
//
#ifndef VIO_SYSTEM_H
#define VIO_SYSTEM_H

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <fstream>
#include <condition_variable>
#include "vins_estimator/src/estimator.h"
#include "vins_estimator/src/parameters.h"
#include "feature_tracker/src/feature_tracker.h"
#include "feature_tracker/src/parameters.h"
#include "slam_api/obslam_api.h"

/// Msg definition for sensor queue
struct IMU_MSG
{
    double header;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
};
typedef std::shared_ptr<IMU_MSG const> ImuConstPtr;

struct IMG_MSG {
    double header;
    vector<Vector3d> points;
    vector<int> id_of_point;
    vector<float> u_of_point;
    vector<float> v_of_point;
    vector<float> velocity_x_of_point;
    vector<float> velocity_y_of_point;
};
typedef std::shared_ptr <IMG_MSG const > ImgConstPtr;


class System
{
public:
    System(std::string sConfig_files);

    ~System();

    void subImageData(double dStampSec, cv::Mat img);

    void subImuData(double dStampSec, const Eigen::Vector3d &vGyr,
                    const Eigen::Vector3d &vAcc);

    // thread: visual-inertial odometry
    void process();
    std::thread vi_th_;

    FeatureTracker trackerData[feature_tracker::NUM_OF_CAM];


    bool bStart_backend, process_exited;

    //feature tracker
    std::vector<uchar> r_status;
    std::vector<float> r_err;
    // std::queue<ImageConstPtr> img_buf;


    double last_publish_time;
    bool first_image_flag = true;
    double last_image_time = 0;

    //estimator
    Estimator estimator;

    double current_time = -1;
    std::queue<ImuConstPtr> imu_buf;
    std::queue<ImgConstPtr> feature_buf;
    // std::queue<PointCloudConstPtr> relo_buf;
    int sum_of_wait = 0;

    std::mutex m_buf;
    std::mutex m_state;
    std::mutex i_buf;
    std::mutex m_estimator;
    std::condition_variable con;

    double latest_time;
    Eigen::Vector3d tmp_P;
    Eigen::Quaterniond tmp_Q;
    Eigen::Vector3d tmp_V;
    Eigen::Vector3d tmp_Ba;
    Eigen::Vector3d tmp_Bg;
    bool first_image = 0;
    double last_imu_t = 0;
    // std::ofstream ofs_pose;
    // std::vector<Eigen::Vector3d> vPath_to_draw;

    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> getMeasurements();

    /// motion only estimation
    void motionOnlyProcess();
    std::thread mo_th_;
    bool mo_estimate_start = true, mo_estimate_exited = false;
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> getMoMeasurements();
    std::queue<ImgConstPtr> mo_img_buf_;
    std::queue<ImuConstPtr> mo_imu_buf_;
    std::mutex mo_buf_mtx_;
    std::condition_variable mo_buf_con_;

    Eigen::Matrix3d Rwbs[vins_estimator::WINDOW_SIZE + 1];
    Eigen::Vector3d Pwbs[vins_estimator::WINDOW_SIZE + 1];
    Eigen::Vector3d Vb_end, Ba_end, Bg_end;
    double ts_end, td;
    unordered_map<int, Vector3d> feature_end;

    bool init_imu = true;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyr_0;

    //slam api
    slam_pose_callback poseCallback_;
    image_process_callback imageProcessCallback_;
    pthread_mutex_t img_pro_ckb_mtx_ = PTHREAD_MUTEX_INITIALIZER;
};


#endif //VIO_SYSTEM_H
