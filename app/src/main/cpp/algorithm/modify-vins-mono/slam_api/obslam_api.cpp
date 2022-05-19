//
// Created by zyn on 2021/11/2.
//


#include "obslam_api.h"
#include "../system.h"

#ifdef __cplusplus
extern "C" {
#endif

void initial_callback(std::vector<double> &ts_, std::vector<Eigen::Vector3d> &Ps_,
                             std::vector<Eigen::Vector3d> &Vs_, std::vector<Eigen::Matrix3d> &Qs_, Eigen::Vector3d ba_,
                             Eigen::Vector3d g, Eigen::Vector3d bg_, std::vector<shared_ptr<IntegrationBase>> &pre_integrations_,
                             std::map<uint, std::vector<std::pair<uint, std::array<double, 6>>>> &features_, std::map<uint, Eigen::Vector3d> &world_pts_) {
    LOG(WARNING) << "DEBUG initial_callback --------------" << "";
    // LOG(WARNING) << "DEBUG world pts size " << world_pts_.size();
    std::map<uint, std::vector<std::pair<uint, std::array<double, 6>>>>::iterator feature_iter;
    for (feature_iter = features_.begin(); feature_iter != features_.end(); feature_iter++) {
        // LOG(WARNING) << "DEBUG f_id " << feature_iter->first;
        for (auto & pair : feature_iter->second) {
            // LOG(WARNING) << "DEBUG frame_id " << pair.first;
            Eigen::Map<Eigen::Matrix<double, 1, 6>> print(pair.second.data());
            // LOG(WARNING) << "DEBUG uv norm_xy v_xy " << print;
        }
        // LOG(WARNING) << "DEBUG world pt " << world_pts_[feature_iter->first].transpose();
        // LOG(WARNING) << "DEBUG  " << "----------------------------";
    }
/*    for (auto & integration : pre_integrations_) {
        LOG(WARNING) << "DEBUG integration delta p " << integration->delta_p.transpose();
    }*/

    LOG(WARNING) << "DEBUG gravity " << g.transpose();
    LOG(WARNING) << "DEBUG ba " << ba_.transpose() << " norm " << ba_.norm();
    LOG(WARNING) << "DEBUG bg " << bg_.transpose() << " norm " << bg_.norm();
}

void create_slam(SLAM_HANDLE *slam_handle, const char *volcabulary_path, const char *config_path)
{
    Eigen::Matrix3d Ric;
    Ric << 0.00160563, -0.99999611, -0.00228006,
            -0.99999675, -0.00160111, -0.00198450,
            0.00198084, 0.00228324, -0.99999543;

    *slam_handle = new System(480, 640,
                              493.92608247343867, 493.94340645062755, 317.7858212847334, 242.05056441352974,
                              RAD_TAN, 0.063162160702101205, -0.090016428378378005, 0.00013963678674432469, 0.00071378641002039535,
                              0.02, 0.012491, 0.001563, 0.003083, 0.0001,
                              Ric, Eigen::Vector3d(0.01924729, 0.00160597, -0.00989951),
                              0.02, 9.8, initial_callback);

/*    ((System*)slam_handle)->addCalibrationParas(480, 640,
                                                493.92608247343867, 493.94340645062755, 317.7858212847334, 242.05056441352974,
                                                RAD_TAN, 0.063162160702101205, -0.090016428378378005, 0.00013963678674432469, 0.00071378641002039535,
                                                0.02, 0.012491, 0.001563, 0.003083, 0.0001,
                                                Ric, Eigen::Vector3d(0.01924729, 0.00160597, -0.00989951),
                                                0.02, 9.8);*/
    // ((System*)slam_handle)->registerInitialCallback(initial_callback);
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
/*    ((System*)slam_handle)->vi_th_ = std::thread(&System::process, ((System*)slam_handle));
    ((System*)slam_handle)->vi_th_.detach();
    ((System*)slam_handle)->mo_th_ = std::thread(&System::motionOnlyProcess, ((System*)slam_handle));
    ((System*)slam_handle)->mo_th_.detach();*/
    return 1;
}

int32_t stop_slam(SLAM_HANDLE slam_handle)
{
/*    ((System*)slam_handle)->bStart_backend = false;
    ((System*)slam_handle)->con.notify_one();
    ((System*)slam_handle)->mo_estimate_start = false;
    ((System*)slam_handle)->mo_buf_con_.notify_one();
    while (!((System*)slam_handle)->process_exited || !((System*)slam_handle)->mo_estimate_exited) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }*/
    return 1;
}

int32_t slam_add_image(SLAM_HANDLE slam_handle, uint64_t timestamp,
                                    uint32_t width, uint32_t height, uint8_t *data)
{
    double imgTimeStamp = timestamp/1e9;
    cv::Mat img(height, width, CV_8UC1, data);
    ((System*)slam_handle)->subImageData(imgTimeStamp, img.clone());
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
    // ((System*)slam_handle)->poseCallback_ = cbkfun;
}

void slam_register_image_process_callback(SLAM_HANDLE slam_handle, image_process_callback cbkfun) {
    ((System*)slam_handle)->imageProcessCallback_ = cbkfun;
}

void slam_register_keyFrame_callback(SLAM_HANDLE slam_handle, keyFame_callback cbkfun) {

}

#ifdef __cplusplus
}
#endif
