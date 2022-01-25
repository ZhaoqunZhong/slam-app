#ifndef VID_PC_DATASET_PLAYER_H
#define VID_PC_DATASET_PLAYER_H

#include <string>
#include <queue>
#include "cam_publisher.h"
#include "imu_publisher.h"
#include "../openvis_simulation/Simulator.h"
#include <unordered_set>

class DataPlayer {
public:
    DataPlayer(void (*f_rgb) (rgb_msg &), void (*f_imu)(imu_msg &), void (*f_feature)(double ts, std::vector<std::pair<size_t, Eigen::VectorXf>> &))
    : rgbCallback_(f_rgb), imuCallback_(f_imu), featureCallback_(f_feature){};
    ~DataPlayer(){};
    void start();
    void stop();
    void playRgb();
    void playImu();
    void simulate();
    std::shared_ptr<ov_msckf::Simulator> sim_;
private:
    pthread_t rgb_t, imu_t, simulation_t;
    bool player_on_ = false, rgb_on_=false, imu_on_=false;
    float rgb_fps_ = 30,  imu_fps_ = 420;
    void (*rgbCallback_) (rgb_msg &);
    void (*imuCallback_) (imu_msg &);
    std::string imu_file_format_;
    std::string data_folder_;
    std::queue<uint64> rgb_ts_;
    std::queue<imu_msg> imu_q_;

    void (*featureCallback_)(double ts, std::vector<std::pair<size_t, Eigen::VectorXf>> &);
    bool feature_instead_of_image_;

    std::unordered_set<size_t> pt_ids_;
};

#endif //VID_PC_DATASET_PLAYER_H
