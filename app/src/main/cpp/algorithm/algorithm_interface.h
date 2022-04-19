#ifndef VID_ALGO_ENTRY_H
#define VID_ALGO_ENTRY_H

/*
 * The purpose of this intermediate level is
 */

#include "imu_publisher.h"
#include "cam_publisher.h"

class AlgorithmInterface {
public:
    AlgorithmInterface() {};

    ~AlgorithmInterface() {};

    void start();
    void runAlgorithm(); //start() will call runAlgorithm() through a thread
    pthread_t algo_t_;

    void stop();


    void rgbCallback(rgb_msg &);

    void accCallback(acc_msg &);

    void gyrCallback(gyr_msg &);

    void imuCallback(imu_msg &);

    void featureCallback(double ts, std::vector<std::pair<size_t, Eigen::VectorXf>> &);

    bool algorithm_on_ = false;

    int run_offline_;
    std::string bag_name_;

};

#endif //VID_ALGO_ENTRY_H
