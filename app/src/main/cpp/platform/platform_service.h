//
// Created by zhongzhaoqun on 2021/4/30.
//

#ifndef ORBBEC_VIO_PLATFORM_SERVICE_H
#define ORBBEC_VIO_PLATFORM_SERVICE_H

#include "imu_publisher.h"
#include "cam_publisher.h"
#include "map_drawer.h"
#include "image_preview.h"
// #include "dataset_player.h"

class PlatformService {
public:
    PlatformService(void (*fimu)(imu_msg &), void (*facc)(acc_msg &), void (*fgyro)(gyr_msg &),
                    void (*frgb)(rgb_msg &), void (*f_feature)(double, std::vector<std::pair<size_t, Eigen::VectorXf>> &)) :
                    imu_callback_(fimu), acc_callback_(facc), gyro_callback_(fgyro), rgb_Callback_(frgb), featureCallback_(f_feature) {};

    void start();

    void stop();

    std::unique_ptr<CamPublisher> camPublisher_;
    std::unique_ptr<ImuPublisher> imuPublisher_;

    ImagePreviewer previewer_;
    MapDrawer viz_;

    // std::unique_ptr<DataPlayer> data_player_;
    bool run_offline_;


private:
    void (*imu_callback_)(imu_msg &);

    void (*acc_callback_)(acc_msg &);

    void (*gyro_callback_)(gyr_msg &);

    void (*rgb_Callback_)(rgb_msg &);

    void (*featureCallback_)(double, std::vector<std::pair<size_t, Eigen::VectorXf>> &);

};

#endif //ORBBEC_VIO_PLATFORM_SERVICE_H
