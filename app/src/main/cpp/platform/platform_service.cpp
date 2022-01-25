//
// Created by zhongzhaoqun on 2021/4/30.
//

#include "platform_service.h"
#include "glog/logging.h"

void PlatformService::start() {
    std::string file_name = "/sdcard/orbbec-vio-data/config.yaml";
    try {
        cv::FileStorage fs(file_name, cv::FileStorage::READ);
        run_offline_ = static_cast<int>(fs["run_offline"]);
        fs.release();
    }
    catch (const cv::Exception& e) {
        LOG(ERROR) << e.what();
    }

    if (run_offline_) {
        // data_player_ = std::make_unique<DataPlayer>(rgb_Callback_, imu_callback_, featureCallback_);
        // data_player_->start();
    } else {
        camPublisher_ = std::make_unique<CamPublisher>(rgb_Callback_,nullptr);
        imuPublisher_ = std::make_unique<ImuPublisher>(imu_callback_, acc_callback_, gyro_callback_);
        camPublisher_->start();
        imuPublisher_->start();
    }

    previewer_.start();
}

void PlatformService::stop() {
    if (run_offline_) {
        // data_player_->stop();
    } else {
        camPublisher_->stop();
        imuPublisher_->stop();
    }
    clearVisualizationBuffers();
    previewer_.stop();
}


