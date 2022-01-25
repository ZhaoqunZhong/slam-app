#include "glog/logging.h"
#include "dataset_player.h"
#include <pthread.h>
#include <unistd.h>
#include <fstream>
#include <queue>
#include "opencv2/imgcodecs.hpp"
#include "openvis_simulation/quat_ops.h"

void* rgbRunner(void* ptr) {
    DataPlayer* classptr = (DataPlayer*)ptr;
    classptr->playRgb();
    return nullptr;
}
void* simulationRunner(void* ptr) {
    DataPlayer* classptr = (DataPlayer*)ptr;
    classptr->simulate();
    return nullptr;
}
void* imuRunner(void* ptr) {
    DataPlayer* classptr = (DataPlayer*)ptr;
    classptr->playImu();
    return nullptr;
}


void DataPlayer::start() {
    cv::FileStorage fs(app_internal_storage + "platform/S9config.yaml", cv::FileStorage::READ);
    rgb_fps_ = static_cast<float>(fs["rgb_play_fps"]);
    imu_fps_ = static_cast<float>(fs["imu_play_fps"]);
    imu_file_format_ = ".txt";
    data_folder_ = "/sdcard/orbbec-vio-data/offline_data/";
    feature_instead_of_image_ = static_cast<int>(fs["feature_instead_of_image"]);
    fs.release();

    if (feature_instead_of_image_ == 1) {
        sim_ = std::make_shared<ov_msckf::Simulator>(app_internal_storage);
        player_on_ = true;
        pthread_create(&simulation_t, nullptr, simulationRunner, this);
    } else {
        std::ifstream f_ts(app_internal_storage + data_folder_ + "rgb_images/timeStamp.txt");
        while (!f_ts.eof()) {
            uint64 ts;
            f_ts >> ts;
            rgb_ts_.push(ts);
        }
        f_ts.close();

        std::ifstream imu_f(app_internal_storage + data_folder_ + "imu" + imu_file_format_);
        std::queue<imu_msg> imu_q;
        while(!imu_f.eof())
        {
            std::string s;
            getline(imu_f,s);
            if(!s.empty())
            {
                std::string item;
                size_t pos = 0;
                double data[7];
                int count = 0;
                while ((pos = s.find(',')) != std::string::npos) {
                    item = s.substr(0, pos);
                    data[count++] = stod(item);
                    s.erase(0, pos + 1);
                }
                item = s.substr(0, pos);
                data[6] = stod(item);
                imu_msg msg;
                msg.ts = data[0];
                msg.acc_part.ts = msg.ts;
                msg.gyro_part.ts = msg.ts;
                msg.acc_part.ax = data[1];
                msg.acc_part.ay = data[2];
                msg.acc_part.az = data[3];
                msg.gyro_part.rx = data[4];
                msg.gyro_part.ry = data[5];
                msg.gyro_part.rz = data[6];
                imu_q.push(msg);
            }
        }
        imu_f.close();
        player_on_ = true;
        //pthread_create(&rgb_t, nullptr, rgbRunner, this);
        //pthread_create(&imu_t, nullptr, imuRunner, this);
    }
}

void DataPlayer::stop() {
    player_on_ = false;
    if (feature_instead_of_image_ == 1) {
        pthread_join(simulation_t, nullptr);
    } else {
        //pthread_join(rgb_t, nullptr);
        //pthread_join(imu_t, nullptr);
    }
}

void DataPlayer::playRgb() {
    while (player_on_ && !rgb_ts_.empty()) {
        static uint64 data_time_start = rgb_ts_.front();
        uint64 cur_ts = rgb_ts_.front();
        std::string cur_img = app_internal_storage + data_folder_ + "rgb_images/data/" + std::to_string(cur_ts) + ".png";
        rgb_ts_.pop();
        rgb_msg msg;
        msg.ts = cur_ts / 1e9;
        msg.yMat = cv::imread(cur_img, cv::IMREAD_GRAYSCALE);
        static TimeLagMeasurer timer;
        timer.lagFromLastSecond();
        rgbCallback_(msg);
        double time_cost = timer.lagFromLastSecond() * 1000;
        double cur_data_time = data_time_start + timer.lagFromStartSecond()*1e9;
        if (time_cost < 1000.0 / rgb_fps_) {
            double time_sleep = 1000.0 / rgb_fps_ - time_cost;
            useconds_t thread_sleep_time = static_cast<useconds_t>(time_sleep*1000);
            usleep(thread_sleep_time);
        } else {
            cur_ts = rgb_ts_.front();
            while (cur_data_time - cur_ts >= 1e9 / rgb_fps_ && !rgb_ts_.empty()) {
                rgb_ts_.pop();
                cur_ts = rgb_ts_.front();
            }
        }
    }
}

void DataPlayer::simulate() {
    pthread_cond_wait(&data_play_cond, &data_play_mtx);

    while (player_on_ && sim_->ok()) {
        // IMU: get the next simulated IMU measurement if we have it
        double time_imu;
        Eigen::Vector3d wm, am;
        bool hasimu = sim_->get_next_imu(time_imu, wm, am);
        if(hasimu) {
            imu_msg imu;
            imu.ts = time_imu * 1e9; // raw message time in nano second
            imu.acc_part.ts = imu.ts;
            imu.acc_part.ax = am.x();
            imu.acc_part.ay = am.y();
            imu.acc_part.az = am.z();
            imu.gyro_part.ts = imu.ts;
            imu.gyro_part.rx = wm.x();
            imu.gyro_part.ry = wm.y();
            imu.gyro_part.rz = wm.z();
            // LOG(INFO) << "acc " << am.transpose() << " gyr " << wm.transpose();
            // LOG(INFO) << "acc norm " << am.norm();
            imuCallback_(imu);
        }
        // CAM: get the next simulated camera uv measurements if we have them
        double time_cam;
        std::vector<int> camids;
        std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> feats;
        bool hascam = sim_->get_next_cam(time_cam, camids, feats);
        if(hascam) {
            featureCallback_(time_cam, feats.front());
            // debug visualization
            Eigen::Vector3d t_wi;
            Eigen::Matrix3d R_iw;
            // sim_->spline.get_pose(time_cam, R_iw, t_wi);
            Eigen::Vector3d body_w, Vi_w;
            if (sim_->spline.get_velocity(time_cam, R_iw, t_wi, body_w, Vi_w))
                LOG(INFO) << "Vi_w " << Vi_w.transpose();
            Eigen::Matrix<double, 17, 1> i_state;
            if (sim_->get_state(time_cam, i_state))
                LOG(INFO) << "i_state v " << i_state.block<3,1>(8,0).transpose();
            Eigen::Matrix4d T_wi = Eigen::Matrix4d::Identity();
            T_wi.block<3,3>(0,0) = R_iw.transpose();
            T_wi.block<3,1>(0,3) = t_wi;
            Eigen::Matrix4d T_ci = Eigen::Matrix4d::Identity();
            T_ci.block<3,3>(0,0) = sim_->R_ItoC;
            T_ci.block<3,1>(0,3) = sim_->p_IinC;
            Eigen::Matrix4d T_wc = T_wi * T_ci.inverse();
            sim_twc_ts.push_back(time_cam);
            sim_twc_gts.insert({time_cam, T_wc});
            sim_Vbks.insert({time_cam, R_iw * Vi_w});
            addKeyFrame(T_wc);
            // updatePoseForDrawing(T_wc);
            // LOG(INFO) << "simulate time stamp " << std::to_string(time_cam);
            // map points visualization
            std::vector<std::array<float,3>> new_pts;
            for (auto & f : feats.front()) {
                if (pt_ids_.find(f.first) == pt_ids_.end()) {
                    pt_ids_.insert(f.first);
                    std::array<float,3> new_pt;
                    Eigen::Map<Eigen::Vector3f> new_pt_eigen(new_pt.data());
                    new_pt_eigen = sim_->featmap[f.first].cast<float>();
                    new_pts.push_back(new_pt);

                    sim_feats.insert({f.first, new_pt_eigen.cast<double>()});
                }
            }
            appendMonoMapPoints(new_pts);

            if (sim_twc_gts.size() == 20)
                return;
        }
        // Sleep based on imu frequency
        double sleep_time = 1 / sim_->sim_freq_imu * 1e6;
        useconds_t thread_sleep_time = static_cast<useconds_t>(sleep_time);
        usleep(thread_sleep_time);
    }
}

void DataPlayer::playImu() {
    while (player_on_ && !imu_q_.empty()) {
        static uint64 data_time_start = imu_q_.front().ts;
        imu_msg msg = imu_q_.front();
        imu_q_.pop();
/*        LOGI("Assembled imu message %lf, %lf, %lf, %lf, %lf, %lf, at time %lf",msg.acc_part.ax, msg.acc_part.ay,
             msg.acc_part.az, msg.gyro_part.rx, msg.gyro_part.ry, msg.gyro_part.rz, msg.ts);*/
        static TimeLagMeasurer timer;
        timer.lagFromLastSecond();
        imuCallback_(msg);
        double time_cost = timer.lagFromLastSecond() * 1000;
        double cur_data_time = data_time_start + timer.lagFromStartSecond() *1e9;
        if (time_cost < 1000.0 / imu_fps_) {
            double time_sleep = 1000.0 / imu_fps_ - time_cost;
            useconds_t thread_sleep_time = static_cast<useconds_t>(time_sleep*1000);
            usleep(thread_sleep_time);
        } else {
            msg = imu_q_.front();
            while (cur_data_time - msg.ts >= 1e9 / imu_fps_ && !imu_q_.empty()) {
                imu_q_.pop();
                msg = imu_q_.front();
            }
        }
    }
}
