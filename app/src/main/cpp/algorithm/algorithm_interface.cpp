#include <unistd.h>
#include <memory>
#include <thread>
#include <chrono>
#include "glog/logging.h"
#include <dirent.h>
#include <RosbagPacker.h>
#include "MessageType/sensor_msgs/Image.h"
#include "MessageType/sensor_msgs/CompressedImage.h"
#include "MessageType/sensor_msgs/Imu.h"
#include "RosbagStorage/rosbag/view.h"
#include "cv_bridge_simple.h"
#include "algorithm_interface.h"
#include "modify-vins-mono/slam_api/obslam_api.h"

// #define IS_SEPARATE_IMU
#define GLOG_TO_FILE

SLAM_HANDLE slam_;

ob_slam::rosbag::Bag play_bag;
#define DEFAULT_TOPIC_IMU   "/imu0"
#define DEFAULT_TOPIC_IMAGE "/cam0/image_raw"
#define DEFAULT_TOPIC_ACCEL "/acc0"
#define DEFAULT_TOPIC_GYRO  "/gyr0"
std::vector <std::string> ros_topics{DEFAULT_TOPIC_IMU, DEFAULT_TOPIC_IMAGE, DEFAULT_TOPIC_ACCEL, DEFAULT_TOPIC_GYRO};

/// Assemble imu finite automata
enum Input {acc,gyr} cur_input;
enum State {WAIT_FOR_MSG, ACC, ACC_GYR, ACC_GYR_ACC, ACC_GYRs, ACC_GYR_ACCs} cur_state;
std::vector<gyr_msg> gyro_cache_;
std::vector<acc_msg> acc_cache_;
void constructImuInterpolateAcc() {
    switch (cur_state) {
        case WAIT_FOR_MSG:
            if (cur_input == acc) {
                cur_state = ACC;
            } else {
                gyro_cache_.clear();
            }
            break;
        case ACC:
            if (cur_input == acc) {
                acc_cache_.erase(acc_cache_.begin());
            } else {
                if (gyro_cache_.front().ts <= acc_cache_.front().ts) {
                    gyro_cache_.clear();
                } else {
                    cur_state = ACC_GYR;
                }
            }
            break;
        case ACC_GYR:
            if (cur_input == acc) {
                if (acc_cache_.back().ts <= gyro_cache_.front().ts) {
                    acc_cache_.erase(acc_cache_.begin());
                } else {
                    imu_msg imu;
                    imu.ts = gyro_cache_.front().ts;
                    imu.gyro_part = gyro_cache_.front();
                    double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                    imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                    acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                    acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                    slam_add_imu(slam_, imu.ts, imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az, 
                                 imu.gyro_part.rx, imu.gyro_part.ry, imu.gyro_part.rz);
                    cur_state = ACC_GYR_ACC;
                }
            } else {
                cur_state = ACC_GYRs;
            }
            break;
        case ACC_GYRs:
            if (cur_input == acc) {
                if (acc_cache_.back().ts <= gyro_cache_.front().ts) {
                    acc_cache_.erase(acc_cache_.begin());
                } else if (acc_cache_.back().ts > gyro_cache_.front().ts && acc_cache_.back().ts < gyro_cache_.back().ts) {
                    int i;
                    for (i = 0; i < gyro_cache_.size(); i++) {
                        if (gyro_cache_[i].ts < acc_cache_.back().ts) {
                            imu_msg imu;
                            imu.ts = gyro_cache_[i].ts;
                            imu.gyro_part = gyro_cache_[i];
                            double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                            imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                            acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                            acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                            slam_add_imu(slam_, imu.ts, imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az, 
                                 imu.gyro_part.rx, imu.gyro_part.ry, imu.gyro_part.rz);
                        } else {
                            break;
                        }
                    }
                    acc_cache_.erase(acc_cache_.begin());
                    gyro_cache_.erase(gyro_cache_.begin(), gyro_cache_.begin() + i);
                } else {
                    for (gyr_msg & gyro : gyro_cache_) {
                        imu_msg imu;
                        imu.ts = gyro.ts;
                        imu.gyro_part = gyro;
                        double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                        imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                        acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                        acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                        slam_add_imu(slam_, imu.ts, imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az, 
                                 imu.gyro_part.rx, imu.gyro_part.ry, imu.gyro_part.rz);
                    }
                    gyro_cache_.erase(gyro_cache_.begin(), gyro_cache_.begin() + gyro_cache_.size() - 1);
                    cur_state = ACC_GYR_ACC;
                }
            } else {}
            break;
        case ACC_GYR_ACC:
            if (cur_input == acc) {
                cur_state = ACC_GYR_ACCs;
            } else {
                if (gyro_cache_.back().ts > acc_cache_.back().ts) {
                    acc_cache_.erase(acc_cache_.begin());
                    gyro_cache_.erase(gyro_cache_.begin());
                    cur_state = ACC_GYR;
                } else {
                    imu_msg imu;
                    imu.ts = gyro_cache_.back().ts;
                    imu.gyro_part = gyro_cache_.back();
                    double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                    imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                    acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                    acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                    slam_add_imu(slam_, imu.ts, imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az, 
                                 imu.gyro_part.rx, imu.gyro_part.ry, imu.gyro_part.rz);
                    gyro_cache_.erase(gyro_cache_.begin());
                }
            }
            break;
        case ACC_GYR_ACCs:
            if (cur_input == acc) {

            } else {
                if (gyro_cache_.back().ts <= acc_cache_[1].ts) {
                    imu_msg imu;
                    imu.ts = gyro_cache_.back().ts;
                    imu.gyro_part = gyro_cache_.back();
                    double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                    imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                    acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                    acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                    slam_add_imu(slam_, imu.ts, imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az, 
                                 imu.gyro_part.rx, imu.gyro_part.ry, imu.gyro_part.rz);
                    gyro_cache_.erase(gyro_cache_.begin());
                } else if (gyro_cache_.back().ts >= acc_cache_.back().ts) {
                    gyro_cache_.erase(gyro_cache_.begin());
                    acc_cache_.erase(acc_cache_.begin(), acc_cache_.begin() + acc_cache_.size() - 1);
                    cur_state = ACC_GYR;
                } else {
                    int i;
                    for (i = 0; i < acc_cache_.size(); i++) {
                        if (acc_cache_[i].ts >= gyro_cache_.back().ts)
                            break;
                    }
                    imu_msg imu;
                    imu.ts = gyro_cache_.back().ts;
                    imu.gyro_part = gyro_cache_.back();
                    double factor = (imu.ts - acc_cache_[i-1].ts)/(acc_cache_[i].ts - acc_cache_[i-1].ts);
                    imu.acc_part = {imu.ts, acc_cache_[i-1].ax * (1-factor) + acc_cache_[i].ax *factor,
                                    acc_cache_[i-1].ay * (1-factor) + acc_cache_[i].ay *factor,
                                    acc_cache_[i-1].az * (1-factor) + acc_cache_[i].az *factor};
                    slam_add_imu(slam_, imu.ts, imu.acc_part.ax, imu.acc_part.ay, imu.acc_part.az, 
                                 imu.gyro_part.rx, imu.gyro_part.ry, imu.gyro_part.rz);
                    gyro_cache_.erase(gyro_cache_.begin());
                    acc_cache_.erase(acc_cache_.begin(), acc_cache_.begin() + i-1);
                }
            }
            break;
    }
}

void AlgorithmInterface::rgbCallback(rgb_msg &msg) {
    if (!algorithm_on_)
        return;
    if (slam_ == nullptr)
        return;
    // updatePreviewMat(msg.yMat, true);
    slam_add_image(slam_, msg.ts, msg.yMat.cols, msg.yMat.rows, msg.yMat.data);
}

void AlgorithmInterface::featureCallback(double ts, std::vector <std::pair<size_t, Eigen::VectorXf>> &feats) {
    if (!algorithm_on_)
        return;
    if (slam_ == nullptr)
        return;
}

void AlgorithmInterface::imuCallback(imu_msg &msg) {
    if (!algorithm_on_)
        return;
    if (slam_ == nullptr)
        return;
#ifndef IS_SEPARATE_IMU
    slam_add_imu(slam_, msg.ts, msg.acc_part.ax, msg.acc_part.ay, msg.acc_part.az,
                 msg.gyro_part.rx, msg.gyro_part.ry, msg.gyro_part.rz);
#endif
}

void AlgorithmInterface::accCallback(acc_msg &msg) {
    if (!algorithm_on_)
        return;
    if (slam_ == nullptr)
        return;
#ifdef IS_SEPARATE_IMU
    acc_cache_.push_back(msg);
    cur_input = acc;
    constructImuInterpolateAcc();
#endif
}

void AlgorithmInterface::gyrCallback(gyr_msg &msg) {
    if (!algorithm_on_)
        return;
    if (slam_ == nullptr)
        return;
#ifdef IS_SEPARATE_IMU
    gyro_cache_.push_back(msg);
    cur_input = gyr;
    constructImuInterpolateAcc();
#endif
}

void *threadRunner(void *ptr) {
    AlgorithmInterface *classptr = (AlgorithmInterface *) ptr;
    classptr->runAlgorithm();
    return nullptr;
}

void previewCallback(slam_processed_image *image) {
    cv::Mat preview(image->image_height, image->image_width, CV_8UC3, image->image_data);
    updatePreviewMat(preview.clone(), true);
}

PerfMonitor AlgorithmInterface::perf_pose_;
void poseCallback(slam_pose *pose) {
    Eigen::Map<Eigen::Vector3f> tran(pose->tran);
    Eigen::Map<Eigen::Quaternionf> rot(pose->rot);
    // Eigen::Vector3d tran(pose->tran);
    // Eigen::Quaternionf rot(pose->rot);
    Eigen::Vector3d dtran = tran.cast<double>();
    Eigen::Quaterniond drot = rot.cast<double>();
    if (pose->type == ACCURATE) {
        updatePoseForDrawing(dtran, drot);
        AlgorithmInterface::perf_pose_.update();
    }

}

double create_time, bootup_time, init_time;
bool init_time_lock = false; //Only keep the first time, not the later re-initializations.
TimeLagMeasurer slam_timer;
void statusCallback(slam_status *status) {
    if (*status == INITIALIZING) {
        bootup_time = slam_timer.getCurrentTimeSecond();
        LOG(INFO) << "slam bootup costs " << bootup_time - create_time;
    }
    if (*status == TRACKING && !init_time_lock) {
        init_time_lock = true;
        init_time = slam_timer.getCurrentTimeSecond();
        LOG(INFO) << "slam initialization costs " << init_time - create_time;
    }
}
float AlgorithmInterface::getSlamBootTime() {
    return std::max(bootup_time - create_time, 0.0);
}
float AlgorithmInterface::getInitializationTime() {
    return std::max(init_time - create_time, 0.0);
}

int AlgorithmInterface::getPoseFps() {
    return perf_pose_.getFPS();
}

void AlgorithmInterface::start() {
    clearVisualizationBuffers();
    cur_state = WAIT_FOR_MSG;
    init_time_lock = false;

//    start_stdcout_logger();
#ifdef GLOG_TO_FILE
    std::string log_file = "/sdcard/slam_app/SlamRun/log";
    google::SetLogDestination(google::GLOG_INFO, "");
    google::SetLogDestination(google::GLOG_ERROR, "");
    google::SetLogDestination(google::GLOG_FATAL, "");
    google::SetLogDestination(google::GLOG_WARNING, log_file.c_str());
    google::InitGoogleLogging("");
#endif
    std::string file_name = "sdcard/slam_app/SlamRun/config.yaml";
    cv::FileStorage fs;
    try {
        fs.open(file_name, cv::FileStorage::READ);
    } catch (const cv::Exception &e) {
        LOG(ERROR) << e.what();
    }
    run_offline_ = static_cast<int>(fs["run_offline"]);
    std::string config;
    if (run_offline_)
        config = static_cast<std::string>(fs["offline_config"]);
    else
        config = static_cast<std::string>(fs["online_config"]);
    bag_name_ = static_cast<std::string>(fs["rosbag_path"]);
    fs.release();

    create_time = slam_timer.getCurrentTimeSecond();
    create_slam(&slam_, nullptr, config.c_str());
    slam_register_image_process_callback(slam_, previewCallback);
    slam_register_pose_callback(slam_, poseCallback);
    slam_register_status_callback(slam_, statusCallback);
    start_slam(slam_);
    bootup_time = slam_timer.getCurrentTimeSecond();
    LOG(INFO) << "slam bootup costs " << bootup_time - create_time;

    algorithm_on_ = true;
    pthread_create(&algo_t_, nullptr, threadRunner, this);
}

void AlgorithmInterface::stop() {
    if (!algorithm_on_) {
        LOG(WARNING) << "AlgorithmInterface hasn't started yet, can't call stop().";
        return;
    }
    algorithm_on_ = false;
    pthread_join(algo_t_, nullptr);

    stop_slam(slam_);
    release_slam(slam_);

#ifdef GLOG_TO_FILE
    google::FlushLogFiles(google::GLOG_WARNING);
    google::ShutdownGoogleLogging();
#endif

}


void AlgorithmInterface::runAlgorithm() {
    if (run_offline_) {
        LOG(INFO) << "opening rosbag: " << bag_name_ << std::endl;
        play_bag.open(bag_name_, static_cast<uint32_t>(ob_slam::rosbag::BagMode::Read));
        ob_slam::rosbag::View view(play_bag, ob_slam::rosbag::TopicQuery(ros_topics),
                                   ob_slam::TIME_MIN, ob_slam::TIME_MAX);
        ob_slam::rosbag::View::iterator iter, next_iter;
        for (iter = view.begin(), next_iter = view.begin(); iter != view.end(); iter++) {
            // LOG_FIRST_N(INFO, 100) << "topic " << iter->getTopic();
            if (DEFAULT_TOPIC_IMAGE == iter->getTopic()) {
                if (iter->isType<ob_slam::sensor_msgs::Image>()) {
                    LOG_FIRST_N(WARNING, 1) << "sensor_msgs::Image detected";
                    ob_slam::sensor_msgs::Image::ConstPtr image_ptr = iter->instantiate<ob_slam::sensor_msgs::Image>();
                    CvBridgeSimple cvb;
                    cv::Mat mat = cvb.ConvertToCvMat(image_ptr);
                    if (slam_ == nullptr)
                        return;
                    slam_add_image(slam_, image_ptr->header.stamp.toNSec(), mat.cols, mat.rows, mat.data);
                } else if (iter->isType<ob_slam::sensor_msgs::CompressedImage>()) { ///compressed image
                    LOG_FIRST_N(WARNING, 1) << "sensor_msgs::CompressedImage detected";
                    ob_slam::sensor_msgs::CompressedImage::ConstPtr image_ptr = iter->instantiate<ob_slam::sensor_msgs::CompressedImage>();
                    cv::Mat mat = cv::imdecode(image_ptr->data, cv::IMREAD_GRAYSCALE);
                    if (slam_ == nullptr)
                        return;
                    slam_add_image(slam_, image_ptr->header.stamp.toNSec(), mat.cols, mat.rows, mat.data);
                } else {
                    LOG(ERROR) << "Unknow image type in ros bag!";
                    return;
                }
            }
#ifndef IS_SEPARATE_IMU
            if (DEFAULT_TOPIC_IMU == iter->getTopic()) {
                LOG_FIRST_N(WARNING, 1) << "sensor_msgs::Imu detected";
                ob_slam::sensor_msgs::Imu::ConstPtr imu_ptr = iter->instantiate<ob_slam::sensor_msgs::Imu>();
                if (slam_ == nullptr)
                    return;
                slam_add_imu(slam_, imu_ptr->header.stamp.toNSec(), imu_ptr->linear_acceleration.x,
                             imu_ptr->linear_acceleration.y,
                             imu_ptr->linear_acceleration.z, imu_ptr->angular_velocity.x,
                             imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z);
            }
#else
            if (DEFAULT_TOPIC_ACCEL == iter->getTopic()) {
                LOG_FIRST_N(WARNING, 1) << "sensor_msgs::Acc detected";
                ob_slam::geometry_msgs::Vector3Stamped::ConstPtr
                        acc_ptr = iter->instantiate<ob_slam::geometry_msgs::Vector3Stamped>();
                if (slam_ == nullptr)
                    return;
                acc_msg accMsg{acc_ptr->header.stamp.toNSec(), acc_ptr->vector.x, acc_ptr->vector.y, acc_ptr->vector.z};
                acc_cache_.push_back(accMsg);
                cur_input = acc;
                constructImuInterpolateAcc();
            }
            if (DEFAULT_TOPIC_GYRO == iter->getTopic()) {
                LOG_FIRST_N(WARNING, 1) << "sensor_msgs::Gyr detected";
                ob_slam::geometry_msgs::Vector3Stamped::ConstPtr
                        gyr_ptr = iter->instantiate<ob_slam::geometry_msgs::Vector3Stamped>();
                if (slam_ == nullptr)
                        return;
                gyr_msg gyrMsg{gyr_ptr->header.stamp.toNSec(), gyr_ptr->vector.x, gyr_ptr->vector.y, gyr_ptr->vector.z};
                gyro_cache_.push_back(gyrMsg);
                cur_input = gyr;
                constructImuInterpolateAcc();
            }
#endif
            /// sleep between messages
            if (++next_iter != view.end()) {
                double sleep_time = next_iter->getTime().toSec() - iter->getTime().toSec();
                int play_rate = 2;
                sleep_time = sleep_time / play_rate;
                struct timespec ts;
                ts.tv_sec = (long) sleep_time;
                ts.tv_nsec = (long) ((sleep_time - (long) sleep_time) * 1e9);
                nanosleep(&ts, 0);
            }

            if (!algorithm_on_)
                break;
        }

        play_bag.close();
        LOG(INFO) << "Ros bag closed: " << bag_name_;
    }

}

