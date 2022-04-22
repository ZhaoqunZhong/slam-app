#include <unistd.h>
#include <memory>
#include <thread>
#include <chrono>
#include "glog/logging.h"
#include <dirent.h>
#include "MessageType/sensor_msgs/Image.h"
#include "MessageType/sensor_msgs/CompressedImage.h"
#include "MessageType/sensor_msgs/Imu.h"
#include "RosbagStorage/rosbag/view.h"
#include "cv_bridge_simple.h"
#include "algorithm_interface.h"
#include "modify-vins-mono/slam_api/obslam_api.h"

// #define IS_SEPARATE_IMU
// #define GLOG_TO_FILE

SLAM_HANDLE slam_;

ob_slam::rosbag::Bag play_bag;
#define DEFAULT_TOPIC_IMU   "/imu0"
#define DEFAULT_TOPIC_IMAGE "/cam0/image_raw"
#define DEFAULT_TOPIC_ACCEL "/acc0"
#define DEFAULT_TOPIC_GYRO  "/gyr0"
std::vector <std::string> ros_topics{DEFAULT_TOPIC_IMU, DEFAULT_TOPIC_IMAGE, DEFAULT_TOPIC_ACCEL, DEFAULT_TOPIC_GYRO};


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
    slam_->accCallback(msg.ts/1e9, msg.ax, msg.ay, msg.az);
#endif
}

void AlgorithmInterface::gyrCallback(gyr_msg &msg) {
    if (!algorithm_on_)
        return;
    if (slam_ == nullptr)
        return;
#ifdef IS_SEPARATE_IMU
    slam_->gyrCallback(msg.ts/1e9, msg.rx, msg.ry, msg.rz);
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
TimeLagMeasurer slam_timer;
void statusCallback(slam_status *status) {
    if (*status == INITIALIZING) {
        bootup_time = slam_timer.getCurrentTimeSecond();
        LOG(INFO) << "slam bootup costs " << bootup_time - create_time;
    }
    if (*status == TRACKING) {
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

//    start_stdcout_logger();
#ifdef GLOG_TO_FILE
    std::string log_file = "/sdcard/orbbec-vio-data/log/log";
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
                ob_slam::geometry_msgs::Vector3Stamped::ConstPtr
                        acc_ptr = iter->instantiate<ob_slam::geometry_msgs::Vector3Stamped>();
                if (slam_ == nullptr)
                    return;
                slam_->accCallback(acc_ptr->header.stamp.toSec(), acc_ptr->vector.x, acc_ptr->vector.y, acc_ptr->vector.z);
            }
            if (DEFAULT_TOPIC_GYRO == iter->getTopic()) {
                ob_slam::geometry_msgs::Vector3Stamped::ConstPtr
                        gyr_ptr = iter->instantiate<ob_slam::geometry_msgs::Vector3Stamped>();
                if (slam_ == nullptr)
                        return;
                slam_->gyrCallback(gyr_ptr->header.stamp.toSec(), gyr_ptr->vector.x, gyr_ptr->vector.y, gyr_ptr->vector.z);
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

