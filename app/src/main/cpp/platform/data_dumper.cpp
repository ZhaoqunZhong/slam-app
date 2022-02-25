#include <dirent.h>
#include <opencv2/imgcodecs.hpp>
#include "data_dumper.h"
#include <fstream>
#include <thread>
#include <iostream>
#include "glog/logging.h"
#include <filesystem>

void DataDumper::dumpRgbImage(rgb_msg & image) {
    if (!dump_open_ || image.yMat.empty())
        return;

    pthread_mutex_lock(&acc_mtx_);
    image_queue_.push(image);
    pthread_mutex_unlock(&acc_mtx_);
}


void DataDumper::dumpAccData(acc_msg & accMsg) {
    if (!dump_open_)
        return;
/*  std::ofstream accf(dump_path_ + "acc" + imu_file_format_, std::ios::app);
    accf << accMsg.ts << "," << accMsg.ax << "," <<accMsg.ay << "," <<accMsg.az << std::endl;
    accf.close();*/
    pthread_mutex_lock(&acc_mtx_);
    acc_queue_.push(accMsg);
    pthread_mutex_unlock(&acc_mtx_);

    /// rosbag
    if (record_rosbag_) {
        double acc[3] {accMsg.ax, accMsg.ay, accMsg.az};
        bag_packer_.writeAccel(accMsg.ts, acc);
    }
}

void DataDumper::dumpGyroData(gyr_msg &gyroMsg) {
    if (!dump_open_)
        return;
/*    std::ofstream gyrf(dump_path_ + "gyr" + imu_file_format_, std::ios::app);
    gyrf << gyroMsg.ts << "," << gyroMsg.rx << "," <<gyroMsg.ry << "," <<gyroMsg.rz << std::endl;
    gyrf.close();*/
    pthread_mutex_lock(&gyr_mtx_);
    gyr_queue_.push(gyroMsg);
    pthread_mutex_unlock(&gyr_mtx_);

    ///rosbag
    if (record_rosbag_) {
        double gyr[3] {gyroMsg.rx, gyroMsg.ry, gyroMsg.rz};
        bag_packer_.writeGyro(gyroMsg.ts, gyr);
    }
}

void DataDumper::dumpImuData(imu_msg & imuMsg) {
    if (!dump_open_)
        return;
/*    std::ofstream imuf(dump_path_ + "imu" + imu_file_format_, std::ios::app);
    imuf << imuMsg.ts << "," << imuMsg.acc_part.ax << "," << imuMsg.acc_part.ay << ","
         << imuMsg.acc_part.az << ","
         << imuMsg.gyro_part.rx << "," << imuMsg.gyro_part.ry << "," << imuMsg.gyro_part.rz
         << std::endl;
    imuf.close();*/
    pthread_mutex_lock(&imu_mtx_);
    imu_queue_.push(imuMsg);
    pthread_mutex_unlock(&imu_mtx_);

    /// rosbag
    if (record_rosbag_) {
        double imu[6] {imuMsg.acc_part.ax, imuMsg.acc_part.ay, imuMsg.acc_part.az, imuMsg.gyro_part.rx, imuMsg.gyro_part.ry, imuMsg.gyro_part.rz};
        bag_packer_.writeImu(imuMsg.ts, imu);
    }
}

void* DumpThreadRunner(void *ptr) {
    DataDumper* classptr = (DataDumper*)ptr;
    classptr->DumpThreadFunction();
    return nullptr;
}

void DataDumper::start(std::string path, int acc_gyr_order, std::string imu_file_format,
                       std::string ts_file_format, bool record_bag, bool save_images) {
    if (started_) {
        LOG(WARNING) << "DataDumper already started, can't call start() again!";
        return;
    }
    started_ = true;
    LOG(INFO) << "DataDumper started.";
    dump_path_ = path;
    acc_gyr_order_ = acc_gyr_order;
    //clear last dump
/*    DIR *dir = opendir(dump_path_.c_str());
    if (dir) {
        LOG(WARNING) << "dump path exists.";
        std::string cmd = "rm -r " + dump_path_;
        system(cmd.c_str());
    }
    std::string cmd = "mkdir -p ";
    system((cmd + dump_path_).c_str());
    system((cmd + dump_path_ + "rgb_images/").c_str());
    system((cmd + dump_path_ + "rgb_images/data/").c_str());*/

/*    cv::FileStorage fs("sdcard/VIdata/config.yaml", cv::FileStorage::READ);
    imu_file_format_ = static_cast<std::string>(fs["imu_file_format"]);
    image_ts_file_format_ = static_cast<std::string>(fs["ts_file_format"]);
    fs.release();*/
    imu_file_format_ = "." + imu_file_format;
    image_ts_file_format_ = "." + ts_file_format;
    record_rosbag_ = record_bag;
    save_images_ = save_images;

    dump_open_ = true;
    pthread_create(&main_th_, nullptr, DumpThreadRunner, this);

    /// rosbag
    if (record_rosbag_) {
        std::string bag_name = dump_path_ + "rosbag.bag";
        bag_packer_.open(bag_name);
    }
}

void DataDumper::stop() {
    if (!started_) {
        LOG(WARNING) << "DataDumper hasn't started yet, can't call stop().";
        return;
    }
    started_ = false;
    LOG(INFO) << "DataDumper stopped.";
    dump_open_ = false;
    pthread_join(main_th_, nullptr);

    ///rosbag
    if (record_rosbag_)
        bag_packer_.close();
}


void DataDumper::DumpThreadFunction() {
    while (dump_open_ || !acc_queue_.empty() || !gyr_queue_.empty() || !imu_queue_.empty() || !image_queue_.empty()) {
        useconds_t thread_sleep_time = static_cast<useconds_t>(10);
        usleep(thread_sleep_time);

        /// write imu sensor caches to file
        std::queue<acc_msg> acc_buf; //here is empty
        pthread_mutex_lock(&acc_mtx_);
        if (!acc_queue_.empty()) {
            std::swap(acc_queue_, acc_buf); //here is filled with accumulated msgs, while acc_queue_ is emptied.
        }
        pthread_mutex_unlock(&acc_mtx_);

        std::queue<gyr_msg> gyr_buf;
        pthread_mutex_lock(&gyr_mtx_);
        if (!gyr_queue_.empty()) {
            std::swap(gyr_queue_, gyr_buf);
        }
        pthread_mutex_unlock(&gyr_mtx_);

        std::queue<imu_msg> imu_buf;
        pthread_mutex_lock(&imu_mtx_);
        if (!imu_queue_.empty()) {
            std::swap(imu_queue_, imu_buf);
        }
        pthread_mutex_unlock(&imu_mtx_);

        std::ofstream accf(dump_path_ + "acc" + imu_file_format_, std::ios::app);
        while (!acc_buf.empty()) {
            acc_msg msg = acc_buf.front();
            acc_buf.pop();
            accf << msg.ts << "," << msg.ax << "," <<msg.ay << "," <<msg.az << std::endl;
        }
        accf.close();

        std::ofstream gyrf(dump_path_ + "gyr" + imu_file_format_, std::ios::app);
        while (!gyr_buf.empty()) {
            gyr_msg msg = gyr_buf.front();
            gyr_buf.pop();
            gyrf << msg.ts << "," << msg.rx << "," <<msg.ry << "," <<msg.rz << std::endl;
        }
        gyrf.close();

        std::ofstream imuf(dump_path_ + "imu" + imu_file_format_, std::ios::app);
        while (!imu_buf.empty()) {
            imu_msg msg = imu_buf.front();
            imu_buf.pop();
            if (acc_gyr_order_ == 0)
                imuf << msg.ts << "," << msg.acc_part.ax << "," << msg.acc_part.ay << ","
                     << msg.acc_part.az << "," << msg.gyro_part.rx << "," << msg.gyro_part.ry
                     << "," << msg.gyro_part.rz << std::endl;
            else
                imuf << msg.ts << "," << msg.gyro_part.rx << "," << msg.gyro_part.ry << ","
                     << msg.gyro_part.rz << "," << msg.acc_part.ax << "," << msg.acc_part.ay << ","
                     << msg.acc_part.az << ","<< std::endl;
        }
        imuf.close();

        /// write image caches to .png files and rosbag packer
        std::queue<rgb_msg> image_buf;
        pthread_mutex_lock(&image_mtx_);
        if (!image_queue_.empty()) {
            std::swap(image_queue_, image_buf);
        }
        pthread_mutex_unlock(&image_mtx_);
        while (!image_buf.empty()) {
            rgb_msg msg = image_buf.front();
            image_buf.pop();

            std::ofstream fs(dump_path_ + "image_ts" + image_ts_file_format_, std::ios::app);
            fs << msg.ts << std::endl;
            fs.close();

            if (save_images_) {
                std::string folder = dump_path_ + "rgb_images/";
                std::string filename = folder + std::to_string(msg.ts) + ".png";

                cv::imwrite(filename, msg.yMat);
                // thread function version of saving image file
/*                auto f = [](std::string &filename, cv::Mat &mat) {
                    cv::imwrite(filename, mat);
                };
                std::thread t(f, filename, msg.yMat);*/
            }

            /// rosbag
            if (record_rosbag_)
                bag_packer_.writeImage(msg.ts, msg.yMat.data, msg.yMat.rows * msg.yMat.cols, msg.yMat.cols, msg.yMat.rows);
        }

        ///rosbag packer flush sensor caches to bag file
        if (record_rosbag_) {
            bag_packer_.imuMutex_.lock();
            while (!bag_packer_.imu_.empty()) {
                try {
                    sensor_msgs::Imu msg = bag_packer_.imu_.front();
                    bag_packer_.bag_.write(DEFAULT_TOPIC_IMU, msg.header.stamp, msg);
                    bag_packer_.imu_.pop();
                } catch (BagException e) {

                }
            }
            bag_packer_.imuMutex_.unlock();

            bag_packer_.accelMutex_.lock();
            while (!bag_packer_.accel_.empty()) {
                try {
                    geometry_msgs::Vector3Stamped msg = bag_packer_.accel_.front();
                    bag_packer_.bag_.write(DEFAULT_TOPIC_ACCEL, msg.header.stamp, msg);
                    bag_packer_.accel_.pop();
                }catch(BagException e){

                }
            }
            bag_packer_.accelMutex_.unlock();

            bag_packer_.gyroMutex_.lock();
            while (!bag_packer_.gyro_.empty()) {
                try{
                    geometry_msgs::Vector3Stamped msg = bag_packer_.gyro_.front();
                    bag_packer_.bag_.write(DEFAULT_TOPIC_GYRO, msg.header.stamp, msg);
                    bag_packer_.gyro_.pop();
                }catch(BagException e){

                }
            }
            bag_packer_.gyroMutex_.unlock();

            bag_packer_.imageMutex_.lock();
            while (!bag_packer_.image_.empty()) {
                try {
                    sensor_msgs::Image image = bag_packer_.image_.front();
                    bag_packer_.bag_.write(DEFAULT_TOPIC_IMAGE, image.header.stamp, image);
                    bag_packer_.image_.pop(); // this line sometimes fails
                } catch (BagException e) {

                }
            }
            bag_packer_.imageMutex_.unlock();
        }
    }
}
