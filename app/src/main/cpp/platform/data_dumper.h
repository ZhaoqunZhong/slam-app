
#ifndef VID_DATA_DUMPER_H
#define VID_DATA_DUMPER_H

#include <string>
#include "cam_publisher.h"
#include "imu_publisher.h"
#include <queue>
#include "RosbagPacker.h"

class DataDumper {
public:
    DataDumper(){};
    ~DataDumper(){};
    void start(std::string path, bool sync_acc_gyr, int acc_gyr_order, std::string imu_file_format,
               std::string ts_file_format, bool record_bag, bool save_images);
    void stop();
    void dumpRgbImage(rgb_msg & image);
    void dumpImuData(imu_msg & imuMsg);
    void dumpAccData(acc_msg & accMsg);
    void dumpGyroData(gyr_msg & gyroMsg);
    void DumpThreadFunction();
    uint64_t getCurrentDataSize();
    void lockFolderSize();
    void unlockFolderSize();

private:
    std::string dump_path_;
    bool dump_open_ = false;
    bool sync_acc_gyr_;
    int acc_gyr_order_ = 0; // 0: acc in front, 1: gyr in front
    std::string imu_file_format_, image_ts_file_format_;
    bool save_images_;
    std::queue<acc_msg> acc_queue_;
    std::queue<gyr_msg> gyr_queue_;
    std::queue<imu_msg> imu_queue_;
    std::queue<rgb_msg> image_queue_;
    pthread_mutex_t acc_mtx_ = PTHREAD_MUTEX_INITIALIZER, gyr_mtx_ = PTHREAD_MUTEX_INITIALIZER,
    imu_mtx_ = PTHREAD_MUTEX_INITIALIZER, image_mtx_ = PTHREAD_MUTEX_INITIALIZER;
    std::atomic<bool> folder_size_lock_ = true;
    pthread_t main_th_;
    bool started_ = false;
    /// rosbag
    RosbagPacker bag_packer_;
    bool record_rosbag_ = false;
};
#endif //VID_DATA_DUMPER_H

