
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
    void start();
    void stop();
    void dumpRgbImage(rgb_msg & image);
    void dumpImuData(imu_msg & imuMsg);
    void dumpAccData(acc_msg & accMsg);
    void dumpGyroData(gyr_msg & gyroMsg);
    void imuDumpThreadFunction();
private:
    std::string dump_path_;
    bool dump_open_ = false;
    std::string imu_file_format_, image_ts_file_format_;
    std::queue<acc_msg> acc_queue_;
    std::queue<gyr_msg> gyr_queue_;
    std::queue<imu_msg> imu_queue_;
    std::queue<rgb_msg> image_queue_;
    pthread_mutex_t acc_mtx_ = PTHREAD_MUTEX_INITIALIZER, gyr_mtx_ = PTHREAD_MUTEX_INITIALIZER,
    imu_mtx_ = PTHREAD_MUTEX_INITIALIZER, image_mtx_ = PTHREAD_MUTEX_INITIALIZER;
    pthread_t main_th_;

    /// rosbag
    RosbagPacker bag_packer_;
};
#endif //VID_DATA_DUMPER_H

