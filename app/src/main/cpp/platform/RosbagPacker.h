//
// Created by Lin_JX on 2019/2/22.
//

#ifndef INCLUDE_ROSBAG_PACKER_H
#define INCLUDE_ROSBAG_PACKER_H

#include <queue>
#include <jni.h>
#include <mutex>
#include <MessageType/geometry_msgs/Vector3Stamped.h>

#include "MessageType/sensor_msgs/Image.h"
#include "MessageType/sensor_msgs/CompressedImage.h"
#include "MessageType/sensor_msgs/Imu.h"
#include "RosbagStorage/rosbag/bag.h"

using namespace std;
using namespace ob_slam;
using namespace ob_slam::rosbag;
using namespace ob_slam::geometry_msgs;

#define DEFAULT_ROSBAG_PATH "/sdcard/VIdata/dump/"
#define DEFAULT_TOPIC_IMU   "/imu0"
#define DEFAULT_TOPIC_IMAGE "/cam0/image_raw"
#define DEFAULT_TOPIC_ACCEL "/acc0"
#define DEFAULT_TOPIC_GYRO  "/gyr0"

class RosbagPacker {

public:

    RosbagPacker(const string topic_imu = DEFAULT_TOPIC_IMU,
                 const string topic_image = DEFAULT_TOPIC_IMAGE);
    //~RosbagPacker();

    void open(const string name);
    void close();

    void exit();
    void writeAccel(long long timeStamp, double *accel);
    void writeGyro(long long timeStamp, double *gyro);
    void writeImu(long long timeStamp, double *imu);
    void writeImage(long long timeStamp, const unsigned char *buf_ptr, int size, int width, int height);

    //void packing();
    void reset();

//private:
    string path_;
    string topic_imu_;
    string topic_image_;
    ob_slam::rosbag::Bag bag_;

    bool exit_;

    queue<ob_slam::geometry_msgs::Vector3Stamped> accel_;
    queue<ob_slam::geometry_msgs::Vector3Stamped> gyro_;
    queue<ob_slam::sensor_msgs::Imu> imu_;
    //queue<sensor_msgs::CompressedImage> image_;
    queue<ob_slam::sensor_msgs::Image> image_;

    mutex accelMutex_;
    mutex gyroMutex_;
    mutex imuMutex_;
    mutex imageMutex_;
};

#endif //INCLUDE_ROSBAG_PACKER_H
