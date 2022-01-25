//
// Created by Lin_JX on 2019/2/27.
//

#include <time.h>
#include <sys/stat.h>
#include <MessageType/sensor_msgs/CompressedImage.h>
#include <MessageType/sensor_msgs/Image.h>
#include "RosbagPacker.h"
#include "native_debug.h"

RosbagPacker::RosbagPacker(const string topic_imu, const string topic_image) {
    topic_imu_ = topic_imu;
    topic_image_ = topic_image;
    exit_ = false;
}

/*RosbagPacker::~RosbagPacker() {
    LOGW("RosbagPacker release");
    close();
}*/

void RosbagPacker::reset() {
    exit_ = false;
    while (!accel_.empty()) accel_.pop();
    while (!gyro_.empty()) gyro_.pop();
    while (!imu_.empty()) imu_.pop();
    while (!image_.empty()) image_.pop();
}

void RosbagPacker::open(const string name) {
    reset();
    path_ = string(name);
    string dirPath = path_.substr(0, path_.find_last_of("/"));
    if (access(dirPath.c_str(), 0) != 0) {
        LOGW("%s does not exit_", dirPath.c_str());
        if (0 == mkdir(dirPath.c_str(), 777)) {
            LOGW("mkdir success");
        } else {
            LOGW("mkdir failured");
        }
    }

    bag_.open(path_, static_cast<uint32_t>(BagMode::Write));
    //bag_.setCompression(rosbag::compression::LZ4);
}

void RosbagPacker::exit() {
    exit_ = true;
}

void RosbagPacker::close() {
    LOGW("RosbagPacker Close.");
    bag_.close();
}

void RosbagPacker::writeAccel(long long timeStamp, double *value) {
    geometry_msgs::Vector3Stamped accelMsg;
    accelMsg.header.stamp.fromNSec(timeStamp);
    accelMsg.vector.x = value[0];
    accelMsg.vector.y = value[1];
    accelMsg.vector.z = value[2];

    accelMutex_.lock();
    accel_.push(accelMsg);
    accelMutex_.unlock();
}

void RosbagPacker::writeGyro(long long timeStamp, double *value) {

    geometry_msgs::Vector3Stamped gyroMsg;
    gyroMsg.header.stamp.fromNSec(timeStamp);
    gyroMsg.vector.x = value[0];
    gyroMsg.vector.y = value[1];
    gyroMsg.vector.z = value[2];

    gyroMutex_.lock();
    gyro_.push(gyroMsg);
    gyroMutex_.unlock();
}

void RosbagPacker::writeImu(long long timeStamp, double *value) {

    sensor_msgs::Imu imuMsg;
    imuMsg.header.stamp.fromNSec(timeStamp);
    imuMsg.header.frame_id = "body";
    imuMsg.angular_velocity.x = value[3];
    imuMsg.angular_velocity.y = value[4];
    imuMsg.angular_velocity.z = value[5];
    imuMsg.linear_acceleration.x = value[0];
    imuMsg.linear_acceleration.y = value[1];
    imuMsg.linear_acceleration.z = value[2];
    imuMutex_.lock();
    imu_.push(imuMsg);
    imuMutex_.unlock();
}

void
RosbagPacker::writeImage(long long timeStamp, const unsigned char *buf_ptr, int size, int width,
                         int height) {
    //LOGW("size = %d", size);
    //LOGW("writeImage image time = %lld", timeStamp);
    static uint32_t image_cnt = 0;
    //sensor_msgs::CompressedImage image;
    sensor_msgs::Image image;
    image.header.stamp.fromNSec(timeStamp);
    image.header.frame_id = "";
    image.header.seq = image_cnt++;
    image.width = width;
    image.height = height;
    image.encoding = "mono8";
    image.is_bigendian = 0;

    image.step = width;
    //image.format = "png";
    image.data.resize(size);
    memcpy(image.data.data(), buf_ptr, size);

    imageMutex_.lock();
    image_.push(image);
    imageMutex_.unlock();
}

/*
void RosbagPacker::packing() {
    while (!exit_) {
        imuMutex_.lock();
        sensor_msgs::Imu msg = imu_.front();
        try {
            bag_.write(DEFAULT_TOPIC_IMU, msg.header.stamp, msg);
        } catch (BagException e) {

        }
        imu_.pop();
        imuMutex_.unlock();

        imageMutex_.lock();
        sensor_msgs::Image image = image_.front();
        try {
            bag_.write(DEFAULT_TOPIC_IMAGE, image.header.stamp, image);
        } catch (BagException e) {

        }
        image_.pop();
        imageMutex_.unlock();
    }
    close();
}*/
