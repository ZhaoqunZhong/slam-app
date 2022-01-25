/*****************************************************************************
 *  Orbbec Calib-X
 *  Copyright (C) 2019 by ORBBEC Technology., Inc.
 *
 *  This file is part of Orbbec Calib-X.
 *
 *  This file belongs to ORBBEC Technology., Inc.
 *  It is considered a trade secret, and is not to be divulged or used by
 * parties who have NOT received written authorization from the owner.
 *
 *  Description
 ****************************************************************************/

#include "cv_bridge_simple.h"

using namespace ob_slam;
cv::Mat CvBridgeSimple::ConvertToCvMat(sensor_msgs::ImageConstPtr const_ptr)
{
    cv::Mat img;
    std::string encoding = const_ptr->encoding;
    if (sensor_msgs::image_encodings::MONO8 == encoding) {
        img.create(const_ptr->height, const_ptr->width, CV_8UC1);
        size_t data_sz = const_ptr->height * const_ptr->width;
        memcpy(img.data, const_ptr->data.data(), data_sz);
    } else if (sensor_msgs::image_encodings::MONO16 == encoding) {
        img.create(const_ptr->height, const_ptr->width, CV_16UC1);
        size_t data_sz = const_ptr->height * const_ptr->width * 2;
        memcpy(img.data, const_ptr->data.data(), data_sz);
    } else if (sensor_msgs::image_encodings::BGR8 == encoding) {
        img.create(const_ptr->height, const_ptr->width, CV_8UC3);
        size_t data_sz = const_ptr->height * const_ptr->width * 3;
        memcpy(img.data, const_ptr->data.data(), data_sz);
    } else if (sensor_msgs::image_encodings::RGB8 == encoding) {
        img.create(const_ptr->height, const_ptr->width, CV_8UC3);
        size_t data_sz = const_ptr->height * const_ptr->width * 3;
        memcpy(img.data, const_ptr->data.data(), data_sz);
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    } else if (sensor_msgs::image_encodings::BGRA8 == encoding) {
        img.create(const_ptr->height, const_ptr->width, CV_8UC4);
        size_t data_sz = const_ptr->height * const_ptr->width * 4;
        memcpy(img.data, const_ptr->data.data(), data_sz);
    } else if (sensor_msgs::image_encodings::RGBA8 == encoding) {
        img.create(const_ptr->height, const_ptr->width, CV_8UC4);
        size_t data_sz = const_ptr->height * const_ptr->width * 4;
        memcpy(img.data, const_ptr->data.data(), data_sz);
        cv::cvtColor(img, img, cv::COLOR_RGBA2BGRA);
    } else if (sensor_msgs::image_encodings::YUV422 == encoding) {
        img.create(const_ptr->height / 2, const_ptr->width, CV_8UC1);
        size_t data_sz = const_ptr->height * const_ptr->width / 2;
        memcpy(img.data, const_ptr->data.data(), data_sz);
    } else {
        img.create(const_ptr->height, const_ptr->width, CV_16UC1);
        size_t data_sz = const_ptr->height * const_ptr->width * 2;
        memcpy(img.data, const_ptr->data.data(), data_sz);
        // fprintf(stderr, "The encoding does not support: %s\n", encoding.c_str());
    }
    return img;
}

cv::Mat CvBridgeSimple::ConvertToCvMat(sensor_msgs::ImagePtr ptr)
{
    cv::Mat img;
    std::string encoding = ptr->encoding;
    if (sensor_msgs::image_encodings::MONO8 == encoding) {
        img.create(ptr->height, ptr->width, CV_8UC1);
        size_t data_sz = ptr->height * ptr->width;
        memcpy(img.data, ptr->data.data(), data_sz);
    } else if (sensor_msgs::image_encodings::MONO16 == encoding) {
        img.create(ptr->height, ptr->width, CV_16UC1);
        size_t data_sz = ptr->height * ptr->width * 2;
        memcpy(img.data, ptr->data.data(), data_sz);
    } else if (sensor_msgs::image_encodings::BGR8 == encoding) {
        img.create(ptr->height, ptr->width, CV_8UC3);
        size_t data_sz = ptr->height * ptr->width * 3;
        memcpy(img.data, ptr->data.data(), data_sz);
    } else if (sensor_msgs::image_encodings::RGB8 == encoding) {
        img.create(ptr->height, ptr->width, CV_8UC3);
        size_t data_sz = ptr->height * ptr->width * 3;
        memcpy(img.data, ptr->data.data(), data_sz);
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    } else if (sensor_msgs::image_encodings::BGRA8 == encoding) {
        img.create(ptr->height, ptr->width, CV_8UC4);
        size_t data_sz = ptr->height * ptr->width * 4;
        memcpy(img.data, ptr->data.data(), data_sz);
    } else if (sensor_msgs::image_encodings::RGBA8 == encoding) {
        img.create(ptr->height, ptr->width, CV_8UC4);
        size_t data_sz = ptr->height * ptr->width * 4;
        memcpy(img.data, ptr->data.data(), data_sz);
        cv::cvtColor(img, img, cv::COLOR_RGBA2BGRA);
    } else {
        fprintf(stderr, "The encoding does not support: %s\n", encoding.c_str());
    }
    return img;
}

void CvBridgeSimple::ConvertToImageMsg(const cv::Mat& cv_img, ob_slam::sensor_msgs::ImagePtr& img_msg)
{
    int type = cv_img.type();
    switch (type) {
    case CV_8UC1: {
        img_msg->encoding = sensor_msgs::image_encodings::MONO8;
        size_t data_sz = cv_img.rows * cv_img.cols;
        img_msg->step = 1;
        img_msg->width = cv_img.cols;
        img_msg->height = cv_img.rows;
        img_msg->data.resize(data_sz);
        memcpy(img_msg->data.data(), cv_img.data, data_sz);
        break;
    }
    case CV_16UC1: {
        img_msg->encoding = sensor_msgs::image_encodings::MONO16;
        size_t data_sz = cv_img.rows * cv_img.cols * 2;
        img_msg->step = 2;
        img_msg->width = cv_img.cols;
        img_msg->height = cv_img.rows;
        img_msg->data.resize(data_sz);
        memcpy(img_msg->data.data(), cv_img.data, data_sz);
        break;
    }
    case CV_8UC3: {
        img_msg->encoding = sensor_msgs::image_encodings::BGR8;
        size_t data_sz = cv_img.rows * cv_img.cols * 3;
        img_msg->step = 3;
        img_msg->width = cv_img.cols;
        img_msg->height = cv_img.rows;
        img_msg->data.resize(data_sz);
        memcpy(img_msg->data.data(), cv_img.data, data_sz);
        break;
    }
    case CV_8UC4: {
        img_msg->encoding = sensor_msgs::image_encodings::BGRA8;
        size_t data_sz = cv_img.rows * cv_img.cols * 4;
        img_msg->step = 4;
        img_msg->width = cv_img.cols;
        img_msg->height = cv_img.rows;
        img_msg->data.resize(data_sz);
        memcpy(img_msg->data.data(), cv_img.data, data_sz);
        break;
    }
    default: {
        fprintf(stderr, "The encoding does not support: %d\n", type);
    }
    }
}
