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

#ifndef __CALIBRATION_CV_BRIDGE_SIMPLE_H__
#define __CALIBRATION_CV_BRIDGE_SIMPLE_H__

#include "MessageType/sensor_msgs/image_encodings.h"
#include "MessageType/sensor_msgs/Image.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class CvBridgeSimple
{
public:
    /// \brief 将rogbag中的图像消息转换为opencv的mat
    cv::Mat ConvertToCvMat(ob_slam::sensor_msgs::ImageConstPtr const_ptr);

    /// \brief 将rogbag中的图像消息转换为opencv的mat
    cv::Mat ConvertToCvMat(ob_slam::sensor_msgs::ImagePtr ptr);

    /// \brief 将opencv的mat转换为rogbag中的图像消息
    void ConvertToImageMsg(const cv::Mat& cv_img, ob_slam::sensor_msgs::ImagePtr& img_msg);

private:
};

#endif //__CALIBRATION_CV_BRIDGE_SIMPLE_H__
