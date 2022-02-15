#ifndef VINSONANDROID_CAM_PUBLISHER_H
#define VINSONANDROID_CAM_PUBLISHER_H

#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraError.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraMetadataTags.h>
#include <media/NdkImageReader.h>
#include "opencv2/core.hpp"
#include "perf_monitor.h"

#define USE_RGB_CAM
//#define USE_DEPTH_CAM
// #define USE_UV_CH
//#define USE_CONFIDENCE

struct rgb_msg {
    uint64 ts;
    cv::Mat yMat;
    cv::Mat rgbMat;
};

struct depth_msg {
    uint64 ts;
    cv::Mat depthMat;
    cv::Mat confiMat;
};

class CamPublisher {
public:
    CamPublisher( void (*f_rgb) (rgb_msg &), void (*f_depth)(depth_msg &)) {
        rgbCallback_ = f_rgb;
        depthCallback_ = f_depth;
    };
    ~CamPublisher(){};
    void start();
    void stop();
    static uint32_t depth_width_, depth_height_;
    static uint32_t rgb_width_, rgb_height_;

//    void depthManualRun();
    static std::vector<std::string> searchSlamCams();

private:
    std::string searchBackCam();
    ACameraManager *cameraMgr_ = nullptr;

    std::string rgb_cam_id_;

    ACaptureSessionOutputContainer *rgb_outputContainer_ = nullptr;
    ACameraDevice *rgb_cam_ = nullptr;
    ACaptureRequest *rgb_capRequest_ = nullptr;
    ACameraCaptureSession *rgb_capSession_ = nullptr;
    ACaptureSessionOutput *rgb_sessionOutput_ = nullptr;
    ACameraOutputTarget *rgb_outputTarget_ = nullptr;
    ANativeWindow *rgb_imageWindow_ = nullptr;
    AImageReader *rgb_imgReader_ = nullptr;
    void (*rgbCallback_) (rgb_msg &);
    //int64_t rgb_exposure_;
    //int32_t rgb_sensitivity_;
    float rgb_focus_;
    //float rgb_exposure_percent_, rgb_sensitivity_percent_;
    std::vector<int32_t> rgb_ae_fps_range_;


    std::string depth_cam_id_;
    ACaptureSessionOutputContainer *depth_outputContainer_ = nullptr;
    ACameraDevice *depth_cam_ = nullptr;
    ACaptureRequest *depth_capRequest_ = nullptr;
    ACameraCaptureSession *depth_capSession_ = nullptr;
    ACaptureSessionOutput *depth_sessionOutput_ = nullptr;
    ACameraOutputTarget *depth_outputTarget_ = nullptr;
    ANativeWindow *depth_imageWindow_ = nullptr;
    AImageReader *depth_imgReader_ = nullptr;
    void (*depthCallback_) (depth_msg &);

};

#endif //VINSONANDROID_CAM_PUBLISHER_H
