#include "cam_publisher.h"
#include "native_debug.h"
#include "camera_utils/ndk_utils.h"
#include <thread>
#include <dirent.h>
#include "opencv2/imgcodecs.hpp"
#include <unistd.h>
#include <type_traits>
#include <fstream>
#include "Eigen/Core"
#include <iostream>
#include "glog/logging.h"


void (*rgbCallback)(rgb_msg &);
void (*depthCallback)(depth_msg &);
uint32_t CamPublisher::depth_width_, CamPublisher::depth_height_;
uint32_t CamPublisher::rgb_width_, CamPublisher::rgb_height_;
uint64_t rgb_last_ts = 0, depth_last_ts = 0;
int64_t rgb_realtime_exposure, rgb_readout_time;

/*pthread_t depth_capture_t;
bool depth_manual_on = true;
const uint depth_manual_rate = 15;
constexpr int32_t DEPTH_PERIOD_US = int32_t(1e6 / depth_manual_rate);*/

/*void *depthManualThreadWrapper(void *ptr) {
    CamPublisher *classptr = (CamPublisher *) ptr;
    classptr->depthManualRun();
    return nullptr;
}*/

void printCamInfos(ACameraManager *cameraManager, const char *id) {
    LOGI("-------------------camera %s info --------------------", id);

    ACameraMetadata *metadataObj;
    ACameraManager_getCameraCharacteristics(cameraManager, id, &metadataObj);
    ACameraMetadata_const_entry entry = {0};
    //camera intrinsic
    ACameraMetadata_getConstEntry(metadataObj,
                                  ACAMERA_LENS_INTRINSIC_CALIBRATION, &entry);
/*    Eigen::Matrix3f intrinsic;
    intrinsic << entry.data.f[0], entry.data.f[4], entry.data.f[2], 0, entry.data.f[1], entry.data.f[3], 0, 0, 1;
    start_stdcout_logger();
    std::cout << "intrinsic : " << intrinsic << std::endl;*/

    // rgb_exposure range
    ACameraMetadata_getConstEntry(metadataObj,
                                  ACAMERA_SENSOR_INFO_EXPOSURE_TIME_RANGE, &entry);

    int64_t minExposure = entry.data.i64[0];
    int64_t maxExposure = entry.data.i64[1];
    LOGI("minExposure=%ld vs maxExposure=%ld", minExposure, maxExposure);

    //rgb focus : 0 un-calibrated 1 approximated 2 calibrated
    ACameraMetadata_getConstEntry(metadataObj,
                                  ACAMERA_LENS_INFO_FOCUS_DISTANCE_CALIBRATION, &entry);
    uint8_t focus_cali = entry.data.u8[0];
//    LOGI("focus calibration quality %d", focus_cali);
    LOGI("focus calibration quality %s",
         (focus_cali == 2) ? "calibrated" : ((focus_cali == 1) ? "approximated" : "un-calibrated"));

    ACameraMetadata_getConstEntry(metadataObj,
                                  ACAMERA_LENS_INFO_MINIMUM_FOCUS_DISTANCE, &entry);
    float min_focus = entry.data.f[0];
    LOGI("minimal focus distance %f m", min_focus);

    // rgb_sensitivity
    ACameraMetadata_getConstEntry(metadataObj,
                                  ACAMERA_SENSOR_INFO_SENSITIVITY_RANGE, &entry);

    int32_t minSensitivity = entry.data.i32[0];
    int32_t maxSensitivity = entry.data.i32[1];

    LOGI("minSensitivity=%d vs maxSensitivity=%d", minSensitivity, maxSensitivity);
    ////////////////////////////////////////////////////////////////

    ACameraMetadata_getConstEntry(metadataObj,
                                  ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &entry);

    for (int i = 0; i < entry.count; i += 4) {
        // We are only interested in output streams, so skip input stream
        int32_t input = entry.data.i32[i + 3];
        if (input)
            continue;

        int32_t format = entry.data.i32[i + 0];
        if (format == AIMAGE_FORMAT_YUV_420_888) {
            int32_t width = entry.data.i32[i + 1];
            int32_t height = entry.data.i32[i + 2];
            LOGI("YUV_420_888: maxWidth=%d vs maxHeight=%d", width, height);
        } else if (format == AIMAGE_FORMAT_RAW16) {
            int32_t width = entry.data.i32[i + 1];
            int32_t height = entry.data.i32[i + 2];
            LOGI("RAW16: maxWidth=%d vs maxHeight=%d", width, height);
        } else if (format == AIMAGE_FORMAT_RGB_888) {
            int32_t width = entry.data.i32[i + 1];
            int32_t height = entry.data.i32[i + 2];
            LOGI("RGB_888: maxWidth=%d vs maxHeight=%d", width, height);
        } else if (format == AIMAGE_FORMAT_JPEG) {
            int32_t width = entry.data.i32[i + 1];
            int32_t height = entry.data.i32[i + 2];
            LOGI("JPEG: maxWidth=%d vs maxHeight=%d", width, height);
        } else if (format == AIMAGE_FORMAT_PRIVATE) {
            int32_t width = entry.data.i32[i + 1];
            int32_t height = entry.data.i32[i + 2];
            LOGI("PRIVATE: maxWidth=%d vs maxHeight=%d", width, height);
        } else {
            int32_t width = entry.data.i32[i + 1];
            int32_t height = entry.data.i32[i + 2];
            LOGI("format %x: maxWidth=%d vs maxHeight=%d", format, width, height);
        }
    }

    ACameraMetadata_getConstEntry(metadataObj,
                                  ACAMERA_SENSOR_ORIENTATION, &entry);
    int32_t orientation = entry.data.i32[0];
    LOGI("sensor orientation: %d", orientation);

    ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_FACING, &entry);
    uint8_t facing = static_cast<acamera_metadata_enum_android_lens_facing_t>(
            entry.data.u8[0]);
    LOGI("cam facing: %s", facing == 0 ? "front" : "back");

    //OIS
    ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION,
                                  &entry);
    for (int i = 0; i < entry.count; i++) {
        uint8_t mode = entry.data.u8[i];
        LOGI("OIS mode option %d", mode);
    }
    //AF
    ACameraMetadata_getConstEntry(metadataObj, ACAMERA_CONTROL_AF_AVAILABLE_MODES, &entry);
    for (int i = 0; i < entry.count; i++) {
        uint8_t mode = entry.data.u8[i];
        LOGI("AF mode option %d", mode);
    }
    //VIS
    ACameraMetadata_getConstEntry(metadataObj, ACAMERA_CONTROL_VIDEO_STABILIZATION_MODE, &entry);
    uint8_t mode = static_cast<acamera_metadata_enum_acamera_control_video_stabilization_mode>(
            entry.data.u8[0]);
    LOGI("VIS mode %d", mode);

    ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_INFO_AVAILABLE_FOCAL_LENGTHS, &entry);
    for (int i = 0; i < entry.count; i++) {
        float focal_length = entry.data.f[i];
        LOGI("available focal length %f", focal_length);
    }

    ACameraMetadata_getConstEntry(metadataObj, ACAMERA_SENSOR_INFO_ACTIVE_ARRAY_SIZE, &entry);
    int32_t active_left = entry.data.i32[0];
    int32_t active_top = entry.data.i32[1];
    int32_t active_right = entry.data.i32[2];
    int32_t active_bottom = entry.data.i32[3];
    LOGI("active array size left %d, top %d, right %d, bottom %d", active_left, active_top,
         active_right, active_bottom);
}


/**
 * Device listeners
 */
void onDisconnected(void *context, ACameraDevice *device) {
    LOGI("onDisconnected");
}

void onError(void *context, ACameraDevice *device, int error) {
    LOGW("error %d", error);
}

ACameraDevice_stateCallbacks cameraDeviceCallbacks = {
        .context = nullptr,
        .onDisconnected = onDisconnected,
        .onError = onError,
};

void depthImgReaderCbk(void *context, AImageReader *reader) {
    static uint frame_drop_cnt = 0;

    int32_t format;
    media_status_t status = AImageReader_getFormat(reader, &format);
/*    if (format != AIMAGE_FORMAT_RAW16)
        return;*/

    AImage *image = nullptr;
    status = AImageReader_acquireLatestImage(reader, &image);
    if (status != AMEDIA_OK) {
        frame_drop_cnt++;
        LOGW("Depth frame drop %d", frame_drop_cnt);
        AImage_delete(image);
        return;
    }

    int64_t time;
    AImage_getTimestamp(image, &time);
//    LOGI("time lag Depth %f", CamPublisher::timer_.getCurrentTimeSecond() - time/1e9);
    depth_msg depthMsg;
    depthMsg.ts = time;
    if (depthMsg.ts <= depth_last_ts) {
        AImage_delete(image);
        LOGW("depth image timestamp disorder!");
        return;
    }
    depth_last_ts = depthMsg.ts;

    uint8_t *data;
    int32_t data_len;
    AImage_getPlaneData(image, 0, &data, &data_len);
//    AImage_delete(image);

    cv::Mat depth_image(CamPublisher::depth_height_, CamPublisher::depth_width_, CV_16UC1,
                        (uint16_t *) data);
    cv::rotate(depth_image, depthMsg.depthMat, cv::ROTATE_90_CLOCKWISE);
#if USE_CONFIDENCE
    cv::Mat confi_image(CamPublisher::depth_height_, CamPublisher::depth_width_, CV_16UC1,
                        (uint16_t *) (data + CamPublisher::depth_width_ * CamPublisher::depth_height_ * 2));
    rotate(confi_image, depthMsg.confiMat, cv::ROTATE_90_CLOCKWISE);
#endif
//    cv::Mat depth_image (CamPublisher::depth_height_, CamPublisher::depth_width_, CV_16UC1);
//    memcpy(depth_image.data, data, CamPublisher::depth_height_ * CamPublisher::depth_width_ * 2);
//    rotate(depth_image, depthMsg.depthMat, cv::ROTATE_90_CLOCKWISE);
//    cv::Mat confi_image (CamPublisher::depth_height_, CamPublisher::depth_width_, CV_16UC1);
//    memcpy(confi_image.data, data + 640*480*2, CamPublisher::depth_height_ * CamPublisher::depth_width_ * 2);
//    rotate(depth_image, depthMsg.confiMat, cv::ROTATE_90_CLOCKWISE);

    depthCallback(depthMsg);
    AImage_delete(image);
}

void rgbImgReaderCbk(void *context, AImageReader *reader) {
    static uint frame_drop_cnt = 0;

    int32_t format;
    AImageReader_getFormat(reader, &format);
    if (format != AIMAGE_FORMAT_YUV_420_888)
        return;

    AImage *image = nullptr;
    media_status_t status = AImageReader_acquireLatestImage(reader, &image);
    if (status != AMEDIA_OK) {
        frame_drop_cnt++;
        LOGW("RGB frame drop %d", frame_drop_cnt);
        AImage_delete(image);
        return;
    }
//    AImageCropRect rect;
//    AImage_getCropRect(image, &rect);
//    LOGI("crop rect %d %d %d %d", rect.bottom, rect.top, rect.left, rect.right);
/*    int32_t num_planes;
    AImage_getNumberOfPlanes(image, &num_planes);
    LOGI("Number of planes: %d", num_planes);*/

    int64_t time;
    AImage_getTimestamp(image, &time);
    rgb_msg rgbMsg;
    rgbMsg.ts = time;
    //rgbMsg.ts = time + rgb_realtime_exposure / 2 + rgb_readout_time / 2;
    if (rgbMsg.ts <= rgb_last_ts) {
        AImage_delete(image);
        LOGW("rgb image timestamp disorder!");
        return;
    }
    rgb_last_ts = rgbMsg.ts;
    uint8_t *y_data;
    int32_t y_len;
    AImage_getPlaneData(image, 0, &y_data, &y_len);
    cv::Mat y_image(CamPublisher::rgb_height_, CamPublisher::rgb_width_, CV_8UC1, y_data);

    // TimeLagMeasurer timer;
    // rgbMsg.yMat = y_image;
    rgbMsg.yMat = y_image.clone();
    // LOG(INFO) << "Cam publisher image clone costs " << timer.lagFromStartSecond()*1e3 << " ms";
#ifdef USE_UV_CH
    uint8_t *uv_data;
    int32_t uv_len;
    AImage_getPlaneData(image, 1, &uv_data, &uv_len);
    cv::Mat yuv(CamPublisher::rgb_height_ * 3 / 2, CamPublisher::rgb_width_, CV_8UC1, y_data);
    std::memcpy(yuv.data + y_len, uv_data, uv_len);
    cv::Mat rgb;
    cv::cvtColor(yuv, rgb, cv::COLOR_YUV2RGB_NV21);
    rgbMsg.rgbMat = rgb.clone();
#endif

    rgbCallback(rgbMsg);
    AImage_delete(image);
}

AImageReader *createImageReader(int32_t width, int32_t height, std::string type) {
    AImageReader *reader = nullptr;

    media_status_t status;

    if (type == "rgb")
        status = AImageReader_new(width, height, AIMAGE_FORMAT_YUV_420_888,
                                  2, &reader);
    else if (type == "depth")
        status = AImageReader_new(width, height * 2, AIMAGE_FORMAT_RAW16,
                                  2,
                                  &reader); //apply for double size of depth because of attached confidence
    ASSERT(status == AMEDIA_OK, "failed to create image reader!");

    AImageReader_ImageListener listener{
            .context = nullptr,
            .onImageAvailable = (type == "depth" ? depthImgReaderCbk : rgbImgReaderCbk),
    };

    AImageReader_setImageListener(reader, &listener);

    return reader;
}


std::string CamPublisher::searchBackCam() {
    std::vector <std::string> candidates;
    ACameraIdList *cameraIds = nullptr;
    ACameraManager_getCameraIdList(cameraMgr_, &cameraIds);
    LOGI("found camera total count %d", cameraIds->numCameras);

    for (int i = 0; i < cameraIds->numCameras; ++i) {
        const char *id = cameraIds->cameraIds[i];

        ACameraMetadata *metadataObj;
        ACameraManager_getCameraCharacteristics(cameraMgr_, id, &metadataObj);

/*        size_t physical_cam_nums;
        const char *const *physical_cam_ids;
        bool logi_cam = ACameraMetadata_isLogicalMultiCamera(metadataObj, &physical_cam_nums,
                                                            &physical_cam_ids);
        if (logi_cam) {
            LOGW("camera id %s is logical cam with %d physical cameras", id, physical_cam_nums);
            std::string physical_ids_string;
            for (int i = 0; i < physical_cam_nums; i++)
                physical_ids_string += std::string(*(physical_cam_ids + i)) + " ";
            LOGW("Physical camera ids are %s", physical_ids_string.c_str());
        }*/

        ACameraMetadata_const_entry lensInfo = {0};
        ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_FACING, &lensInfo);

        auto facing = static_cast<acamera_metadata_enum_android_lens_facing_t>(
                lensInfo.data.u8[0]);

        if (facing != ACAMERA_LENS_FACING_BACK)
            continue;

        candidates.push_back(std::string(id));
//        printCamInfos(cameraMgr_, id);
    }
    ACameraManager_deleteCameraIdList(cameraIds);
    ASSERT(candidates.size(), "didn't find rgb camera!");

    std::string cam_id = candidates.front();
    //LOGI("found rgb camera %s", cam_id.c_str());
    return cam_id;
}


/**
 * Session state callbacks
 */

void onSessionActive(void *context, ACameraCaptureSession *session) {
    LOGI("onSessionActive()");
}

void onSessionReady(void *context, ACameraCaptureSession *session) {
    LOGI("onSessionReady()");
}

void onSessionClosed(void *context, ACameraCaptureSession *session) {
    LOGI("onSessionClosed()");
}

ACameraCaptureSession_stateCallbacks sessionStateCallbacks{
        .context = nullptr,
        .onClosed = onSessionClosed,
        .onReady = onSessionReady,
        .onActive = onSessionActive,
};


/**
 * Capture callbacks
 */

void rgb_onCaptureFailed(void *context, ACameraCaptureSession *session,
                         ACaptureRequest *request, ACameraCaptureFailure *failure) {
    LOGW("RGB capture fail.");
}

void depth_onCaptureFailed(void *context, ACameraCaptureSession *session,
                           ACaptureRequest *request, ACameraCaptureFailure *failure) {
    LOGW("Depth capture fail.");
}

void onCaptureSequenceCompleted(void *context, ACameraCaptureSession *session,
                                int sequenceId, int64_t frameNumber) {}

void onCaptureSequenceAborted(void *context, ACameraCaptureSession *session,
                              int sequenceId) {}

void rgb_onCaptureCompleted(
        void *context, ACameraCaptureSession *session,
        ACaptureRequest *request, const ACameraMetadata *result) {

    ACameraMetadata_const_entry entry = {0};

    ACameraMetadata_getConstEntry(result, ACAMERA_SENSOR_ROLLING_SHUTTER_SKEW, &entry);
    rgb_readout_time = entry.data.i64[0];
    LOG_FIRST_N(INFO,1) << "rgb_readout_time " << rgb_readout_time;

    ACameraMetadata_getConstEntry(result, ACAMERA_SENSOR_EXPOSURE_TIME, &entry);
    int64_t exposure_time = entry.data.i64[0];
    LOG_FIRST_N(INFO,1) << "rgb_exposure_time " << exposure_time;
//    rgb_realtime_exposure = exposure_time;

//    ACameraMetadata_getConstEntry(result, ACAMERA_SENSOR_FRAME_DURATION, &entry);
//    int64_t frame_duration = entry.data.i64[0];
//    LOGI("rgb ACAMERA_SENSOR_FRAME_DURATION %ld", frame_duration);

//    ACameraMetadata_getConstEntry(result,ACAMERA_LENS_FOCUS_RANGE, &entry);
//    float minFocus = entry.data.f[0];
//    float maxFocus = entry.data.f[1];
//    LOGI("minFocus=%f vs maxFocus=%f", minFocus, maxFocus);
//
//    //(left, top, right, bottom)
//    ACameraMetadata_getConstEntry(result,ACAMERA_SCALER_CROP_REGION, &entry);
//    int32_t crop_left = entry.data.i32[0];
//    int32_t crop_top = entry.data.i32[1];
//    int32_t crop_right = entry.data.i32[2];
//    int32_t crop_bottom = entry.data.i32[3];
//    LOGI("crop region left %d, top %d, right %d, bottom %d", crop_left, crop_top, crop_right, crop_bottom);

}

void depth_onCaptureCompleted(
        void *context, ACameraCaptureSession *session,
        ACaptureRequest *request, const ACameraMetadata *result) {

    ACameraMetadata_const_entry entry = {0};
/*
    ACameraMetadata_getConstEntry(result, ACAMERA_SENSOR_ROLLING_SHUTTER_SKEW, &entry);
    int64_t rolling_shutter = entry.data.i64[0];
    LOGI("depth ACAMERA_SENSOR_ROLLING_SHUTTER_SKEW %ld", rolling_shutter);

    ACameraMetadata_getConstEntry(result, ACAMERA_SENSOR_EXPOSURE_TIME, &entry);
    int64_t exposure_time = entry.data.i64[0];
    LOGI("depth ACAMERA_SENSOR_EXPOSURE_TIME %ld", exposure_time);

    ACameraMetadata_getConstEntry(result, ACAMERA_SENSOR_FRAME_DURATION, &entry);
    int64_t frame_duration = entry.data.i64[0];
    LOGI("depth ACAMERA_SENSOR_FRAME_DURATION %ld", frame_duration);*/
}

ACameraCaptureSession_captureCallbacks rgb_captureCallbacks{
        .context = nullptr,
        .onCaptureStarted = nullptr,
        .onCaptureProgressed = nullptr,
        .onCaptureCompleted = rgb_onCaptureCompleted,
        .onCaptureFailed = rgb_onCaptureFailed,
        .onCaptureSequenceCompleted = onCaptureSequenceCompleted,
        .onCaptureSequenceAborted = onCaptureSequenceAborted,
        .onCaptureBufferLost = nullptr,
};

ACameraCaptureSession_captureCallbacks depth_captureCallbacks{
        .context = nullptr,
        .onCaptureStarted = nullptr,
        .onCaptureProgressed = nullptr,
        .onCaptureCompleted = depth_onCaptureCompleted,
        .onCaptureFailed = depth_onCaptureFailed,
        .onCaptureSequenceCompleted = onCaptureSequenceCompleted,
        .onCaptureSequenceAborted = onCaptureSequenceAborted,
        .onCaptureBufferLost = nullptr,
};


void CamPublisher::start() {
    rgbCallback = rgbCallback_;
    depthCallback = depthCallback_;
    cameraMgr_ = ACameraManager_create();
//    searchBackCam();
    std::string file_name = "sdcard/orbbec-vio-data/config.yaml";
    cv::FileStorage fs(file_name, cv::FileStorage::READ);
    rgb_width_ = static_cast<int>(fs["rgb_width"]);
    rgb_height_ = static_cast<int>(fs["rgb_height"]);
/*
    rgb_exposure_ = static_cast<int>(fs["rgb_exposure"]);
    rgb_sensitivity_ = static_cast<int>(fs["rgb_sensitivity"]);
    rgb_exposure_percent_ = static_cast<float>(fs["rgb_req_exposure_percent"]);
    rgb_sensitivity_percent_ = static_cast<float>(fs["rgb_req_sensitivity_percent"]);
    */
    rgb_focus_ = static_cast<float>(fs["rgb_focus"]);
    LOGW("rgb request focus %f", rgb_focus_);
    // depth_width_ = static_cast<int>(fs["depth_req_width"]);
    // depth_height_ = static_cast<int>(fs["depth_req_height"]);
    fs["rgb_cam_id"] >> rgb_cam_id_;
    LOGW("rgb cam id : %s", rgb_cam_id_.c_str());

#ifdef USE_RGB_CAM
    ACameraManager_openCamera(cameraMgr_, rgb_cam_id_.c_str(), &cameraDeviceCallbacks, &rgb_cam_);
    rgb_imgReader_ = createImageReader(rgb_width_, rgb_height_, "rgb");
    AImageReader_getWindow(rgb_imgReader_, &rgb_imageWindow_);
    ANativeWindow_acquire(rgb_imageWindow_);
    ACaptureSessionOutput_create(rgb_imageWindow_, &rgb_sessionOutput_);
    ACameraOutputTarget_create(rgb_imageWindow_, &rgb_outputTarget_);
    ACameraDevice_createCaptureRequest(rgb_cam_, TEMPLATE_PREVIEW, &rgb_capRequest_);
    ACaptureRequest_addTarget(rgb_capRequest_, rgb_outputTarget_);
/*
    ACameraMetadata *metadataObj;
    ACameraManager_getCameraCharacteristics(cameraMgr_, rgb_cam_id_.c_str(), &metadataObj);
    ACameraMetadata_const_entry entry = {0};
    ACameraMetadata_getConstEntry(metadataObj, ACAMERA_SENSOR_INFO_EXPOSURE_TIME_RANGE, &entry);
    int64_t minExposure = entry.data.i64[0];
    int64_t maxExposure = entry.data.i64[1];
    int64_t exposure_ = minExposure + (maxExposure - minExposure) * rgb_exposure_percent_;
    ACameraMetadata_getConstEntry(metadataObj, ACAMERA_SENSOR_INFO_SENSITIVITY_RANGE, &entry);
    int32_t minSensitivity = entry.data.i32[0];
    int32_t maxSensitivity = entry.data.i32[1];
    int32_t sensitivity_ = minSensitivity + (maxSensitivity - minSensitivity) * rgb_sensitivity_percent_;

    CALL_METADATA(getConstEntry(metadataObj, ACAMERA_SENSOR_ORIENTATION, &entry));
    rgb_sensor_orientation = entry.data.i32[0];
    ACameraMetadata_free(metadataObj);
    */
    uint8_t aeMode = ACAMERA_CONTROL_AE_MODE_ON;
//    uint8_t aeMode = ACAMERA_CONTROL_AE_MODE_OFF;
    CALL_REQUEST(setEntry_u8(rgb_capRequest_, ACAMERA_CONTROL_AE_MODE, 1, &aeMode));
/*    CALL_REQUEST(setEntry_i32(rgb_capRequest_, ACAMERA_SENSOR_SENSITIVITY, 1, &sensitivity_));
    CALL_REQUEST(setEntry_i64(rgb_capRequest_, ACAMERA_SENSOR_EXPOSURE_TIME, 1, &exposure_));*/
//    int32_t kFpsRange[2] = {30, 60};
    fs["rgb_AE_fps_range"] >> rgb_ae_fps_range_;
    CALL_REQUEST(setEntry_i32(rgb_capRequest_, ACAMERA_CONTROL_AE_TARGET_FPS_RANGE, 2,
                              rgb_ae_fps_range_.data()));

    uint8_t afModeOff = ACAMERA_CONTROL_AF_MODE_OFF;
    CALL_REQUEST(setEntry_u8(rgb_capRequest_, ACAMERA_CONTROL_AF_MODE, 1, &afModeOff));
    CALL_REQUEST(setEntry_float(rgb_capRequest_, ACAMERA_LENS_FOCUS_DISTANCE, 1, &rgb_focus_));

    uint8_t faceMode = ACAMERA_STATISTICS_FACE_DETECT_MODE_OFF;
    CALL_REQUEST (setEntry_u8(rgb_capRequest_, ACAMERA_STATISTICS_FACE_DETECT_MODE, 1, &faceMode));
    uint8_t vsMode = ACAMERA_CONTROL_VIDEO_STABILIZATION_MODE_OFF;
    CALL_REQUEST (
            setEntry_u8(rgb_capRequest_, ACAMERA_CONTROL_VIDEO_STABILIZATION_MODE, 1, &vsMode));
    uint8_t oisMode = ACAMERA_LENS_OPTICAL_STABILIZATION_MODE_OFF;
    CALL_REQUEST (
            setEntry_u8(rgb_capRequest_, ACAMERA_LENS_OPTICAL_STABILIZATION_MODE, 1, &oisMode));

    ACaptureSessionOutputContainer_create(&rgb_outputContainer_);
    ACaptureSessionOutputContainer_add(rgb_outputContainer_, rgb_sessionOutput_);
    ACameraDevice_createCaptureSession(rgb_cam_, rgb_outputContainer_, &sessionStateCallbacks,
                                       &rgb_capSession_);
    ACameraCaptureSession_setRepeatingRequest(rgb_capSession_, &rgb_captureCallbacks, 1,
                                              &rgb_capRequest_, nullptr);
#endif

#ifdef USE_DEPTH_CAM
    ACameraManager_openCamera(cameraMgr_, depth_cam_id_.c_str(), &cameraDeviceCallbacks,
                              &depth_cam_);
    depth_imgReader_ = createImageReader(depth_width_, depth_height_, "depth");
    AImageReader_getWindow(depth_imgReader_, &depth_imageWindow_);
    ANativeWindow_acquire(depth_imageWindow_);
    ACaptureSessionOutput_create(depth_imageWindow_, &depth_sessionOutput_);
    ACameraOutputTarget_create(depth_imageWindow_, &depth_outputTarget_);
    ACameraDevice_createCaptureRequest(depth_cam_, TEMPLATE_PREVIEW, &depth_capRequest_);
    ACaptureRequest_addTarget(depth_capRequest_, depth_outputTarget_);
    ACaptureSessionOutputContainer_create(&depth_outputContainer_);
    ACaptureSessionOutputContainer_add(depth_outputContainer_, depth_sessionOutput_);
    ACameraDevice_createCaptureSession(depth_cam_, depth_outputContainer_, &sessionStateCallbacks,
                                       &depth_capSession_);
    ACameraCaptureSession_setRepeatingRequest(depth_capSession_, &depth_captureCallbacks, 1,
                                              &depth_capRequest_, nullptr);

//    pthread_create(&depth_capture_t, nullptr, depthManualThreadWrapper, this);
#endif

}


void CamPublisher::stop() {
#ifdef USE_RGB_CAM
    ACaptureRequest_free(rgb_capRequest_);
    ACameraOutputTarget_free(rgb_outputTarget_);
    ACameraDevice_close(rgb_cam_);
    ACaptureSessionOutput_free(rgb_sessionOutput_);
    // ACameraCaptureSession_stopRepeating(rgb_capSession_);
    // ACameraCaptureSession_close(rgb_capSession_);
    ACaptureSessionOutputContainer_free(rgb_outputContainer_);


#endif
#ifdef USE_DEPTH_CAM
    ACameraCaptureSession_stopRepeating(depth_capSession_);
    ACameraCaptureSession_close(depth_capSession_);
    ACameraDevice_close(depth_cam_);

//    depth_manual_on = false;
//    pthread_join(depth_capture_t, nullptr);
#endif
    ACameraManager_delete(cameraMgr_);
}

/*void CamPublisher::depthManualRun() {
    while (depth_manual_on) {
        useconds_t thread_sleep_time = static_cast<useconds_t>(DEPTH_PERIOD_US);
        usleep(thread_sleep_time);

        CALL_SESSION(capture(depth_capSession_, &depth_captureCallbacks, 1, &depth_capRequest_,
                             nullptr));
    }
}*/

