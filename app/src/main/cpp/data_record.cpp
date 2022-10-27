//
// Created by zhongzhaoqun on 2022/1/25.
//

#include <jni.h>
#include <vector>
#include <string>
#include "platform/cam_publisher.h"
#include "platform/imu_publisher.h"
#include "platform/image_preview.h"
#include <android/native_window_jni.h>
#include <dirent.h>
#include "glog/logging.h"
#include "platform/data_dumper.h"
#include <sys/stat.h>
#include <filesystem>
#include <fstream>
#include "nlohmann/json.hpp"

DataDumper dataDumper;
PerfMonitor perf_cam, perf_acc, perf_gyr, perf_imu, perf_mag;

void rgbCallback(rgb_msg &msg) {
   dataDumper.dumpRgbImage(msg);

   updatePreviewMat(msg.yMat, true);

   perf_cam.update();
}

void imuCallback(imu_msg &msg) {
    dataDumper.dumpImuData(msg);

    perf_imu.update();
}

void accCallback(acc_msg &msg) {
    dataDumper.dumpAccData(msg);

    perf_acc.update();
}

void gyrCallback(gyr_msg &msg) {
    dataDumper.dumpGyroData(msg);

    perf_gyr.update();
}

void magCallback(mag_msg &msg) {
    dataDumper.dumpMagData(msg);

    perf_mag.update();
}

CamPublisher camPublisher(rgbCallback, nullptr);
ImuPublisher imuPublisher(imuCallback, accCallback, gyrCallback, magCallback);
ImagePreviewer previewer;

extern "C"
JNIEXPORT void JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_startDumpJNI(JNIEnv *env,
                                                                           jobject thiz,
                                                                           jstring config) {
    perf_cam.reset();
    perf_acc.reset();
    perf_gyr.reset();
    perf_imu.reset();
    perf_mag.reset();

    std::string config_string = env->GetStringUTFChars(config, nullptr);
    LOG(INFO) << "record config string " << config_string;

/*    std::string root_path = "/sdcard/slam_app/RecordedData/";
    DIR *dir = opendir(root_path.c_str());
    if (dir) {
    } else {
        std::string cmd = "mkdir -m 777 -p ";
        system((cmd + root_path).c_str());
    }*/
    // Create sdcard root folder for slam_app.
    std::filesystem::path root_path("/sdcard/slam_app/RecordedData/");
    if (std::filesystem::exists(root_path)) {
    } else {
        std::filesystem::create_directories(root_path);
    }
    /// Parse json string
    nlohmann::json config_j = nlohmann::json::parse(config_string);
    std::string folder_name = config_j["folder_name"];
    bool post_with_time = config_j["post_with_time"];
    std::string time_postfix = config_j["time_postfix"];
    bool record_camera = config_j["record_camera"];
    bool save_image = config_j["save_image"];
    std::string camera_id = config_j["camera_id"];
    int camera_resolution = config_j["camera_resolution"];
    bool enable60hz = config_j["enable60hz"];
    std::string image_ts_file_type = config_j["image_ts_file_type"];
    bool record_imu = config_j["record_imu"];
    bool sync_acc_gyr = config_j["sync_acc_gyr"];
    int imu_freq = config_j["imu_freq"];
    std::string imu_file_type = config_j["imu_file_type"];
    int acc_gyr_order = config_j["acc_gyr_order"];
    bool pack_rosbag = config_j["pack_rosbag"];

    //Create data folder for this record.
    std::string data_folder_name = folder_name;
    if (data_folder_name.empty()) {
        data_folder_name = "default";
        config_j["folder_name"] = data_folder_name;
    }
    if (post_with_time)
        data_folder_name += time_postfix;
    data_folder_name += "/";
    // LOG(INFO) << "data folder name " << data_folder_name;
    std::filesystem::path data_folder(root_path.string() + data_folder_name);
    dataDumper.lockFolderSize();
    if (std::filesystem::exists(data_folder))
        std::filesystem::remove_all(data_folder);
    std::filesystem::create_directories(data_folder.string() + "rgb_images/");
    // Write config file of this record.
    std::string record_config_file = data_folder.string() + "record_config.json";
    // LOG(INFO) << "record config file: " << record_config_file;
    std::ofstream cfs(record_config_file, std::ios::out);
    if (cfs.is_open()) {
        cfs << config_string << std::endl;
        cfs.close();
    } else {
        LOG(WARNING) << "Writing record config string to data folder failed!";
    }

    if (record_camera) {
        std::map<int, std::pair<int, int>> cam_res {
                {0, std::make_pair(640, 480)},
                {1, std::make_pair(1280, 720)},
                {2, std::make_pair(1920, 1080)}
        };
        std::pair<int,int> res = cam_res[camera_resolution];
        camPublisher.start(camera_id, res.first, res.second, enable60hz);
        previewer.start(res.second, res.first, preview_native_window);
    }
    if (record_imu)
        imuPublisher.start(imu_freq, sync_acc_gyr);

    if (record_camera || record_imu)
        dataDumper.start(data_folder.string(), record_imu, sync_acc_gyr, acc_gyr_order, imu_file_type,
                         record_camera, image_ts_file_type, pack_rosbag, save_image);
    // dataDumper.unlockFolderSize();
}
extern "C"
JNIEXPORT void JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_stopDumpJNI(JNIEnv *env,
                                                                          jobject thiz) {
    camPublisher.stop();
    previewer.stop();
    imuPublisher.stop();
    dataDumper.stop();
}
extern "C"
JNIEXPORT jobjectArray JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_getImuFreqs(JNIEnv *env,
                                                                          jobject thiz) {
    std::vector<std::string> imu_freqs_str{"0"};
    imu_freqs_str = ImuPublisher::getAvailableImuFreqs();
    jobjectArray imu_freqs = env->NewObjectArray(imu_freqs_str.size(), env->FindClass(
            "java/lang/String"), env->NewStringUTF(""));
    for (int i = 0; i < imu_freqs_str.size(); i++)
        env->SetObjectArrayElement(imu_freqs, i, env->NewStringUTF(imu_freqs_str[i].c_str()));
    return imu_freqs;
}
extern "C"
JNIEXPORT jobjectArray JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_getBackCamIDs(JNIEnv *env,
                                                                            jobject thiz) {
    std::vector<std::string> cams_str{"-1"};
    cams_str = CamPublisher::searchSlamCams();
    jobjectArray cams = env->NewObjectArray(cams_str.size(), env->FindClass(
            "java/lang/String"), env->NewStringUTF(""));
    for (int i = 0; i < cams_str.size(); i++)
        env->SetObjectArrayElement(cams, i, env->NewStringUTF(cams_str[i].c_str()));
    return cams;
}
extern "C"
JNIEXPORT void JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_sendSurfaceToJNI(JNIEnv *env,
                                                                               jobject thiz,
                                                                               jobject cam_sf) {
    if (preview_native_window)
        ANativeWindow_release(preview_native_window);
    preview_native_window = ANativeWindow_fromSurface(env, cam_sf);
    ANativeWindow_acquire(preview_native_window);
}
extern "C"
JNIEXPORT jint JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_getCamFps(JNIEnv *env, jobject thiz) {
    return perf_cam.getFPS();
}
extern "C"
JNIEXPORT jint JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_getAccFps(JNIEnv *env, jobject thiz) {
    return perf_acc.getFPS();
}
extern "C"
JNIEXPORT jint JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_getGyrFps(JNIEnv *env, jobject thiz) {
    return perf_gyr.getFPS();
}
extern "C"
JNIEXPORT jint JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_getImuFps(JNIEnv *env, jobject thiz) {
    return perf_imu.getFPS();
}
extern "C"
JNIEXPORT jint JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_getMagFps(JNIEnv *env, jobject thiz) {
    return perf_mag.getFPS();
}
extern "C"
JNIEXPORT jint JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_getDataSize(JNIEnv *env,
                                                                          jobject thiz) {
    return dataDumper.getCurrentDataSize();
}