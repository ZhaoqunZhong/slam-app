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

DataDumper dataDumper;
PerfMonitor perf_rgb, perf_acc, perf_gyr, perf_imu;

void rgbCallback(rgb_msg &msg) {
   dataDumper.dumpRgbImage(msg);

   updatePreviewMat(msg.yMat, true);

   perf_rgb.update();
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

CamPublisher camPublisher(rgbCallback, nullptr);
ImuPublisher imuPublisher(imuCallback, accCallback, gyrCallback);
ImagePreviewer previewer;

extern "C"
JNIEXPORT void JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_startDumpJNI(JNIEnv *env,
                                                                           jobject thiz,
                                                                           jstring config) {
    std::string config_string = env->GetStringUTFChars(config, nullptr);
    LOG(INFO) << "record config string " << config_string;

/*    std::string root_path = "/sdcard/slam_app/RecordedData/";
    DIR *dir = opendir(root_path.c_str());
    if (dir) {
    } else {
        std::string cmd = "mkdir -m 777 -p ";
        system((cmd + root_path).c_str());
    }*/
/*    std::string root_path = "/sdcard/slam_app/RecordedData/";
    if(0 == access(root_path.c_str(), 0)) {
        LOG(INFO) << "root path exists.";
    } else {
        FILE* root_folder = std::fopen(root_path.c_str(), "w+");
        if (root_folder == nullptr)
            LOG(WARNING) << "Create root path fails.";
    }*/
    std::filesystem::path root_path("/sdcard/slam_app/RecordedData/");
    if (std::filesystem::exists(root_path)) {
        // LOG(INFO) << "vio folder size " << std::filesystem::file_size("/sdcard/vio");
    } else {
        std::filesystem::create_directories(root_path);
    }

    // camPublisher.start();
    // imuPublisher.start();
    // previewer.start();
    // dataDumper.start(root_path + );
}
extern "C"
JNIEXPORT void JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_stopDumpJNI(JNIEnv *env,
                                                                          jobject thiz) {
    // TODO: implement stopDumpJNI()
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