#include <jni.h>
#include <android/native_window_jni.h>
#include <vector>
#include <string>
#include "platform/cam_publisher.h"
#include "platform/imu_publisher.h"
#include "platform/image_preview.h"
#include "platform/map_drawer.h"
#include "glog/logging.h"
#include "nlohmann/json.hpp"
#include "algorithm/algorithm_interface.h"

PerfMonitor perf_cam, perf_acc, perf_gyr, perf_imu, perf_pose;
AlgorithmInterface algo_inter;

void rgbCallback(rgb_msg &msg) {
    algo_inter.rgbCallback(msg);
    perf_cam.update();
}

void imuCallback(imu_msg &msg) {
    algo_inter.imuCallback(msg);
    perf_imu.update();
}

void accCallback(acc_msg &msg) {

    perf_acc.update();
}

void gyrCallback(gyr_msg &msg) {

    perf_gyr.update();
}

CamPublisher camPublisher(rgbCallback, nullptr);
ImuPublisher imuPublisher(imuCallback, accCallback, gyrCallback);
ImagePreviewer previewer;
MapDrawer mapDrawer;


extern "C"
JNIEXPORT void JNICALL
Java_com_zhaoqun_slam_1app_ui_slam_MyGLRenderer_drawFrameJNI(JNIEnv *env, jobject thiz,
                                                             jfloatArray projection_matrix) {
    float *tm = env->GetFloatArrayElements(projection_matrix, 0);
    memcpy(mapDrawer.vp_matrix_, tm, 16 * sizeof(float));
    env->ReleaseFloatArrayElements(projection_matrix, tm, 0);
    mapDrawer.draw(); //put heavy stuff after releasing env, otherwise the glsurfaceview stucks
}
extern "C"
JNIEXPORT void JNICALL
Java_com_zhaoqun_slam_1app_ui_slam_MyGLRenderer_initMapDrawer(JNIEnv *env, jobject thiz) {
    mapDrawer.init(); //the cpp gl part should stay in the current thread
}
extern "C"
JNIEXPORT jobjectArray JNICALL
Java_com_zhaoqun_slam_1app_ui_slam_SlamFragment_getBackCamIDs(JNIEnv *env, jobject thiz) {
    std::vector<std::string> cams_str{"-1"};
    cams_str = CamPublisher::searchSlamCams();
    jobjectArray cams = env->NewObjectArray(cams_str.size(), env->FindClass(
            "java/lang/String"), env->NewStringUTF(""));
    for (int i = 0; i < cams_str.size(); i++)
        env->SetObjectArrayElement(cams, i, env->NewStringUTF(cams_str[i].c_str()));
    return cams;
}
extern "C"
JNIEXPORT jobjectArray JNICALL
Java_com_zhaoqun_slam_1app_ui_slam_SlamFragment_getImuFreqs(JNIEnv *env, jobject thiz) {
    std::vector<std::string> imu_freqs_str{"0"};
    imu_freqs_str = ImuPublisher::getAvailableImuFreqs();
    jobjectArray imu_freqs = env->NewObjectArray(imu_freqs_str.size(), env->FindClass(
            "java/lang/String"), env->NewStringUTF(""));
    for (int i = 0; i < imu_freqs_str.size(); i++)
        env->SetObjectArrayElement(imu_freqs, i, env->NewStringUTF(imu_freqs_str[i].c_str()));
    return imu_freqs;
}
extern "C"
JNIEXPORT void JNICALL
Java_com_zhaoqun_slam_1app_ui_slam_SlamFragment_sendSurfaceToJNI(JNIEnv *env, jobject thiz,
                                                                 jobject cam_sf) {
    if (preview_native_window)
        ANativeWindow_release(preview_native_window);
    preview_native_window = ANativeWindow_fromSurface(env, cam_sf);
    ANativeWindow_acquire(preview_native_window);
}
extern "C"
JNIEXPORT void JNICALL
Java_com_zhaoqun_slam_1app_ui_slam_SlamFragment_startSlamJNI(JNIEnv *env, jobject thiz,
                                                             jstring config) {
    std::string config_string = env->GetStringUTFChars(config, nullptr);
    LOG(INFO) << "slam config string " << config_string;
    /// Parse json string
    nlohmann::json config_j = nlohmann::json::parse(config_string);
    std::string internal_folder_path = config_j["internal_folder_path"];
    std::string camera_id = config_j["camera_id"];
    int camera_resolution = config_j["camera_resolution"];
    bool enable60hz = config_j["enable60hz"];
    int imu_freq = config_j["imu_freq"];

    std::filesystem::path slam_path("/sdcard/slam_app/SlamRun/");
    if (std::filesystem::exists(slam_path)) {
    } else {
        std::filesystem::create_directories(slam_path);
    }
    if (std::filesystem::exists("/sdcard/slam_app/SlamRun/config.yaml")) {
    } else {
        // Copy slam run config file from Downloads folder to sdcard.
        std::filesystem::copy_file(internal_folder_path + "config.yaml",
                                   "/sdcard/slam_app/SlamRun/config.yaml");
    }

    std::string file_name = "sdcard/slam_app/SlamRun/config.yaml";
    cv::FileStorage fs;
    try {
        fs.open(file_name, cv::FileStorage::READ);
    } catch (const cv::Exception &e) {
        LOG(ERROR) << e.what();
    }
    int run_offline = static_cast<int>(fs["run_offline"]);
    if (run_offline) {
    } else {
        std::map<int, std::pair<int, int>> cam_res {
                {0, std::make_pair(640, 480)},
                {1, std::make_pair(1280, 720)},
                {2, std::make_pair(1920, 1080)}
        };
        std::pair<int,int> res = cam_res[camera_resolution];
        camPublisher.start(camera_id, res.first, res.second, enable60hz);
        previewer.start(res.second, res.first, preview_native_window);
        imuPublisher.start(imu_freq, true);
    }

    algo_inter.start();
}
extern "C"
JNIEXPORT void JNICALL
Java_com_zhaoqun_slam_1app_ui_slam_SlamFragment_stopSlamJNI(JNIEnv *env, jobject thiz) {
    camPublisher.stop();
    previewer.stop();
    imuPublisher.stop();

    algo_inter.stop();
}
extern "C"
JNIEXPORT jint JNICALL
Java_com_zhaoqun_slam_1app_ui_slam_SlamFragment_getCamFps(JNIEnv *env, jobject thiz) {
    return perf_cam.getFPS();
}
extern "C"
JNIEXPORT jint JNICALL
Java_com_zhaoqun_slam_1app_ui_slam_SlamFragment_getImuFps(JNIEnv *env, jobject thiz) {
    return perf_imu.getFPS();
}
extern "C"
JNIEXPORT jint JNICALL
Java_com_zhaoqun_slam_1app_ui_slam_SlamFragment_getPoseFps(JNIEnv *env, jobject thiz) {
    return perf_pose.getFPS();
}