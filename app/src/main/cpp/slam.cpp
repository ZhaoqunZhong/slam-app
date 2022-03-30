#include <jni.h>
#include <vector>
#include <string>
#include "platform/cam_publisher.h"
#include "platform/imu_publisher.h"
#include "platform/image_preview.h"
#include "platform/map_drawer.h"

PerfMonitor perf_rgb, perf_acc, perf_gyr, perf_imu;

void rgbCallback(rgb_msg &msg) {
    updatePreviewMat(msg.yMat, true);

    perf_rgb.update();
}

void imuCallback(imu_msg &msg) {

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