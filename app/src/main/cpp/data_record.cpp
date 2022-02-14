//
// Created by zhongzhaoqun on 2022/1/25.
//

#include <jni.h>
#include <vector>
#include <string>

extern "C"
JNIEXPORT void JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_startDumpJNI(JNIEnv *env, jobject thiz) {
    // TODO: implement startDumpJNI()
}
extern "C"
JNIEXPORT void JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_stopDumpJNI(JNIEnv *env, jobject thiz) {
    // TODO: implement stopDumpJNI()
}
extern "C"
JNIEXPORT jobjectArray JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_getImuFreqs(JNIEnv *env,
                                                                          jobject thiz) {
    std::vector<std::string> imu_freqs_str{"400", "800"};
    jobjectArray imu_freqs = env->NewObjectArray(imu_freqs_str.size(), env->FindClass(
            "java/lang/String"),env->NewStringUTF(""));
    for (int i = 0; i < imu_freqs_str.size(); i++)
        env->SetObjectArrayElement(imu_freqs, i, env->NewStringUTF(imu_freqs_str[i].c_str()));
    return imu_freqs;
}
extern "C"
JNIEXPORT jobjectArray JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_getBackCamIDs(JNIEnv *env,
                                                                            jobject thiz) {
    std::vector<std::string> cams_str{"0", "1", "2"};
    jobjectArray cams = env->NewObjectArray(cams_str.size(), env->FindClass(
            "java/lang/String"),env->NewStringUTF(""));
    for (int i = 0; i < cams_str.size(); i++)
        env->SetObjectArrayElement(cams, i, env->NewStringUTF(cams_str[i].c_str()));
    return cams;
}