//
// Created by zhongzhaoqun on 2022/1/25.
//

#include <jni.h>

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
JNIEXPORT jobject JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_getImuFreqs(JNIEnv *env,
                                                                          jobject thiz) {
    // TODO: implement getImuFreqs()
}
extern "C"
JNIEXPORT jobject JNICALL
Java_com_zhaoqun_slam_1app_ui_data_1record_DataRecordFragment_getBackCamIDs(JNIEnv *env,
                                                                            jobject thiz) {
    // TODO: implement getBackCamIDs()
}