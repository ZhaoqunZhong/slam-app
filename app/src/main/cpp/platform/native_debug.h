#ifndef SLAMAPP_NATIVE_DEBUG_H
#define SLAMAPP_NATIVE_DEBUG_H

#include <android/log.h>
#include <Eigen/Eigen>
#include "opencv2/opencv.hpp"
#include "perf_monitor.h"

#define LOG_TAG "slam_app"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define ASSERT(cond, fmt, ...)                                \
  if (!(cond)) {                                              \
    __android_log_assert(#cond, LOG_TAG, fmt, ##__VA_ARGS__); \
  }

#define REACH_HERE LOGW("Program reach %s %s line %d", __FILE__, __func__, __LINE__);

int start_stdcout_logger();

// extern std::string app_internal_storage;

/*----------------visualization--------------------*/
extern pthread_mutex_t pose_mtx;
extern Eigen::Matrix4f pose_to_draw, test_pose_to_draw;
extern Eigen::RowVector3f pose_color;
extern Eigen::RowVector3f traj_color;
extern std::vector<std::array<float,3>> traj_to_draw, traj_gt;
extern std::vector<std::array<float,6>> map_points;
extern std::vector<std::array<float,3>> mono_map_points;
extern std::vector<Eigen::Matrix4d> key_frames;
extern void updatePose4x4ForDrawing (Eigen::Matrix4d &pose);
extern PerfMonitor pose_fps;
extern void updatePoseForDrawing (Eigen::Vector3d &p, Eigen::Quaterniond &q);
extern void updateTestPoseForDrawing (Eigen::Matrix4f &pose);
extern void updatePoseColor(Eigen::RowVector3f &color);
extern void updateTrajColor(Eigen::RowVector3f &color);
extern void setTrajGroundtruth(std::vector<std::array<float,3>> &traj);
extern void updateMapPoints(std::vector<std::array<float,6>> &map);
extern void addKeyFrame(Eigen::Matrix4d &pose);
extern void clearKeyFrames();
extern void appendMonoMapPoints(std::vector<std::array<float,3>> &pts);
extern bool reset;
extern void clearVisualizationBuffers();


/*----------------preview--------------------*/
extern pthread_mutex_t preview_mtx;
extern pthread_cond_t preview_cond;
extern cv::Mat preview_mat;
extern bool rotate;
extern void updatePreviewMat(cv::Mat mat, bool r);

/*----------------dataset player--------------------*/
extern pthread_mutex_t data_play_mtx;
extern pthread_cond_t data_play_cond;

/*----------------simulation groundtruth--------------------*/
extern std::unordered_map<double, Eigen::Matrix4d> sim_twc_gts;
extern std::unordered_map<int, Eigen::Vector3d> sim_feats;
extern std::vector<Eigen::Vector3d> sim_feats_BA_debug;
// extern Eigen::Matrix3d sim_relative_R;
// extern Eigen::Vector3d sim_relative_t;
extern std::vector<double> sim_twc_ts;
extern std::unordered_map<int, Eigen::Vector3d> sim_Vbks;
extern std::vector<double> sim_ts_used;
extern double sim_scale;

#endif //SLAMAPP_NATIVE_DEBUG_H