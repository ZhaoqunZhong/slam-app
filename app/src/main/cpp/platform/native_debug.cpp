
/*http://cncc.bingj.com/cache.aspx?q=use+std+cout+in+android+ndk&d=4687377020238001&mkt=en-US&setlang=en-US&w=ngI5Ze_Z6VlTktzu3WAM95_46h9ZgL4T
 * Redirect std::cout to android log output*/
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <utility>
//#include <android/log.h>
#include "native_debug.h"

static int pfd[2];
static pthread_t thr;
static const char *tag = "std cout";

static void *thread_func(void*)
{
    ssize_t rdsz;
    char buf[128];
    while((rdsz = read(pfd[0], buf, sizeof buf - 1)) > 0) {
        if(buf[rdsz - 1] == '\n') --rdsz;
        buf[rdsz] = 0;  /* add null-terminator */
        __android_log_write(ANDROID_LOG_WARN, tag, buf);
    }
    return nullptr;
}

int start_stdcout_logger()
{
    std::cout.precision(17);

    /* make stdout line-buffered and stderr unbuffered */
    setvbuf(stdout, nullptr, _IOLBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    /* create the pipe and redirect stdout and stderr */
    pipe(pfd);
    dup2(pfd[1], 1);
    dup2(pfd[1], 2);

    /* spawn the logging thread */
    if(pthread_create(&thr, nullptr, thread_func, nullptr) == -1)
        return -1;
    pthread_detach(thr);
    return 0;
}

/*-----------------------app storage path---------------------------*/
// std::string app_internal_storage;


/*-----------------------visualization------------------------------*/
bool reset = false;
pthread_mutex_t pose_mtx = PTHREAD_MUTEX_INITIALIZER;
Eigen::Matrix4f pose_to_draw = Eigen::Matrix4f::Identity();
Eigen::Matrix4f test_pose_to_draw = Eigen::Matrix4f::Identity();
Eigen::RowVector3f pose_color(160.0/255, 32.0/255, 240.0/255);
Eigen::RowVector3f traj_color(0, 1.0, 0);
std::vector<std::array<float,6>> map_points;
std::vector<std::array<float,3>> mono_map_points;
std::vector<std::array<float,3>> traj_to_draw, traj_gt;
std::vector<Eigen::Matrix4d> key_frames;
PerfMonitor pose_fps;
void updatePoseForDrawing (Eigen::Matrix4d &pose) {
    pthread_mutex_lock(&pose_mtx);
    pose_to_draw = pose.cast<float>();
    std::array<float,3> tp{pose_to_draw.block<3,1>(0,3).coeff(0), pose_to_draw.block<3,1>(0,3).coeff(1),
                           pose_to_draw.block<3,1>(0,3).coeff(2)};
    traj_to_draw.push_back(tp);
    pose_fps.update();
    pthread_mutex_unlock(&pose_mtx);
}
void updatePoseForDrawing (Eigen::Vector3d &p, Eigen::Quaterniond &q) {
    Eigen::Matrix<float,4,4> cur_pose = Eigen::Matrix<float,4,4>::Identity();
    cur_pose.block<3,3>(0,0) = q.toRotationMatrix().cast<float>();
    cur_pose(0,3) = static_cast<float>(p.x());
    cur_pose(1,3) = static_cast<float>(p.y());
    cur_pose(2,3) = static_cast<float>(p.z());
    pthread_mutex_lock(&pose_mtx);
    pose_to_draw = cur_pose;
    std::array<float,3> tp{pose_to_draw.block<3,1>(0,3).coeff(0), pose_to_draw.block<3,1>(0,3).coeff(1),
                           pose_to_draw.block<3,1>(0,3).coeff(2)};
    traj_to_draw.push_back(tp);
    pose_fps.update();
    pthread_mutex_unlock(&pose_mtx);
}
void updateTestPoseForDrawing (Eigen::Matrix4f &pose) {
    pthread_mutex_lock(&pose_mtx);
    test_pose_to_draw = pose;
    std::array<float,3> tp{pose.block<3,1>(0,3).coeff(0), pose.block<3,1>(0,3).coeff(1),
                           pose.block<3,1>(0,3).coeff(2)};
//    test_traj_to_draw.push_back(tp);
    pthread_mutex_unlock(&pose_mtx);
}
void updatePoseColor(Eigen::RowVector3f &color) {
    pthread_mutex_lock(&pose_mtx);
    pose_color = color;
    pthread_mutex_unlock(&pose_mtx);
}
void updateTrajColor(Eigen::RowVector3f &color) {
    pthread_mutex_lock(&pose_mtx);
    traj_color = color;
    pthread_mutex_unlock(&pose_mtx);
}
void setTrajGroundtruth(std::vector<std::array<float,3>> &traj) {
    pthread_mutex_lock(&pose_mtx);
    traj_gt = traj;
    pthread_mutex_unlock(&pose_mtx);
}
void updateMapPoints(std::vector<std::array<float,6>> &map) {
    pthread_mutex_lock(&pose_mtx);
    map_points = map;
    pthread_mutex_unlock(&pose_mtx);
}
void addKeyFrame(Eigen::Matrix4d &pose) {
    pthread_mutex_lock(&pose_mtx);
    key_frames.push_back(pose);
    pthread_mutex_unlock(&pose_mtx);
}
void clearKeyFrames(){
    pthread_mutex_lock(&pose_mtx);
    key_frames.clear();
    pthread_mutex_unlock(&pose_mtx);
}
void appendMonoMapPoints(std::vector<std::array<float,3>> &pts) {
    pthread_mutex_lock(&pose_mtx);
    for (auto &pt : pts) {
        mono_map_points.push_back(pt);
    }
    pthread_mutex_unlock(&pose_mtx);
}
void clearVisualizationBuffers() {
    pthread_mutex_lock(&pose_mtx);
    map_points.clear();
    mono_map_points.clear();
    key_frames.clear();
    traj_to_draw.clear();
    traj_gt.clear();
    pose_to_draw = Eigen::Matrix4f::Identity();
    pthread_mutex_unlock(&pose_mtx);
}

/*-----------------------preview------------------------------*/
pthread_mutex_t preview_mtx = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t preview_cond = PTHREAD_COND_INITIALIZER;
cv::Mat preview_mat;
bool rotate;
void updatePreviewMat(cv::Mat mat, bool r) {
    pthread_mutex_lock(&preview_mtx);
    preview_mat = mat;
    rotate = r;
    pthread_cond_signal(&preview_cond);
    pthread_mutex_unlock(&preview_mtx);
}


/*----------------dataset player--------------------*/
pthread_mutex_t data_play_mtx = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t data_play_cond = PTHREAD_COND_INITIALIZER;

/*----------------simulation groundtruth--------------------*/
std::unordered_map<double, Eigen::Matrix4d> sim_twc_gts;
std::unordered_map<int, Eigen::Vector3d> sim_feats;
std::vector<Eigen::Vector3d> sim_feats_BA_debug;
Eigen::Matrix3d sim_relative_R;
Eigen::Vector3d sim_relative_t;
std::vector<double> sim_twc_ts;
std::unordered_map<int, Eigen::Vector3d> sim_Vbks;
double sim_scale;
std::vector<double> sim_ts_used;