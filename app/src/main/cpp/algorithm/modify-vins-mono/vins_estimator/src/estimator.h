#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>


#include "../../slam_api/obslam_api.h"
#include "utility/perf_monitor.h"

using namespace vins_estimator;

typedef void (*initial_result_callback) (std::vector<double> &ts_, std::vector<Eigen::Vector3d> &Ps_,
        std::vector<Eigen::Vector3d> &Vs_, std::vector<Eigen::Matrix3d> &Qs_, Eigen::Vector3d ba_,
        Eigen::Vector3d g, Eigen::Vector3d bg_, std::vector<shared_ptr<IntegrationBase>> &pre_integrations_,
        std::map<uint, std::vector<std::pair<uint, std::array<double, 6>>>> &features_, std::map<uint, Eigen::Vector3d> &world_pts_);

class Estimator
{
  public:
    Estimator();

    // void setParameter();

    // interface
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    bool processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header);
    // void setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r);

    // internal
    void clearState();
    // bool initialStructure();
    bool myInitialStructure();
    // bool visualInitialAlign();
    // bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    // void solveOdometry();
    void slideWindowNew();
    void slideWindowOld();
    // void optimization();
    // void vector2double();
    // void double2vector();
    // bool failureDetection();


    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    TimeLagMeasurer keyframe_timer_;
    Vector3d g;
    // MatrixXd Ap[2], backup_A;
    // VectorXd bp[2], backup_b;

    Matrix3d ric[NUM_OF_CAM];
    Vector3d tic[NUM_OF_CAM];

    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    // std_msgs::Header Headers[(WINDOW_SIZE + 1)];
    double Headers[(WINDOW_SIZE + 1)];

    // IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    shared_ptr<IntegrationBase> pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    FeatureManager f_manager;
    // MotionEstimator m_estimator;
    // InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    // MarginalizationInfo *last_marginalization_info = nullptr;
    // vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    // IntegrationBase *tmp_pre_integration;
    shared_ptr<IntegrationBase> tmp_pre_integration;

    //relocalization variable
    // bool relocalization_info;
    // double relo_frame_stamp;
    // double relo_frame_index;
    // int relo_frame_local_index;
    // vector<Vector3d> match_points;
    // double relo_Pose[SIZE_POSE];
    // Matrix3d drift_correct_r;
    // Vector3d drift_correct_t;
    // Vector3d prev_relo_t;
    // Matrix3d prev_relo_r;
    // Vector3d relo_relative_t;
    // Quaterniond relo_relative_q;
    // double relo_relative_yaw;

    // std::vector<Eigen::VectorXd> biases_save_;
    // vector<Eigen::VectorXd> extrinsic_save_;
    // vector<double> td_save_;
    // void load_td_bias_extrinsic();
    // std::string config_path_;
    //
    // MotionOnlyEstimator mo_estimator_;

    //slam api
    slam_status_callback statusCallback_;

    //initial module
    bool initial_finished_;
    initial_result_callback initial_callback_;
};
