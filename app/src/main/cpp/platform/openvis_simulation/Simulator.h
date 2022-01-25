/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_MSCKF_SIMULATOR_H
#define OV_MSCKF_SIMULATOR_H

#include <Eigen/Eigen>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>

// #include "core/VioManagerOptions.h"
#include "BsplineSE3.h"
// #include "utils/colors.h"
#include "camera_model.h"

using namespace ov_core;

namespace ov_msckf {

/**
 * @brief Master simulator class that generated visual-inertial measurements
 *
 * Given a trajectory this will generate a SE(3) @ref ov_core::BsplineSE3 for that trajectory.
 * This allows us to get the inertial measurement information at each timestep during this trajectory.
 * After creating the bspline we will generate an environmental feature map which will be used as our feature measurements.
 * This map will be projected into the frame at each timestep to get our "raw" uv measurements.
 * We inject bias and white noises into our inertial readings while adding our white noise to the uv measurements also.
 * The user should specify the sensor rates that they desire along with the seeds of the random number generators.
 *
 */
    class Simulator {

    public:
        /**
         * @brief Default constructor, will load all configuration variables
         * @param params_ VioManager parameters. Should have already been loaded from cmd.
         */
        // Simulator(VioManagerOptions &params_);
        Simulator(std::string &app_storage);

        /**
         * @brief Returns if we are actively simulating
         * @return True if we still have simulation data
         */
        bool ok() { return is_running; }

        /**
         * @brief Gets the timestamp we have simulated up too
         * @return Timestamp
         */
        double current_timestamp() { return timestamp; }

        /**
         * @brief Get the simulation state at a specified timestep
         * @param desired_time Timestamp we want to get the state at
         * @param imustate State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
         * @return True if we have a state
         */
        bool get_state(double desired_time, Eigen::Matrix<double, 17, 1> &imustate);

        /**
         * @brief Gets the next inertial reading if we have one.
         * @param time_imu Time that this measurement occured at
         * @param wm Angular velocity measurement in the inertial frame
         * @param am Linear velocity in the inertial frame
         * @return True if we have a measurement
         */
        bool get_next_imu(double &time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am);

        /**
         * @brief Gets the next inertial reading if we have one.
         * @param time_cam Time that this measurement occured at
         * @param camids Camera ids that the corresponding vectors match
         * @param feats Noisy uv measurements and ids for the returned time
         * @return True if we have a measurement
         */
        bool get_next_cam(double &time_cam, std::vector<int> &camids,
                          std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats);

        /// Returns the true 3d map of features
        std::unordered_map<size_t, Eigen::Vector3d> get_map() { return featmap; }

        /// Access function to get the true parameters (i.e. calibration and settings)
        // VioManagerOptions get_true_paramters() { return params; }

    public:
        /**
         * @brief This will load the trajectory into memory.
         * @param path_traj Path to the trajectory file that we want to read in.
         */
        void load_data(std::string path_traj);

        /**
         * @brief Projects the passed map features into the desired camera frame.
         * @param R_GtoI Orientation of the IMU pose
         * @param p_IinG Position of the IMU pose
         * @param camid Camera id of the camera sensor we want to project into
         * @param feats Our set of 3d features
         * @return True distorted raw image measurements and their ids for the specified camera
         */
        std::vector<std::pair<size_t, Eigen::VectorXf>>
        project_pointcloud(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG,
                           int camid, const std::unordered_map<size_t, Eigen::Vector3d> &feats);

        /**
         * @brief Will generate points in the fov of the specified camera
         * @param R_GtoI Orientation of the IMU pose
         * @param p_IinG Position of the IMU pose
         * @param camid Camera id of the camera sensor we want to project into
         * @param[out] feats Map we will append new features to
         * @param numpts Number of points we should generate
         */
        void
        generate_points(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG, int camid,
                        std::unordered_map<size_t, Eigen::Vector3d> &feats, int numpts);

        //===================================================================
        // Configuration variables
        //===================================================================

        /// True vio manager params (a copy of the parsed ones)
        // VioManagerOptions params;

        //===================================================================
        // State related variables
        //===================================================================

        /// Our loaded trajectory data (timestamp(s), q_GtoI, p_IinG)
        std::vector<Eigen::VectorXd> traj_data;

        /// Our b-spline trajectory
        BsplineSE3 spline;

        /// Our map of 3d features
        size_t id_map = 0;
        std::unordered_map<size_t, Eigen::Vector3d> featmap;

        /// Mersenne twister PRNG for measurements (IMU)
        std::mt19937 gen_meas_imu;

        /// Mersenne twister PRNG for measurements (CAMERAS)
        std::vector<std::mt19937> gen_meas_cams;

        /// Mersenne twister PRNG for state initialization
        std::mt19937 gen_state_init;

        /// Mersenne twister PRNG for state perturbations
        std::mt19937 gen_state_perturb;

        /// If our simulation is running
        bool is_running;

        //===================================================================
        // Simulation specific variables
        //===================================================================

        /// Current timestamp of the system
        double timestamp;

        /// Last time we had an IMU reading
        double timestamp_last_imu;

        /// Last time we had an CAMERA reading
        double timestamp_last_cam;

        /// Our running acceleration bias
        Eigen::Vector3d true_bias_accel = Eigen::Vector3d::Zero();

        /// Our running gyroscope bias
        Eigen::Vector3d true_bias_gyro = Eigen::Vector3d::Zero();

        // Our history of true biases
        std::vector<double> hist_true_bias_time;
        std::vector<Eigen::Vector3d> hist_true_bias_accel;
        std::vector<Eigen::Vector3d> hist_true_bias_gyro;

        // SIMULATOR ===============================

        /// Path to the trajectory we will b-spline and simulate on. Should be time(s),pos(xyz),ori(xyzw) format.
        std::string sim_traj_path = "../ov_data/sim/udel_gore.txt";

        /// We will start simulating after we have moved this much along the b-spline. This prevents static starts as we init from groundtruth in
        /// simulation.
        double sim_distance_threshold = 0.1;

        /// Frequency (Hz) that we will simulate our cameras
        double sim_freq_cam = 10.0;

        /// Frequency (Hz) that we will simulate our inertial measurement unit
        double sim_freq_imu = 400.0;

        /// Seed for initial states (i.e. random feature 3d positions in the generated map)
        int sim_seed_state_init = 0;

        /// Seed for calibration perturbations. Change this to perturb by different random values if perturbations are enabled.
        int sim_seed_preturb = 0;

        /// Measurement noise seed. This should be incremented for each run in the Monte-Carlo simulation to generate the same true measurements,
        /// but diffferent noise values.
        int sim_seed_measurements = 0;

        /// If we should perturb the calibration that the estimator starts with
        bool sim_do_perturbation = false;

        /// Number of distinct cameras that we will observe features in
        int num_cameras = 1;

        // STATE DEFAULTS ==========================

        /// Gravity magnitude in the global frame (i.e. should be 9.81 typically)
        double gravity_mag = 9.79235;

        /// Time offset between camera and IMU.
        double calib_camimu_dt = 0.0;

        /// Map between camid and camera model (true=fisheye, false=radtan)
        std::map<size_t, bool> camera_fisheye;

        /// Map between camid and intrinsics. Values depends on the model but each should be a 4x1 vector normally.
        std::map<size_t, Eigen::VectorXd> camera_intrinsics;

        /// Map between camid and camera extrinsics (q_ItoC, p_IinC).
        // std::map<size_t, Eigen::VectorXd> camera_extrinsics;
        Eigen::Matrix3d R_ItoC;
        Eigen::Matrix<double, 3, 1> p_IinC;

        /// Map between camid and the dimensions of incoming images (width/cols, height/rows). This is normally only used during simulation.
        std::map<size_t, std::pair<int, int>> camera_wh;

        /// The number of points we should extract and track in *each* image frame. This highly effects the computation required for tracking.
        int num_pts = 150;

        /// factor for convenient debug of imu noises
        double debug_factor = 1;

        /// Gyroscope white noise (rad/s/sqrt(hz))
        double sigma_w = 0.003083 * debug_factor;

        /// Gyroscope white noise covariance
        double sigma_w_2 = pow(sigma_w, 2);

        /// Gyroscope random walk (rad/s^2/sqrt(hz))
        double sigma_wb = 0.0001 * debug_factor;

        /// Gyroscope random walk covariance
        double sigma_wb_2 = pow(sigma_wb, 2);

        /// Accelerometer white noise (m/s^2/sqrt(hz))
        double sigma_a = 0.012491 * debug_factor;

        /// Accelerometer white noise covariance
        double sigma_a_2 = pow(sigma_a, 2);

        /// Accelerometer random walk (m/s^3/sqrt(hz))
        double sigma_ab = 0.001563 * debug_factor;

        /// Accelerometer random walk covariance
        double sigma_ab_2 = pow(sigma_ab, 2);

        /// Noise sigma for our raw pixel measurements
        double sigma_pix = 1;

        /// feature points depth range
        double d_min = 0.5;
        double d_max = 10;
        /// project feature points clip range
        double clip_min = 0.5;
        double clip_max = 15;

        bool use_stereo = false;

        CameraModel m_cam_model;
    };

} // namespace ov_msckf

#endif // OV_MSCKF_SIMULATOR_H
