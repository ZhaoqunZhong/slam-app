#include "estimator.h"
#include "glog/logging.h"
#include <memory>


Estimator::Estimator() : f_manager{pre_integrations} {
    // LOG(INFO) << "init begins ";
    clearState();
}
/*
void Estimator::load_td_bias_extrinsic() {
    // load td from vins optimization
    std::ifstream ftd;
    ftd.open(OUTPUT_PATH + "td_save.csv");
    if (!ftd) {
        LOG(WARNING) << "No td save found, using offline calibration value.";
    } else {
        std::string current_line;
        std::vector<double> tds;
        while (std::getline(ftd, current_line)) {
            tds.push_back(std::atof(current_line.c_str()));
        }
        if (tds.size() > 100) {
            td = tds.back();
            LOG(WARNING) << "Loaded td from saved file: " << td;
        }
    }
    ftd.close();

    // load biases from vins optimization
    bool load_bias_calibration = true;
    std::ifstream fbias;
    fbias.open(OUTPUT_PATH + "biases_save.csv");
    if (!fbias) {
        // LOG(WARNING) << "No biases save found, using zeros as init values.";
    } else {
        std::string current_line;
        std::vector<Eigen::VectorXd> biases;
        while (std::getline(fbias, current_line)) {
            // Loop variables
            int i = 0;
            std::istringstream s(current_line);
            std::string field;
            Eigen::Matrix<double, 6, 1> data;

            // Loop through this line (ba_x, ba_y, ba_z, bg_x, bg_y, bg_z)
            while (std::getline(s, field, ' ')) {
                // Skip if empty
                if (field.empty() || i >= data.rows())
                    continue;
                data(i) = std::atof(field.c_str());
                i++;
            }
            // Only a valid line if we have all the parameters
            if (i > 5) {
                biases.push_back(data);
            }
        }
        if (biases.size() > 100) {
            for (int j=0; j < WINDOW_SIZE + 1; j++) {
                Bas[j] = biases.back().topRows(3);
                Bgs[j] = biases.back().bottomRows(3);
            }
            LOG(WARNING) << "Loaded bias from saved file Ba: " << Bas[WINDOW_SIZE].transpose()
                << " Bg: " << Bgs[WINDOW_SIZE].transpose();
            load_bias_calibration = false;
        }
    }
    fbias.close();

    // load_bias_calibration = true;
    // load biases and g_norm from offset calibration
    if (load_bias_calibration) {
        cv::FileStorage cvfs(config_path_, cv::FileStorage::READ);
        std::vector<double> ba, bg;
        cvfs["acc_offsets"] >> ba;
        cvfs["gyr_offsets"] >> bg;
        cvfs.release();
        for (int j=0; j < WINDOW_SIZE + 1; j++) {
            Bas[j] = Eigen::Vector3d(ba[0], ba[1], ba[2]);
            Bgs[j] = Eigen::Vector3d(bg[0], bg[1], bg[2]);
        }
        LOG(WARNING) << "Loaded ba from offline calibration " << Bas[0].transpose();
        LOG(WARNING) << "Loaded bg from offline calibration " << Bgs[0].transpose();
    }

    return;

    //load extrinsic
    std::ifstream fext;
    fext.open(OUTPUT_PATH + "extrinsic_save.csv");
    if (!fext) {
        LOG(WARNING) << "No extrinsic save found, using calibration as init values.";
    } else {
        std::string current_line;
        std::vector<Eigen::VectorXd> extrinsics;
        while (std::getline(fext, current_line)) {
            // Loop variables
            int i = 0;
            std::istringstream s(current_line);
            std::string field;
            Eigen::Matrix<double, 7, 1> data;

            // Loop through this line (ba_x, ba_y, ba_z, bg_x, bg_y, bg_z)
            while (std::getline(s, field, ' ')) {
                // Skip if empty
                if (field.empty() || i >= data.rows())
                    continue;
                data(i) = std::atof(field.c_str());
                i++;
            }
            // Only a valid line if we have all the parameters
            if (i > 6) {
                extrinsics.push_back(data);
            }
        }
        if (extrinsics.size() > 100) {
            tic[0] = extrinsics.back().topRows(3);
            Eigen::Quaterniond qic;
            qic.coeffs() = extrinsics.back().bottomRows(4);
            ric[0] = qic.toRotationMatrix();
            LOG(INFO) << "Loaded extrinsic from saved file tic: " << tic[0].transpose()
                      << " ric: \n" << ric[0];
        }
    }
    fext.close();
}*/

/*void Estimator::setParameter() {
    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    // f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
    /// load td and biases from saved file
    // load_td_bias_extrinsic();
}*/

void Estimator::clearState() {
    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

/*        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;*/
        pre_integrations[i].reset();
    }

    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    for (auto &it : all_image_frame) {
/*        if (it.second.pre_integration != nullptr) {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }*/
        it.second.pre_integration.reset();
    }

    solver_flag = INITIAL;
    first_imu = false, sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;


    // if (last_marginalization_info != nullptr)
    //     delete last_marginalization_info;

    tmp_pre_integration.reset();
    // last_marginalization_info = nullptr;
    // last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;
    // relocalization_info = 0;

    // drift_correct_r = Matrix3d::Identity();
    // drift_correct_t = Vector3d::Zero();
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration,
                           const Vector3d &angular_velocity) {
    if (!first_imu) {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count]) {
/*        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count],
                                                            Bgs[frame_count]};*/
        pre_integrations[frame_count] = make_shared<IntegrationBase>(acc_0, gyr_0, Bas[frame_count],
                                                            Bgs[frame_count]);
    }

    if (frame_count != 0) {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

bool Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                             const double header) {
    // LOG(INFO) << "new image coming ------------------------------------------";
    // ROS_DEBUG("Adding feature points %lu", image.size());

    if (f_manager.addFeatureCheckParallax(frame_count, image, td)) {
        marginalization_flag = MARGIN_OLD;
        keyframe_timer_.restart();
    } else {
        marginalization_flag = MARGIN_SECOND_NEW;
    }
    if (keyframe_timer_.lagFromStartSecond() > 1) {
        marginalization_flag = MARGIN_OLD;
        keyframe_timer_.restart();
    }
    // LOG(INFO) << "marginalization_flag " << marginalization_flag << " ";
    // LOG(INFO) << "Add feature costs " << timer.lagFromStartSecond()*1e3 << " ms";
    // marginalization_flag = MARGIN_OLD; /// try always marginalize old

    // ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    // ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    // ROS_DEBUG("Solving %d", frame_count);
    // ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = make_shared<IntegrationBase>(acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]);

/*    if (ESTIMATE_EXTRINSIC == 2) {
        // ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0) {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1,
                                                                                 frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres,
                                                          pre_integrations[frame_count]->delta_q,
                                                          calib_ric)) {
                // ROS_WARN("initial extrinsic rotation calib success");
                // ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }*/

    if (solver_flag == INITIAL) {
        marginalization_flag = MARGIN_OLD; /// my initialization
        if (frame_count == WINDOW_SIZE) {
            bool result = false;
            TimeLagMeasurer timer;
            if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1) {
                // result = initialStructure();
/*                qspower::init();
                LOG(WARNING) << " Is qspower supported: " << qspower_is_supported();
                qspower::request_mode(qspower::mode::perf_burst);*/
                result = myInitialStructure();
/*                qspower::request_mode(qspower::mode::normal);
                qspower::terminate();*/
                initial_timestamp = header;
            }
            if (result) {
                // slam_status status = TRACKING;
                // statusCallback_(&status);
                initial_finished_ = true;
                /// construct initial callback
                std::vector<double> ts_(std::begin(Headers), std::end(Headers));
                std::vector<Vector3d> Ps_(std::begin(Ps), std::end(Ps));
                std::vector<Vector3d> Vs_(std::begin(Vs), std::end(Vs));
                std::vector<Matrix3d> Qs_(std::begin(Rs), std::end(Rs));
                std::vector<shared_ptr<IntegrationBase>> pre_integrations_(std::begin(pre_integrations), std::end(pre_integrations));
                std::map<uint, std::vector<std::pair<uint, std::array<double, 6>>>> features_;
                std::map<uint, Eigen::Vector3d> world_pts_;
                int f_cnt = 0;
                for (auto &it_per_id : f_manager.feature)
                {
                    it_per_id.used_num = it_per_id.feature_per_frame.size();
                    if (it_per_id.used_num >= 2 && it_per_id.estimated_depth > 0 && it_per_id.estimated_depth < 50) {
                        f_cnt++;
                        // LOG(WARNING) << "DEBUG f_id " << it_per_id.feature_id << " estimated_depth " << it_per_id.estimated_depth
                        //             << " f_cnt " << f_cnt;
                        std::vector<std::pair<uint, std::array<double, 6>>> feature_in_frames;
                        uint frame_id = static_cast<uint>(it_per_id.start_frame);
                        for (auto & ff : it_per_id.feature_per_frame) {
                            std::array<double,6> cur_f;
                            cur_f[0] = ff.uv.x();
                            cur_f[1] = ff.uv.y();
                            cur_f[2] = ff.point.x();
                            cur_f[3] = ff.point.y();
                            cur_f[4] = ff.velocity.x();
                            cur_f[5] = ff.velocity.y();
                            feature_in_frames.push_back(std::make_pair(frame_id, cur_f));

                            // world pt
                            if (frame_id == static_cast<uint>(it_per_id.start_frame)) {
                                // LOG(WARNING) << "DEBUG frame_id " << frame_id << " start_frame " << static_cast<uint>(it_per_id.start_frame);
                                Eigen::Vector3d frame_pt {cur_f[2] * it_per_id.estimated_depth,
                                                          cur_f[3] * it_per_id.estimated_depth,
                                                          it_per_id.estimated_depth};
                                Eigen::Vector3d world_pt = Rs[frame_id] * ric[0] * frame_pt + (Rs[frame_id] * tic[0] + Ps[frame_id]);
                                world_pts_[it_per_id.feature_id] = world_pt;
                            }

                            frame_id++;
                        }
                        features_[it_per_id.feature_id] = feature_in_frames;
                    }
                }
                // LOG(WARNING) << "DEBUG world pts size " << world_pts_.size();
                initial_callback_(ts_, Ps_, Vs_, Qs_, Bas[0], g, Bgs[0], pre_integrations_, features_, world_pts_);
                LOG(WARNING) << "--- Initialization costs " << timer.lagFromStartSecond()*1e3 << " ms ---";
                return true;
/*                solver_flag = NON_LINEAR;
                solveOdometry();
                // f_manager.removeFailures();
                /// Update states for motion only estimator
                mo_estimator_.updateBackend(Rs, Ps,Vs[WINDOW_SIZE],Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE],
                                            Headers[WINDOW_SIZE], td, f_manager.feature);
                slideWindow();
                // ROS_INFO("Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];*/

            } else
                slideWindow();
        } else
            frame_count++;
    } else {
        /*vins_estimator::TicToc t_solve;
        solveOdometry();
        LOG(INFO) << "solver costs: " << t_solve.toc() << " ms";
        // f_manager.removeFailures();

        if (failureDetection()) {
            LOG(ERROR) << "Failure detected, system reboot.";
            failure_occur = 1;
            clearState();
            setParameter();
            return;
        }

        /// Update states for motion only estimator
        mo_estimator_.updateBackend(Rs, Ps,Vs[WINDOW_SIZE],Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE],
                                    Headers[WINDOW_SIZE], td, f_manager.feature);
        // vins_estimator::TicToc t_margin;
        slideWindow();
        // LOG(INFO) << "marginalization costs: " << t_margin.toc() << " ms";
        // prepare output of VINS
*//*        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);*//*

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];*/
    }
    return false;
}
/*

bool Estimator::initialStructure() {
    vins_estimator::TicToc t_sfm;
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++;
             frame_it != all_image_frame.end(); frame_it++) {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int) all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++;
             frame_it != all_image_frame.end(); frame_it++) {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int) all_image_frame.size() - 1));
        //// ROS_WARN("IMU variation %f!", var);
        if (var < 0.25) {
            // ROS_INFO("IMU excitation not enouth!");
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature) {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(
                    make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    }
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l)) {
        // ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if (!sfm.construct(frame_count + 1, Q, T, l,
                       relative_R, relative_T,
                       sfm_f, sfm_tracked_points)) {
        // ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin();
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++) {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if ((frame_it->first) == Headers[i]) {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if ((frame_it->first) > Headers[i]) {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = -R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points) {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second) {
                it = sfm_tracked_points.find(feature_id);
                if (it != sfm_tracked_points.end()) {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if (pts_3_vector.size() < 6) {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            // ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1)) {
            // ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp, tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else {
        // ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}
*/

#define AnalyticVisualBA

bool Estimator::myInitialStructure() {
    TicToc t_sfm;
    /// check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.5)
        {
            LOG(ERROR) << "IMU excitation not enouth!";
            return false;
        }
    }
    /// Prepare feature data structure for visual BA
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        if (it_per_id.feature_per_frame.size() < 2)
            continue;
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    }
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    vector<pair<Vector3d, Vector3d>> corres_l;
    /// Find out which previous frame has the largest rotation-compensated parallax with the last frame in the window.
    {
        double min_inner_product = 2;
        // Only use the features that can be seen from all the frames to calculate parallax.
        std::unordered_set<int> id_set;
        vector<int> ids;
        for (auto &it : f_manager.feature)
        {
            if (it.start_frame <= 0 && it.endFrame() >= WINDOW_SIZE)
            {
                id_set.insert(it.feature_id);
                ids.push_back(it.feature_id);
            }
        }
        if (id_set.size() < 20) {
            LOG(ERROR) << "Not enough shared features across the window, slide window and try again.";
            return false;
        } else {
            LOG_ASSERT(id_set.size() == ids.size()) << "vector and unordered_set are of equal size";
            LOG(WARNING) << "Shared feature number across the window: " << ids.size();
        }

        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            Eigen::Quaterniond q_i_wz{1,0,0,0};
            for (int k = i+1; k < WINDOW_SIZE+1; k++) {
                q_i_wz *= pre_integrations[k]->delta_q;
            }
            Eigen::Matrix3d Rc_i_wz = RIC[0]* q_i_wz * RIC[0].transpose();
            vector<pair<Vector3d, Vector3d>> corres;
            for (auto &it : f_manager.feature)
            {
                if (id_set.find(it.feature_id) == id_set.end())
                    continue;
                Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
                int idx_l = i - it.start_frame;
                int idx_r = WINDOW_SIZE - it.start_frame;
                a = it.feature_per_frame[idx_l].point;
                b = it.feature_per_frame[idx_r].point;
                corres.push_back(make_pair(a, b));
            }

            double sum_inner_product = 0;
            double average_inner_product;
            for (int j = 0; j < int(corres.size()); j++) {
                Vector3d vec_0 = corres[j].first.normalized();
                Vector3d vec_1 = (Rc_i_wz * corres[j].second).normalized();
                double inner_product = vec_0.transpose() * vec_1;
                sum_inner_product += inner_product;
            }
            average_inner_product = 1.0 * sum_inner_product / int(corres.size());
            // LOG(INFO) << "average inner product "<< i <<" " <<average_inner_product;
            if (average_inner_product < min_inner_product) {
                min_inner_product = average_inner_product;
                l = i;
                corres_l = corres;
                relative_R = Rc_i_wz;
            }

         }

        if (l > 2) {
            LOG(WARNING) << "Initial structure solve relative pose l index > 2, slide window.";
            return false;
        } else {
            LOG(WARNING) << "Initial structure solve relative pose l index: " << l;
        }
    }
    /// Solve relative R,t problem with corres_l
    {
        if (corres_l.size() >= 20) {
            int sz = corres_l.size();
            Eigen::MatrixXd normals(3, sz);
            Vector3d sum_norms = Vector3d::Zero();
            double min_inner_product = 2;
            Vector3d fi_minus_Rij_fj; // Use the one with the largest magnitude
            for (int i = 0; i < sz; i++)
            {
                Vector3d vec_0 = corres_l[i].first.normalized();
                Vector3d vec_1 = (relative_R * corres_l[i].second).normalized();
                if (vec_0.transpose() * vec_1 < min_inner_product) {
                    min_inner_product = vec_0.transpose() * vec_1;
                    fi_minus_Rij_fj = vec_0 - vec_1;
                }
                // Vector3d norm = vec_0.cross(vec_1).normalized(); /// normalize norm vectors
                Vector3d norm = vec_0.cross(vec_1); /// un-normalized norm vectors
                normals.block<3,1>(0, i) = norm;
                sum_norms += norm;
            }
            // LOG(INFO) << "norms\n" << normals;
            Vector3d avg_norm = sum_norms / sz;
            // LOG(INFO) << "avg_norm\n " << avg_norm;
            MatrixXd centered_norms = normals.colwise() - avg_norm; /// centered PCA
            // MatrixXd centered_norms = normals; /// un-centered PCA
            // LOG(INFO) << "centered norm\n " << centered_norms;
            SelfAdjointEigenSolver<MatrixXd> es;
            es.compute(centered_norms * centered_norms.transpose());
            Vector3d eigen_values = es.eigenvalues().cwiseAbs();
            // LOG(INFO) << "eigen values " << eigen_values.transpose();
            int k;
            eigen_values.minCoeff(&k);
            Vector3d trans = es.eigenvectors().col(k);
            // LOG(INFO) << "eigen vectors\n " << es.eigenvectors();
            // LOG(INFO) << "trans " << trans.transpose();
            relative_T = trans;
            if (trans.dot(fi_minus_Rij_fj) < 0) {
                relative_T = -trans;
            }
            LOG(INFO) << "Rotation euler angles " << relative_R.eulerAngles(0,1,2).transpose() * 180/M_PI;
            LOG(INFO) << "Translation " << relative_T.transpose();
        }
    }
    /// Triangulate the corres features and solve Pnp for other frames' pose
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    GlobalSFM sfm;
    sfm.feature_num = sfm_f.size();
    // T_wc
    Q[l].w() = 1;
    Q[l].x() = 0;
    Q[l].y() = 0;
    Q[l].z() = 0;
    T[l].setZero();
    Q[frame_count] = Q[l] * Quaterniond(relative_R);
    T[frame_count] = relative_T;
    //cout << "init q_l " << Q[l].w() << " " << Q[l].vec().transpose() << endl;
    //cout << "init t_l " << T[l].transpose() << endl;

    //rotate to cam frame
    Matrix3d c_Rotation[frame_count + 1];
    Vector3d c_Translation[frame_count + 1];
    Quaterniond c_Quat[frame_count + 1];
    double c_rotation[frame_count + 1][4];
    double c_translation[frame_count + 1][3];
    Eigen::Matrix<double, 3, 4> Pose[frame_count + 1];

    // T_cw, triangulation needs Tcw, so inverse Twc.
    c_Quat[l] = Q[l].inverse();
    c_Rotation[l] = c_Quat[l].toRotationMatrix();
    c_Translation[l] = -1 * (c_Rotation[l] * T[l]);
    Pose[l].block<3, 3>(0, 0) = c_Rotation[l];
    Pose[l].block<3, 1>(0, 3) = c_Translation[l];

    c_Quat[frame_count] = Q[frame_count].inverse();
    c_Rotation[frame_count] = c_Quat[frame_count].toRotationMatrix();
    c_Translation[frame_count] = -1 * (c_Rotation[frame_count] * T[frame_count]);
    Pose[frame_count].block<3, 3>(0, 0) = c_Rotation[frame_count];
    Pose[frame_count].block<3, 1>(0, 3) = c_Translation[frame_count];

    for (int i = l; i < frame_count ; i++)
    {
        if (i == l) {
            sfm.triangulateTwoFrames(i, Pose[i], frame_count, Pose[frame_count], sfm_f);
        } else {
            // solve pnp
            Matrix3d R_initial = c_Rotation[i - 1];
            Vector3d P_initial = c_Translation[i - 1];
            // Turns out that pnp is sensitive to initial values.
            if(!sfm.solveFrameByPnP(R_initial, P_initial, i, sfm_f)) {
                LOG(WARNING) << "Solve pnp failed.";
                return false;
            }
            c_Rotation[i] = R_initial;
            c_Translation[i] = P_initial;
            c_Quat[i] = c_Rotation[i];
            Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
            Pose[i].block<3, 1>(0, 3) = c_Translation[i];
        }
    }
    for (int i = l - 1; i >= 0; i--)
    {
        //solve pnp
        Matrix3d R_initial = c_Rotation[i + 1];
        Vector3d P_initial = c_Translation[i + 1];
        if(!sfm.solveFrameByPnP(R_initial, P_initial, i, sfm_f)){
            LOG(WARNING) << "Solve pnp failed.";
            return false;
        }
        c_Rotation[i] = R_initial;
        c_Translation[i] = P_initial;
        c_Quat[i] = c_Rotation[i];
        Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
        Pose[i].block<3, 1>(0, 3) = c_Translation[i];
    }

#ifndef AnalyticVisualBA
    // auto sfm_f_copy = sfm_f;
    /// Visual BA using ceres::AutoDiffCostFunction
    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
    LOG(INFO) << " begin Visual BA " << endl;
    for (int i = 0; i < frame_count + 1; i++)
    {
        //double array for ceres
        c_translation[i][0] = c_Translation[i].x();
        c_translation[i][1] = c_Translation[i].y();
        c_translation[i][2] = c_Translation[i].z();
        c_rotation[i][0] = c_Quat[i].w();
        c_rotation[i][1] = c_Quat[i].x();
        c_rotation[i][2] = c_Quat[i].y();
        c_rotation[i][3] = c_Quat[i].z();
//		LOG(INFO) <<"original quaternion "<<c_Quat[i].coeffs().transpose();
//		LOG(INFO) <<"original translation "<<c_Translation[i].transpose();
        problem.AddParameterBlock(c_rotation[i], 4, local_parameterization);
        problem.AddParameterBlock(c_translation[i], 3);
        if (i == l)
        {
            problem.SetParameterBlockConstant(c_rotation[i]);
        }
        if (i == l || i == frame_count)
        {
            problem.SetParameterBlockConstant(c_translation[i]);
        }
    }
    for (int i = 0; i < sfm_f.size(); i++)
    {
        if (sfm_f[i].state != true)
            continue;

        for (int j = 0; j < int(sfm_f[i].observation.size()); j++)
        {
            int k = sfm_f[i].observation[j].first;
            ceres::CostFunction* cost_function = ReprojectionError3D::Create(
                    sfm_f[i].observation[j].second.x(),
                    sfm_f[i].observation[j].second.y());

            problem.AddResidualBlock(cost_function, loss_function, c_rotation[k], c_translation[k],
                                     sfm_f[i].position);
        }
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.minimizer_progress_to_stdout = true;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 100;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(WARNING) << "visual BA result " << summary.FullReport();
    if (summary.termination_type == ceres::CONVERGENCE || (summary.final_cost > 0 && summary.final_cost < 2e-03) )
    {
        LOG(WARNING) << "visual BA converge" << endl;
    } else {
        LOG(WARNING) << "visual BA not converge " << endl;
        return false;
    }
    for (int i = 0; i < frame_count + 1; i++)
    {
        Q[i].w() = c_rotation[i][0];
        Q[i].x() = c_rotation[i][1];
        Q[i].y() = c_rotation[i][2];
        Q[i].z() = c_rotation[i][3];
        Q[i] = Q[i].inverse();
        // LOG(INFO) << "final  Q" << " i " << i <<"  " <<Q[i].w() << "  " << Q[i].vec().transpose() << endl;
    } /// Q[i] are now R_c0 _ci
    for (int i = 0; i < frame_count + 1; i++)
    {
        T[i] = -1 * (Q[i] * Vector3d(c_translation[i][0], c_translation[i][1], c_translation[i][2]));
        // LOG(INFO) << "final  t" << " i " << i <<"  " << T[i](0) <<"  "<< T[i](1) <<"  "<< T[i](2) << endl;
    } /// T[i] are now T_c0 _ci

#else
    /// visual BA using ceres::sizedCostFunction
    {
        // sfm_f = sfm_f_copy;
        ceres::Problem problem;
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        ceres::LossFunction *loss_function;
        // loss_function = new ceres::HuberLoss(1.0);
        loss_function = new ceres::CauchyLoss(1.0);
        vector<array<double, 7>> poses_cw;
        poses_cw.resize(frame_count + 1);
        for (int i = 0; i < frame_count + 1; i++) {
            poses_cw[i][0] = c_Translation[i].x();
            poses_cw[i][1] = c_Translation[i].y();
            poses_cw[i][2] = c_Translation[i].z();
            poses_cw[i][3] = c_Quat[i].x();
            poses_cw[i][4] = c_Quat[i].y();
            poses_cw[i][5] = c_Quat[i].z();
            poses_cw[i][6] = c_Quat[i].w();
            problem.AddParameterBlock(poses_cw[i].data(), 7, local_parameterization);
            if (i == l) {
                problem.SetParameterBlockConstant(poses_cw[i].data());
            }
/*            if (i == frame_count) {
                std::vector<int> const_indices{0, 1, 2};
                ceres::SubsetParameterization *subset_parameterization = new ceres::SubsetParameterization(7, const_indices);
                problem.SetParameterization(poses_cw[i].data(), subset_parameterization);
            }*/
        }
        for (int i = 0; i < sfm_f.size(); i++) {
            if (sfm_f[i].state != true)
                continue;

            for (int j = 0; j < int(sfm_f[i].observation.size()); j++) {
                int k = sfm_f[i].observation[j].first;
                Vector3d pt_norm(sfm_f[i].observation[j].second.x(), sfm_f[i].observation[j].second.y(), 1);
                ReprojectionAnalytic *f = new ReprojectionAnalytic(pt_norm);
                problem.AddResidualBlock(f, loss_function, poses_cw[k].data(), sfm_f[i].position);
            }
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        // options.minimizer_progress_to_stdout = true;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.max_num_iterations = 100;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        LOG(WARNING) << "visual BA result " << summary.FullReport();
        if (summary.termination_type == ceres::CONVERGENCE || (summary.final_cost > 0 && summary.final_cost < 2e-03)) {
            LOG(WARNING) << "visual BA converge" << endl;
        } else {
            LOG(WARNING) << "visual BA not converge " << endl;
            return false;
        }
        for (int i = 0; i < frame_count + 1; i++) {
            Q[i].w() = poses_cw[i][6];
            Q[i].x() = poses_cw[i][3];
            Q[i].y() = poses_cw[i][4];
            Q[i].z() = poses_cw[i][5];
            Q[i] = Q[i].inverse();
            T[i] = -1 * (Q[i] * Vector3d(poses_cw[i][0], poses_cw[i][1], poses_cw[i][2]));
        }
    }
#endif
    // map<int, Vector3d> sfm_tracked_points;
    unordered_map<int, double> sfm_tracked_points;
    for (int i = 0; i < (int)sfm_f.size(); i++)
    {
        if(sfm_f[i].state) {
            // sfm_tracked_points[sfm_f[i].id] = Vector3d(sfm_f[i].position[0], sfm_f[i].position[1], sfm_f[i].position[2]);
            // calculate estimated depth from BA
            int k = sfm_f[i].observation.front().first;
            Eigen::Quaterniond q_ck_c0 = Q[k].inverse();
            Eigen::Vector3d t_ck_c0 = -1 * (q_ck_c0 * T[k]);
            Eigen::Vector3d p_ck = q_ck_c0 * Vector3d(sfm_f[i].position[0], sfm_f[i].position[1], sfm_f[i].position[2]) + t_ck_c0;
            sfm_f[i].depth = p_ck.z();
            sfm_tracked_points[sfm_f[i].id] = p_ck.z();
        }
    }

    map<double, ImageFrame> visual_frames;
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++) {
        if ((frame_it->first) == Headers[i]) {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose(); //R_c0_bk
            frame_it->second.T = T[i]; // t_c0_ck
            visual_frames.insert(make_pair(frame_it->first, frame_it->second));
            i++;
        }
    } // second.R is now R_c0_bi, second.T is now T_c0_ci

    /// Visual inertial alignment
    /*
    VectorXd x;
    int all_frame_count = visual_frames.size();
    LOG(INFO) << "visual_frames size " << all_frame_count;
    int n_state = all_frame_count * 3 + 3 + 1;
    int m = (all_frame_count - 1) * 2 * 3;
    if ( m < n_state) {
        LOG(ERROR) << "Linear alignment matrix has less rows than columns.";
        return false;
    }
    MatrixXd A{m, n_state};
    A.setZero();
    VectorXd b{m};
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    for (frame_i = visual_frames.begin(); next(frame_i) != visual_frames.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);

        MatrixXd tmp_A(6, 10);
        tmp_A.setZero();
        VectorXd tmp_b(6);
        tmp_b.setZero();

        double dt = frame_j->second.pre_integration->sum_dt;
        tmp_A.block<3, 3>(0, 0) = frame_i->second.R.transpose() * dt * dt / 2;
        tmp_A.block<3, 3>(0, 3) = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = Eigen::Matrix3d::Zero();
        tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T);
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0];

        tmp_A.block<3, 3>(3, 0) = frame_i->second.R.transpose() * dt;
        tmp_A.block<3, 3>(3, 3) = -Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * frame_j->second.R;
        tmp_A.block<3, 1>(3, 9) = Eigen::Vector3d::Zero();
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;

        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Identity();
        /// weighted with imu pre-integration covariance
        // Matrix<double, 6, 6> cov = frame_j->second.pre_integration->covariance.block<6, 6>(0, 0);
        // LOG_FIRST_N(INFO, 1) << "pre_integration covariance alpha and beta part\n"<< cov ;
        // cov_inv = cov.inverse();
        // cov_inv.bottomRightCorner<3,3>() = Eigen::Matrix3d::Identity() * 10;
        // LOG_FIRST_N(INFO, 1) << "cov_inv \n" << cov_inv;
        tmp_A = cov_inv * tmp_A;
        tmp_b = cov_inv * tmp_b;

        A.block<6, 3>(i * 6, 0) += tmp_A.block<6,3>(0, 0);
        A.block<6, 3>(i * 6, 3 + 3 * i) += tmp_A.block<6,3>(0, 3);
        A.block<6, 3>(i * 6, 6 + 3 * i) += tmp_A.block<6,3>(0, 6);
        A.block<6, 1>(i * 6, n_state - 1) += tmp_A.block<6,1>(0, 9);
        b.segment<6>(i * 6) += tmp_b;
    }
    JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
    x = svd.solve(b); // x = [g v0...vk s]
    g = x.segment<3>(0);
    LOG(WARNING) << "g norm: " << g.norm() << " value: " << g.transpose();
    // Direct scale g norm for now.
    g = g * G.norm() / g.norm();
    double s = (x.tail<1>())(0);
    LOG(WARNING) << "s " << s;
    if (s < 0) {
        LOG(ERROR) << "Scale has minus sign, initialization failed.";
        return false;
    }*/
    /// Visual inertial alignment with imu biases
    VectorXd x;
    int all_frame_count = visual_frames.size();
    LOG(INFO) << "visual_frames size " << all_frame_count;
    // int n_state = all_frame_count * 3 + 3 + 1 + 6; //43
    int n_state = all_frame_count * 3 + 3 + 1 + 3; //without acc bias
    int m = (all_frame_count - 1) * 3 * 3; //90
    if ( m < n_state) {
        LOG(ERROR) << "Linear alignment matrix has less rows than columns.";
        return false;
    }
    MatrixXd A{m, n_state};
    A.setZero();
    VectorXd b{m};
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    for (frame_i = visual_frames.begin(); next(frame_i) != visual_frames.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);

        // MatrixXd tmp_A(9, 16);
        MatrixXd tmp_A(9, 13);//without acc bias
        tmp_A.setZero();
        VectorXd tmp_b(9);
        tmp_b.setZero();

        Eigen::Matrix3d dp_dba = frame_j->second.pre_integration->jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = frame_j->second.pre_integration->jacobian.block<3, 3>(O_P, O_BG);
        Eigen::Matrix3d dq_dbg = frame_j->second.pre_integration->jacobian.block<3, 3>(O_R, O_BG);
        Eigen::Matrix3d dv_dba = frame_j->second.pre_integration->jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = frame_j->second.pre_integration->jacobian.block<3, 3>(O_V, O_BG);

        double dt = frame_j->second.pre_integration->sum_dt;
        tmp_A.block<3, 3>(0, 0) = frame_i->second.R.transpose() * dt * dt / 2;
        tmp_A.block<3, 3>(0, 3) = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = Eigen::Matrix3d::Zero();
        tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T);
        // tmp_A.block<3, 3>(0, 10) = -dp_dba;
        // tmp_A.block<3, 3>(0, 13) = -dp_dbg;
        tmp_A.block<3, 3>(0, 10) = -dp_dbg;//without acc bias
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0];

        tmp_A.block<3, 3>(3, 0) = frame_i->second.R.transpose() * dt;
        tmp_A.block<3, 3>(3, 3) = -Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * frame_j->second.R;
        tmp_A.block<3, 1>(3, 9) = Eigen::Vector3d::Zero();
        // tmp_A.block<3, 3>(3, 10) = -dv_dba;
        // tmp_A.block<3, 3>(3, 13) = -dv_dbg;
        tmp_A.block<3, 3>(3, 10) = -dv_dbg;//without acc bias
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;

        // tmp_A.block<3, 3>(6, 13) = dq_dbg / 2;
        tmp_A.block<3, 3>(6, 10) = dq_dbg / 2; //without acc bias
        tmp_b.block<3, 1>(6, 0) = (frame_j->second.pre_integration->delta_q.inverse() *
                Eigen::Quaterniond(frame_i->second.R.transpose() * frame_j->second.R)).vec();

        /// weighted with imu pre-integration covariance
/*        Matrix<double, 9, 9> cov = frame_j->second.pre_integration->covariance.block<9, 9>(0, 0);
        Matrix<double, 9, 9> cov_inv = cov.inverse();
        Matrix<double, 9, 9> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 9, 9>>(cov_inv).matrixL().transpose();
        LOG_FIRST_N(INFO, 1) << "sqrt_info \n" << sqrt_info;
        tmp_A = sqrt_info * tmp_A * 1e-5;
        tmp_b = sqrt_info * tmp_b * 1e-5;*/

        A.block<9, 3>(i * 9, 0) += tmp_A.block<9,3>(0, 0); // g part
        A.block<9, 3>(i * 9, 3 + 3 * i) += tmp_A.block<9,3>(0, 3); // vb_k part
        A.block<9, 3>(i * 9, 6 + 3 * i) += tmp_A.block<9,3>(0, 6); // vb_k+1 part
        A.block<9, 1>(i * 9, n_state - 7) += tmp_A.block<9,1>(0, 9); // s part
        // A.block<9, 3>(i * 9, n_state - 6) += tmp_A.block<9,3>(0, 10); // ba part
        // A.block<9, 3>(i * 9, n_state - 3) += tmp_A.block<9,3>(0, 13); // bg part
        A.block<9, 3>(i * 9, n_state - 3) += tmp_A.block<9,3>(0, 10); // bg part, without acc bias
        b.segment<9>(i * 9) += tmp_b;
    }
    JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
    x = svd.solve(b); // x = [g v0...vk s ba bg]
    LOG(WARNING) << "Ax - b norm: " << (A*x - b).norm();
    g = x.segment<3>(0);
    LOG(WARNING) << "g norm: " << g.norm() << " value: " << g.transpose();
    // Direct scale g norm for now.
    g = g * G.norm() / g.norm();
    double s = (x.segment<1>(n_state -7))(0);
    LOG(WARNING) << "s " << s;
    if (s < 0) {
        LOG(ERROR) << "Scale has minus sign, initialization failed.";
        return false;
    }
/*    Eigen::Vector3d ba = x.segment<3>(n_state -6);
    Eigen::Vector3d bg = x.segment<3>(n_state -3);
    if (ba.norm() > 1 || bg.norm() > 1) {
        LOG(ERROR) << "Imu biases result too large, initialization failed.";
        LOG(ERROR) << "ba " << ba.transpose() << " bg " << bg.transpose();
        LOG(ERROR) << "g + ba in body_k: " << (g + ba).transpose() << " with norm: " << (g + ba).norm();
        // return false;
    }
    LOG(WARNING) << "ba " << ba.transpose() << " bg " << bg.transpose();*/
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg = x.segment<3>(n_state -3);
    if (bg.norm() > 1) {
        LOG(ERROR) << "Imu biases result too large, initialization failed.";
        LOG(ERROR) << " bg " << bg.transpose();
        // return false;
    }
    LOG(WARNING) << " bg " << bg.transpose();
    for (frame_i = visual_frames.begin(); next(frame_i) != visual_frames.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(ba, bg);
    }


    ///linear refine g and biases
    // RefineGravityAndBias(visual_frames, g, ba, bg);

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = visual_frames[Headers[i]].R; // R_c0_bi
        Vector3d Pi = visual_frames[Headers[i]].T; // t_c0_ci
        Ps[i] = Pi;
        Rs[i] = Ri;
        visual_frames[Headers[i]].is_key_frame = true;
    }
    int j = -1;
    for (frame_i = visual_frames.begin(); frame_i != visual_frames.end(); frame_i++)
    {
        j++;
        Vs[j] = x.segment<3>((j + 1) * 3); //vb_i
    }
    /// Inertial only optimization
#if 1
    ceres::Problem Inertial_only_op;
    ceres::LocalParameterization *R_local_parameterization = new RotationMatrixLocalParameterization();
    Eigen::Matrix3d Rg = Eigen::Matrix3d::Identity();
    Inertial_only_op.AddParameterBlock(Rg.data(), 9, R_local_parameterization);
    i = 0;
    for (frame_i = visual_frames.begin(); next(frame_i) != visual_frames.end(); frame_i++,i++) {
        frame_j = next(frame_i);
        InertialOnlyConstraint *f = new InertialOnlyConstraint(frame_j->second.pre_integration, Rs[i],
                                                               Rs[i+1], Ps[i], Ps[i+1], g, TIC[0]);
        Inertial_only_op.AddResidualBlock(f, nullptr, Rg.data(), Vs[i].data(), Vs[i+1].data(), &s, ba.data(), bg.data());
    }
    // fix ba to 0
    // Inertial_only_op.SetParameterBlockConstant(ba.data());
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 100;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &Inertial_only_op, &summary);
    LOG(WARNING) << "DEBUG Inertial only optimization result " << summary.FullReport();
    // update gravity
    g = Rg * g;
    LOG(WARNING) << "DEBUG g " << g.transpose();
    LOG(WARNING) << "DEBUG acc bias " << ba.transpose() << " norm: " << ba.norm();
    LOG(WARNING) << "DEBUG gyr bias " << bg.transpose();
    LOG(WARNING) << "DEBUG s " << s;

    for (frame_i = visual_frames.begin(); next(frame_i) != visual_frames.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(ba, bg);
    }
#endif
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        Bas[i] = ba;
        Bgs[i] = bg;
    }

/*    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);*/

    // Use BA result of feature depths instead of triangulating all over.
    int ba_feature_cnt = 0;
    for (auto &it_per_id : f_manager.feature)
    {
        if (sfm_tracked_points.find(it_per_id.feature_id) != sfm_tracked_points.end()) {
            it_per_id.estimated_depth = sfm_tracked_points[it_per_id.feature_id];
            // LOG(WARNING) << "DEBUG ba feature " << ++ba_feature_cnt << " estimated_depth " << it_per_id.estimated_depth * s;
        }
    }

    //triangulate on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for (int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    // ric[0] = RIC[0];
    // f_manager.setRic(ric);
    f_manager.triangulate(Ps, Rs, &(TIC_TMP[0]), &(RIC[0]));

    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2/*used_num >= 2 debug*/ && it_per_id.start_frame < WINDOW_SIZE/*WINDOW_SIZE -2 debug*/))
            continue;
        it_per_id.estimated_depth *= s;
    }

    // Ps from t_c0_ci to t_c0_bi, then make b0 the new origin
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);

    int kv = -1;
    for (frame_i = visual_frames.begin(); frame_i != visual_frames.end(); frame_i++)
    {
        kv++;
        // Vs[kv] = frame_i->second.R * x.segment<3>((kv + 1) * 3); // Vs are relative to c0 now
        Vs[kv] = frame_i->second.R * Vs[kv];
    }

    Matrix3d R0 = Utility::g2R(g); //Rotation from world to c0
    double yaw = Utility::R2ypr(R0 * Rs[0]).x(); // yaw from world to b0
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0; //Rotation from world to c0, but in 'rotation from world to b0' yaw = 0 case.
    g = R0 * g; // g in world coord
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    LOG(WARNING) << "Initialization successful!";
    return true;
}
/*
bool Estimator::visualInitialAlign() {
    vins_estimator::TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if (!result) {
        // ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++) {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);

    //triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for (int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    // f_manager.setRic(ric);
    f_manager.triangulate(Ps, Rs, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++) {
        if (frame_i->second.is_key_frame) {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    for (auto &it_per_id : f_manager.feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2*//*used_num >= 2 debug*//* && it_per_id.start_frame < WINDOW_SIZE*//*WINDOW_SIZE -2 debug*//*))
            continue;
        it_per_id.estimated_depth *= s;
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++) {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    // ROS_DEBUG_STREAM("g0     " << g.transpose());
    // ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose()); 

    return true;
}*/

/*
bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l) {
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++) {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20) {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++) {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if (average_parallax * 460 > 30 &&
                m_estimator.solveRelativeRT(corres, relative_R, relative_T)) {
                l = i;
                // ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::solveOdometry() {
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR) {
        vins_estimator::TicToc t_tri;
        f_manager.triangulate(Ps, Rs, tic, ric);
        LOG(INFO) << "triangulation costs " << t_tri.toc() << " ms";
        optimization();
    }
}*/
/*
void Estimator::vector2double() {
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++) {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    // for (int i = 0; i < f_manager.getFeatureCount(); i++)
    for (int i = 0; i < dep.size(); i++)
        para_Feature[i][0] = dep(i); // inverse depth here
    if (ESTIMATE_TD)
        para_Td[0][0] = td;
}

void Estimator::double2vector() {
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur) {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();

    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0) {
        LOG(WARNING) << "euler singular point!";
        rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                       para_Pose[0][3],
                                       para_Pose[0][4],
                                       para_Pose[0][5]).toRotationMatrix().transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++) {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4],
                                       para_Pose[i][5]).normalized().toRotationMatrix();

        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;

        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5]).toRotationMatrix();
    }

    VectorXd dep = f_manager.getDepthVector();
    // for (int i = 0; i < f_manager.getFeatureCount(); i++)
    for (int i = 0; i < dep.size(); i++) {
        dep(i) = para_Feature[i][0];
    }
    f_manager.setDepth(dep);
    f_manager.removeFailures();

    if (ESTIMATE_TD)
        td = para_Td[0][0];

    // relative info between two loop frame
    if (relocalization_info) {
        Matrix3d relo_r;
        Vector3d relo_t;
        relo_r = rot_diff * Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4],
                                        relo_Pose[5]).normalized().toRotationMatrix();
        relo_t = rot_diff * Vector3d(relo_Pose[0] - para_Pose[0][0],
                                     relo_Pose[1] - para_Pose[0][1],
                                     relo_Pose[2] - para_Pose[0][2]) + origin_P0;
        double drift_correct_yaw;
        drift_correct_yaw = Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
        drift_correct_r = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
        drift_correct_t = prev_relo_t - drift_correct_r * relo_t;
        relo_relative_t = relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
        relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
        relo_relative_yaw = Utility::normalizeAngle(
                Utility::R2ypr(Rs[relo_frame_local_index]).x() - Utility::R2ypr(relo_r).x());
        //cout << "vins relo " << endl;
        //cout << "vins relative_t " << relo_relative_t.transpose() << endl;
        //cout << "vins relative_yaw " <<relo_relative_yaw << endl;
        relocalization_info = 0;

    }
}*/
/*

bool Estimator::failureDetection() {
    if (f_manager.last_track_num < 2) {
        LOG(ERROR) << "too few tracked features " << f_manager.last_track_num;
        // return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 1.0) {
        LOG(ERROR) << "big acc bias estimation " << Bas[WINDOW_SIZE].norm();
        // return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0) {
        LOG(ERROR) << "big gyr bias estimation " << Bgs[WINDOW_SIZE].norm();
        // return true;
    }
    */
/*
    if (tic(0) > 1)
    {
        // ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    *//*

    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5) {
        LOG(ERROR) << "big translation";
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1) {
        LOG(ERROR) << "big z translation";
        return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50) {
        // ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}
*/
/*

#define MARGINALIZATION 1
void Estimator::optimization() {
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
        /// fix biases
*/
/*        std::vector<int> const_indices{6,7,8};
        ceres::SubsetParameterization *subset_parameterization = new ceres::SubsetParameterization(9, const_indices);
        problem.SetParameterization(para_SpeedBias[i], subset_parameterization);*//*

    }
    for (int i = 0; i < NUM_OF_CAM; i++) {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC) {
            // LOG(WARNING) << "fix extinsic param";
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        } else {
            // LOG(WARNING) << "estimate extinsic param";
        }
    }
    if (ESTIMATE_TD) {
        problem.AddParameterBlock(para_Td[0], 1);
        //problem.SetParameterBlockConstant(para_Td[0]);
    }

    vins_estimator::TicToc t_whole, t_prepare;
    vector2double();
#if MARGINALIZATION
    if (last_marginalization_info) {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(
                last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
#endif
    for (int i = 0; i < WINDOW_SIZE; i++) {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 2.0) /// 10.0
            continue;
        IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j],
                                 para_SpeedBias[j]);
    }
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2*/
/*used_num >= 2 debug*//*
 && it_per_id.start_frame < WINDOW_SIZE*/
/*WINDOW_SIZE -2 debug*//*
))
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;
            if (imu_i == imu_j) {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;
            if (ESTIMATE_TD) {
                ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j,
                                                                  it_per_id.feature_per_frame[0].velocity,
                                                                  it_per_frame.velocity,
                                                                  it_per_id.feature_per_frame[0].cur_td,
                                                                  it_per_frame.cur_td,
                                                                  it_per_id.feature_per_frame[0].uv.y(),
                                                                  it_per_frame.uv.y());
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j],
                                         para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
                */
/*
                double **para = new double *[5];
                para[0] = para_Pose[imu_i];
                para[1] = para_Pose[imu_j];
                para[2] = para_Ex_Pose[0];
                para[3] = para_Feature[feature_index];
                para[4] = para_Td[0];
                f_td->check(para);
                *//*

            } else {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j],
                                         para_Ex_Pose[0], para_Feature[feature_index]);
            }
            f_m_cnt++;
        }
    }

    LOG(INFO) << "visual measurement count: " << f_m_cnt;
    // ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    if (relocalization_info) {
        //printf("set relocalization factor! \n");
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(relo_Pose, SIZE_POSE, local_parameterization);
        int retrive_feature_index = 0;
        int feature_index = -1;
        for (auto &it_per_id : f_manager.feature) {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2*/
/*used_num >= 2 debug*//*
 && it_per_id.start_frame < WINDOW_SIZE*/
/*WINDOW_SIZE -2 debug*//*
))
                continue;
            ++feature_index;
            int start = it_per_id.start_frame;
            if (start <= relo_frame_local_index) {
                while ((int) match_points[retrive_feature_index].z() < it_per_id.feature_id) {
                    retrive_feature_index++;
                }
                if ((int) match_points[retrive_feature_index].z() == it_per_id.feature_id) {
                    Vector3d pts_j = Vector3d(match_points[retrive_feature_index].x(),
                                              match_points[retrive_feature_index].y(), 1.0);
                    Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                    problem.AddResidualBlock(f, loss_function, para_Pose[start], relo_Pose,
                                             para_Ex_Pose[0], para_Feature[feature_index]);
                    retrive_feature_index++;
                }
            }
        }

    }

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    vins_estimator::TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    // ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    // ROS_DEBUG("solver costs: %f", t_solver.toc());

    double2vector();
#if MARGINALIZATION
    vins_estimator::TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD) {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info) {
            vector<int> drop_set;
            for (int i = 0;
                 i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(
                    last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor,
                                                                           NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        {
            if (pre_integrations[1]->sum_dt < 10.0) {
                IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                               vector<double *>{
                                                                                       para_Pose[0],
                                                                                       para_SpeedBias[0],
                                                                                       para_Pose[1],
                                                                                       para_SpeedBias[1]},
                                                                               vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature) {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2*/
/*used_num >= 2 debug*//*
 && it_per_id.start_frame < WINDOW_SIZE*/
/*WINDOW_SIZE -2 debug*//*
))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame) {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    if (ESTIMATE_TD) {
                        ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j,
                                                                          it_per_id.feature_per_frame[0].velocity,
                                                                          it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td,
                                                                          it_per_frame.cur_td,
                                                                          it_per_id.feature_per_frame[0].uv.y(),
                                                                          it_per_frame.uv.y());
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td,
                                                                                       loss_function,
                                                                                       vector<double *>{
                                                                                               para_Pose[imu_i],
                                                                                               para_Pose[imu_j],
                                                                                               para_Ex_Pose[0],
                                                                                               para_Feature[feature_index],
                                                                                               para_Td[0]},
                                                                                       vector<int>{
                                                                                               0,
                                                                                               3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    } else {
                        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f,
                                                                                       loss_function,
                                                                                       vector<double *>{
                                                                                               para_Pose[imu_i],
                                                                                               para_Pose[imu_j],
                                                                                               para_Ex_Pose[0],
                                                                                               para_Feature[feature_index]},
                                                                                       vector<int>{
                                                                                               0,
                                                                                               3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }

        vins_estimator::TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        // ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

        vins_estimator::TicToc t_margin;
        marginalization_info->marginalize();
        // ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++) {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        if (ESTIMATE_TD) {
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        }
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;

    } else {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks),
                       std::end(last_marginalization_parameter_blocks),
                       para_Pose[WINDOW_SIZE - 1])) {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info) {
                vector<int> drop_set;
                for (int i = 0;
                     i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
                    // ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(
                        last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                        marginalization_factor, NULL,
                        last_marginalization_parameter_blocks,
                        drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            vins_estimator::TicToc t_pre_margin;
            // ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            // ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            vins_estimator::TicToc t_margin;
            // ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            // ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++) {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE) {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                } else {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            if (ESTIMATE_TD) {
                addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            }

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(
                    addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;

        }
    }
    LOG(INFO) << "whole marginalization costs: " << t_whole_marginalization.toc() << " ms";
#endif
    // ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}
*/

void Estimator::slideWindow() {
    vins_estimator::TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD) {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE) {
            for (int i = 0; i < WINDOW_SIZE; i++) {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
            }
            /// Here is better to use imu prediction.
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

/*            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE],
                                                                Bgs[WINDOW_SIZE]};*/
            pre_integrations[WINDOW_SIZE] = make_shared<IntegrationBase>(acc_0, gyr_0, Bas[WINDOW_SIZE],
                                                                    Bgs[WINDOW_SIZE]);

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (true || solver_flag == INITIAL) {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                // delete it_0->second.pre_integration;
                // it_0->second.pre_integration = nullptr;
                it_0->second.pre_integration.reset();

                for (map<double, ImageFrame>::iterator it = all_image_frame.begin();
                     it != it_0; ++it) {
/*                    if (it->second.pre_integration)
                        delete it->second.pre_integration;
                    it->second.pre_integration = NULL;*/
                    it->second.pre_integration.reset();
                }

                all_image_frame.erase(all_image_frame.begin(), it_0);
                all_image_frame.erase(t_0);

            }
            slideWindowOld();
        }
    } else {
        if (frame_count == WINDOW_SIZE) {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++) {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration,
                                                             tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

/*            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE],
                                                                Bgs[WINDOW_SIZE]};*/
            pre_integrations[WINDOW_SIZE] = make_shared<IntegrationBase>(acc_0, gyr_0, Bas[WINDOW_SIZE],
                                                                         Bgs[WINDOW_SIZE]);

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew() {
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld() {
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth) {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    } else
        f_manager.removeBack();
}
/*
void Estimator::setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points,
                             Vector3d _relo_t, Matrix3d _relo_r) {
    relo_frame_stamp = _frame_stamp;
    relo_frame_index = _frame_index;
    match_points.clear();
    match_points = _match_points;
    prev_relo_t = _relo_t;
    prev_relo_r = _relo_r;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        if (relo_frame_stamp == Headers[i]) {
            relo_frame_local_index = i;
            relocalization_info = 1;
            for (int j = 0; j < SIZE_POSE; j++)
                relo_Pose[j] = para_Pose[i][j];
        }
    }
}*/

