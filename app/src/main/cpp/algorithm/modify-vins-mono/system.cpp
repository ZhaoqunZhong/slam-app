#include "system.h"
//debug
// #include "native_debug.h"

using namespace std;
using namespace cv;

System::System(uint image_height, uint image_width, double fx, double fy, double alpha_x, double alpha_y,
               cam_distortion_type type, double d1, double d2, double d3, double d4,//distortion paras
               double readout, double acc_n, double acc_w, double gyr_n, double gyr_w,
               Eigen::Matrix3d Ric, Eigen::Vector3d tic, double timeshift,
               double gravity_norm, initial_result_callback cbk) : bStart_backend(true), process_exited(false) {
    // estimator.config_path_ = sConfig_file_;
    // vins_estimator::readParameters(sConfig_file_);
    // feature_tracker::readParameters(sConfig_file_);
    // trackerData[0].readIntrinsicParameter(sConfig_file_);
    // estimator.setParameter();
    // trackerData[0].m_camera = CameraFactory::instance()->generateCamera(Camera::PINHOLE, "",
    //                                                                     cv::Size(640, 480));

    vins_estimator::SOLVER_TIME = 0.04;
    vins_estimator::NUM_ITERATIONS = 8;
    vins_estimator::MIN_PARALLAX = 10.0;
    vins_estimator::MIN_PARALLAX = vins_estimator::MIN_PARALLAX / vins_estimator::FOCAL_LENGTH;
    vins_estimator::ACC_N = acc_n;
    vins_estimator::ACC_W = acc_w;
    vins_estimator::GYR_N = gyr_n;
    vins_estimator::GYR_W = gyr_w;
    vins_estimator::G.z() = gravity_norm;
    vins_estimator::ROW = image_height;
    vins_estimator::COL = image_width;
    vins_estimator::ESTIMATE_EXTRINSIC = 0;
    vins_estimator::RIC.push_back(Ric);
    vins_estimator::TIC.push_back(tic);
    vins_estimator::INIT_DEPTH = 5.0;
    vins_estimator::TD = timeshift;
    vins_estimator::ESTIMATE_TD = 1;
    vins_estimator::ROLLING_SHUTTER = 1;
    vins_estimator::TR = readout;
    for (int i = 0; i < vins_estimator::NUM_OF_CAM; i++) {
        estimator.tic[i] = TIC[i];
        estimator.ric[i] = RIC[i];
    }
    // f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = vins_estimator::FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = vins_estimator::FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    estimator.td = vins_estimator::TD;

    feature_tracker::MAX_CNT = 150;
    feature_tracker::MIN_DIST = 30;
    feature_tracker::ROW = image_height;
    feature_tracker::COL = image_width;
    feature_tracker::FREQ = 10;
    feature_tracker::F_THRESHOLD = 1.0;
    feature_tracker::EQUALIZE = 0;
    feature_tracker::FISHEYE = 0;
    feature_tracker::WINDOW_SIZE = vins_estimator::WINDOW_SIZE;
    feature_tracker::FOCAL_LENGTH = 460;
    feature_tracker::PUB_THIS_FRAME = false;
    trackerData[0].image_readout_s = readout;


    if (type == RAD_TAN) {
        trackerData[0].m_camera = CameraFactory::instance()->generateCamera(Camera::PINHOLE, "",
                                                                            cv::Size(feature_tracker::COL, feature_tracker::ROW));

    } else if (type == FISH_EYE) {
        trackerData[0].m_camera = CameraFactory::instance()->generateCamera(Camera::KANNALA_BRANDT, "",
                                                                            cv::Size(feature_tracker::COL, feature_tracker::ROW));
    }
    std::vector<double> cam_paras {d1, d2, d3, d4, fx, fy, alpha_x, alpha_y};
    trackerData[0].m_camera->readParameters(cam_paras);

    estimator.initial_callback_ = cbk;

    vi_th_ = std::thread(&System::process, this);
    vi_th_.detach();
    thread_initialized_ = true;
}

void System::addCalibrationParas(uint image_height, uint image_width, double fx, double fy, double alpha_x, double alpha_y,
                         cam_distortion_type type, double d1, double d2, double d3, double d4,//distortion paras
                         double readout, double acc_n, double acc_w, double gyr_n, double gyr_w,
                         Eigen::Matrix3d Ric, Eigen::Vector3d tic, double timeshift,
                         double gravity_norm) {

    vins_estimator::SOLVER_TIME = 0.04;
    vins_estimator::NUM_ITERATIONS = 8;
    vins_estimator::MIN_PARALLAX = 10.0;
    vins_estimator::MIN_PARALLAX = vins_estimator::MIN_PARALLAX / vins_estimator::FOCAL_LENGTH;
    vins_estimator::ACC_N = acc_n;
    vins_estimator::ACC_W = acc_w;
    vins_estimator::GYR_N = gyr_n;
    vins_estimator::GYR_W = gyr_w;
    vins_estimator::G.z() = gravity_norm;
    vins_estimator::ROW = image_height;
    vins_estimator::COL = image_width;
    vins_estimator::ESTIMATE_EXTRINSIC = 0;
    vins_estimator::RIC.push_back(Ric);
    vins_estimator::TIC.push_back(tic);
    vins_estimator::INIT_DEPTH = 5.0;
    vins_estimator::TD = timeshift;
    vins_estimator::ESTIMATE_TD = 1;
    vins_estimator::ROLLING_SHUTTER = 1;
    vins_estimator::TR = readout;
    for (int i = 0; i < vins_estimator::NUM_OF_CAM; i++) {
        estimator.tic[i] = TIC[i];
        estimator.ric[i] = RIC[i];
    }
    // f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = vins_estimator::FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = vins_estimator::FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    estimator.td = vins_estimator::TD;

    feature_tracker::MAX_CNT = 150;
    feature_tracker::MIN_DIST = 30;
    feature_tracker::ROW = image_height;
    feature_tracker::COL = image_width;
    feature_tracker::FREQ = 10;
    feature_tracker::F_THRESHOLD = 1.0;
    feature_tracker::EQUALIZE = 0;
    feature_tracker::FISHEYE = 0;
    feature_tracker::WINDOW_SIZE = vins_estimator::WINDOW_SIZE;
    feature_tracker::FOCAL_LENGTH = 460;
    feature_tracker::PUB_THIS_FRAME = false;
    trackerData[0].image_readout_s = readout;


    if (type == RAD_TAN) {
        trackerData[0].m_camera = CameraFactory::instance()->generateCamera(Camera::PINHOLE, "",
                                  cv::Size(feature_tracker::COL, feature_tracker::ROW));

    } else if (type == FISH_EYE) {
        trackerData[0].m_camera = CameraFactory::instance()->generateCamera(Camera::KANNALA_BRANDT, "",
                                   cv::Size(feature_tracker::COL, feature_tracker::ROW));
    }
    std::vector<double> cam_paras {d1, d2, d3, d4, fx, fy, alpha_x, alpha_y};
    trackerData[0].m_camera->readParameters(cam_paras);
    // LOG(WARNING) << "DEBUG camera info " << trackerData[0].m_camera->parametersToString();

}

System::~System() {
    m_buf.lock();
    while (!feature_buf.empty())
        feature_buf.pop();
    while (!imu_buf.empty())
        imu_buf.pop();
    m_buf.unlock();

    m_estimator.lock();
    estimator.clearState();
    m_estimator.unlock();
}

void System::subImageData(double dStampSec, Mat img) {
    if (estimator.initial_finished_ || !thread_initialized_)
        return;

    if (first_image_flag) {
        LOG(INFO) << "subImageData first_image_flag" << endl;
        first_image_flag = false;
        last_publish_time = -1;
        last_image_time = dStampSec;
        trackerData[0].readImage(img, dStampSec);
        return;
    }
    // detect unstable camera stream
    if (dStampSec - last_image_time > 1.0 || dStampSec < last_image_time) {
        LOG(ERROR) << "subImageData image discontinue! reset the feature tracker!" << endl;
        first_image_flag = true;
        last_image_time = 0;
        return;
    }
    last_image_time = dStampSec;

    // frequency control
    if (dStampSec - last_publish_time >= 0.999 / FREQ) {
        PUB_THIS_FRAME = true;
        last_publish_time = dStampSec;
    } else
        PUB_THIS_FRAME = false;

    // LOG(WARNING) << "DEBUG camera info " << trackerData[0].m_camera->parametersToString();
    trackerData[0].readImage(img, dStampSec);

    shared_ptr<IMG_MSG> feature_points(new IMG_MSG());
    feature_points->header = dStampSec;
    for (int i = 0; i < feature_tracker::NUM_OF_CAM; i++) {
        auto &un_pts = trackerData[i].cur_un_pts;
        auto &cur_pts = trackerData[i].cur_pts;
        auto &ids = trackerData[i].ids;
        auto &pts_velocity = trackerData[i].pts_velocity;
        for (unsigned int j = 0; j < ids.size(); j++) {
            if (trackerData[i].track_cnt[j] > 1) {
                int p_id = ids[j];
                double x = un_pts[j].x;
                double y = un_pts[j].y;
                double z = 1;
                feature_points->points.push_back(Vector3d(x, y, z));
                feature_points->id_of_point.push_back(p_id * feature_tracker::NUM_OF_CAM + i);
                feature_points->u_of_point.push_back(cur_pts[j].x);
                feature_points->v_of_point.push_back(cur_pts[j].y);
                feature_points->velocity_x_of_point.push_back(pts_velocity[j].x);
                feature_points->velocity_y_of_point.push_back(pts_velocity[j].y);
            }
        }
    }
    if (estimator.solver_flag == Estimator::NON_LINEAR) {
        mo_buf_mtx_.lock();
        mo_img_buf_.push(feature_points);
        mo_buf_mtx_.unlock();
        mo_buf_con_.notify_one();
    }
    if (PUB_THIS_FRAME) {
        //Features guarantee to have velocity now
        m_buf.lock();
        feature_buf.push(feature_points);
        m_buf.unlock();
        con.notify_one();
    }

    /// Feature tracking visualization
/*    cv::Mat show_img;
    cv::cvtColor(img, show_img, COLOR_GRAY2RGB);
    if (SHOW_TRACK) {
        for (unsigned int j = 0; j < trackerData[0].cur_pts.size(); j++) {
            double len = min(1.0, 1.0 * trackerData[0].track_cnt[j] / vins_estimator::WINDOW_SIZE);
            cv::circle(show_img, trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        }
        updatePreviewMat(show_img, true);
    }*/
/*    cv::Mat show_img;
    cv::cvtColor(img, show_img, COLOR_GRAY2RGB);
    if (imageProcessCallback_) {
        for (unsigned int j = 0; j < trackerData[0].cur_pts.size(); j++) {
            double len = min(1.0, 1.0 * trackerData[0].track_cnt[j] / vins_estimator::WINDOW_SIZE);
            cv::circle(show_img, trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        }
        slam_processed_image image;
        image.ts = dStampSec * 1e9;
        image.image_data = show_img.data;
        image.image_width = show_img.cols;
        image.image_height = show_img.rows;
        image.processing_time = 0;
        imageProcessCallback_(&image);
    }*/
    auto f = [](uint64_t ts, cv::Mat mat, System *sys) {
        cv::Mat show_img;
        cv::cvtColor(mat, show_img, COLOR_GRAY2RGB);
        for (unsigned int j = 0; j < sys->trackerData[0].cur_pts.size(); j++) {
            double len = min(1.0, 1.0 * sys->trackerData[0].track_cnt[j] / vins_estimator::WINDOW_SIZE);
            cv::circle(show_img, sys->trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        }
        slam_processed_image image;
        image.ts = ts;
        image.image_data = show_img.data;
        image.image_width = show_img.cols;
        image.image_height = show_img.rows;
        image.processing_time = 0;
        pthread_mutex_lock(&sys->img_pro_ckb_mtx_);
        sys->imageProcessCallback_(&image);
        pthread_mutex_unlock(&sys->img_pro_ckb_mtx_);
    };
    uint64_t image_ts = dStampSec * 1e9;
    std::thread t (f, std::ref(image_ts), img.clone(), this);
    t.detach();
}

vector<pair<vector<ImuConstPtr>, ImgConstPtr>> System::getMeasurements() {
    vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;

    while (true) {
        if (imu_buf.empty() || feature_buf.empty()) {
            // LOG(WARNING) << "imu_buf.empty() || feature_buf.empty()" << endl;
            return measurements;
        }

        if (imu_buf.back()->header <= feature_buf.front()->header + estimator.td) {
            // LOG(WARNING) << "wait for imu, only should happen at the beginning, sum_of_wait: " << sum_of_wait << endl;
            sum_of_wait++;
            return measurements;
        }

        if (imu_buf.front()->header >= feature_buf.front()->header + estimator.td) {
            LOG(WARNING) << "throw img, only should happen at the beginning" << endl;
            feature_buf.pop();
            continue;
        }
        ImgConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        vector<ImuConstPtr> IMUs;
        while (imu_buf.front()->header < img_msg->header + estimator.td) {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }

        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty()) {
            LOG(ERROR) << "no imu between two image" << endl;
        }

        measurements.emplace_back(IMUs, img_msg);
        // LOG(INFO) << "remaining imu num " << imu_buf.size() << " ";
    }

    // return measurements;
}

void System::subImuData(double dStampSec, const Eigen::Vector3d &vGyr, const Eigen::Vector3d &vAcc) {
    if (estimator.initial_finished_ || !thread_initialized_)
        return;

    shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
    imu_msg->header = dStampSec;
    imu_msg->linear_acceleration = vAcc;
    imu_msg->angular_velocity = vGyr;

    if (dStampSec <= last_imu_t) {
        LOG(ERROR) << "imu message in disorder!" << endl;
        return;
    }
    last_imu_t = dStampSec;

    // TimeLagMeasurer timer;
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    // con.notify_one();
    // double t = timer.lagFromStartSecond()*1e3;
    // LOG_IF(INFO, t > 1.5) << "subImuData cost " << t << " ms";

    if (estimator.solver_flag == Estimator::NON_LINEAR) {
        mo_buf_mtx_.lock();
        mo_imu_buf_.push(imu_msg);
        mo_buf_mtx_.unlock();
    }
}

// thread: visual-inertial odometry
void System::process() {
    while (bStart_backend) {
        vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;

        unique_lock<mutex> lk(m_buf);
        con.wait(lk, [&] {
            if (!bStart_backend) {
                return true;
            }
            return !(measurements = getMeasurements()).empty();
        });
        lk.unlock();

        m_estimator.lock();
        LOG_IF(WARNING, measurements.size() > 1) << "measurement size " << measurements.size() << " ";
        for (auto &measurement : measurements) {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first) {
                double t = imu_msg->header;
                double img_t = img_msg->header + estimator.td;
                if (t <= img_t) {
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x();
                    dy = imu_msg->linear_acceleration.y();
                    dz = imu_msg->linear_acceleration.z();
                    rx = imu_msg->angular_velocity.x();
                    ry = imu_msg->angular_velocity.y();
                    rz = imu_msg->angular_velocity.z();
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    // printf("1 BackEnd imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                } else {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x();
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y();
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z();
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x();
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y();
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z();
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // cout << "processing vision data with stamp:" << img_msg->header
            //     << " img_msg->points.size: "<< img_msg->points.size() << endl;

            // TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++) {
                int v = img_msg->id_of_point[i] + 0.5;
                int feature_id = v / feature_tracker::NUM_OF_CAM;
                int camera_id = v % feature_tracker::NUM_OF_CAM;
                double x = img_msg->points[i].x();
                double y = img_msg->points[i].y();
                double z = img_msg->points[i].z();
                double p_u = img_msg->u_of_point[i];
                double p_v = img_msg->v_of_point[i];
                double velocity_x = img_msg->velocity_x_of_point[i];
                double velocity_y = img_msg->velocity_y_of_point[i];
                assert(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
            }
            vins_estimator::TicToc t_processImage;
            estimator.processImage(image, img_msg->header);

            /// save or visualize result so far
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR) {
                /// display current pose
                Vector3d p_wi;
                Quaterniond q_wi;
                q_wi = Quaterniond(estimator.Rs[vins_estimator::WINDOW_SIZE]);
                p_wi = estimator.Ps[vins_estimator::WINDOW_SIZE];
                // updatePoseForDrawing(p_wi, q_wi);

                /// display sliding window
/*                pthread_mutex_lock(&pose_mtx);
                key_frames.clear();
                for (int i = 0; i < vins_estimator::WINDOW_SIZE + 1; i++) {
                    q_wi = Quaterniond(estimator.Rs[i]);
                    p_wi = estimator.Ps[i];
                    Eigen::Matrix4d T_wi = Eigen::Matrix4d::Identity();
                    T_wi.block<3, 3>(0, 0) = q_wi.toRotationMatrix();
                    T_wi.block<3, 1>(0, 3) = p_wi;
                    key_frames.push_back(T_wi);
                }
                pthread_mutex_unlock(&pose_mtx);*/
                /// save biases
                LOG(INFO) << "current acc bias " << estimator.Bas[vins_estimator::WINDOW_SIZE].transpose()
                    << " norm: " << estimator.Bas[vins_estimator::WINDOW_SIZE].norm();
                LOG(INFO) << "current gyr bias " << estimator.Bgs[vins_estimator::WINDOW_SIZE].transpose()
                        << " norm: " << estimator.Bgs[vins_estimator::WINDOW_SIZE].norm();
                Eigen::VectorXd biases(6);
                biases.topRows(3) = estimator.Bas[vins_estimator::WINDOW_SIZE];
                biases.bottomRows(3) = estimator.Bgs[vins_estimator::WINDOW_SIZE];
                estimator.biases_save_.push_back(biases);
                /// save extrinsic
                if (ESTIMATE_EXTRINSIC != 0) {
                    Eigen::VectorXd extrinsic(7);
                    extrinsic.topRows(3) = estimator.tic[0];
                    extrinsic.bottomRows(4) = Eigen::Quaterniond(estimator.ric[0]).coeffs();
                    estimator.extrinsic_save_.push_back(extrinsic);
                    LOG(INFO) << "current extrinsic values: " << extrinsic.transpose();
                }
                /// save td
                if (ESTIMATE_TD) {
                    LOG(INFO) << "current td: " << estimator.td;
                    estimator.td_save_.push_back(estimator.td);
                }
            }
        }
        m_estimator.unlock();
    }

    /// save vectors to file
    std::ofstream fbias(vins_estimator::OUTPUT_PATH + "biases_save.csv", std::ios::out);
    LOG(INFO) << "Saving biases to " << vins_estimator::OUTPUT_PATH + "biases_save.csv";
    for (auto &b : estimator.biases_save_) {
        fbias << b.transpose() << std::endl;
    }
    fbias.close();

    if (ESTIMATE_EXTRINSIC != 0) {
        ofstream fextrinsic(vins_estimator::OUTPUT_PATH + "extrinsic_save.csv", std::ios::out);
        LOG(INFO) << "Saving extrinsic to " << vins_estimator::OUTPUT_PATH + "extrinsic_save.csv";
        for (auto &ex : estimator.extrinsic_save_) {
            fextrinsic << ex.transpose() << std::endl;
        }
        fextrinsic.close();
    }

    if (ESTIMATE_TD) {
        ofstream ftd(vins_estimator::OUTPUT_PATH + "td_save.csv", std::ios::out);
        LOG(INFO) << "Saving td to " << vins_estimator::OUTPUT_PATH + "td_save.csv";
        for (auto &td : estimator.td_save_) {
            ftd << td << std::endl;
        }
        ftd.close();
    }

    process_exited = true;
}

std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> System::getMoMeasurements() {
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;

    estimator.mo_estimator_.updateFrontend(Rwbs, Pwbs, Vb_end, Ba_end, Bg_end, ts_end, td, feature_end);

    if (ts_end < 0) {
        std::queue<ImgConstPtr> empty_img_buf;
        std::queue<ImuConstPtr> empty_imu_buf;
        swap(empty_img_buf, mo_img_buf_);
        swap(empty_imu_buf, mo_imu_buf_);
        return measurements;
    }
    while (!mo_imu_buf_.empty() && mo_imu_buf_.front()->header <= ts_end + td) {
        mo_imu_buf_.pop();
    }
    while (!mo_img_buf_.empty() && mo_img_buf_.front()->header <= ts_end) {
        mo_img_buf_.pop();
    }

    auto imu_buf_copy = mo_imu_buf_;
    while (true) {
        if (mo_imu_buf_.empty() || mo_img_buf_.empty()) {
            return measurements;
        }
        if (imu_buf_copy.back()->header <= mo_img_buf_.front()->header + td) {
            // LOG(WARNING) << "getMoMeasurements() wait for imu.";
            return measurements;
        }
        if (imu_buf_copy.front()->header >= mo_img_buf_.front()->header + td) {
            /*            LOG(WARNING) << "getMoMeasurements() throw image.";
                        LOG(INFO) << "ts_end " << to_string(ts_end) << " ";
                        LOG(INFO) << "imu_buf_copy.front()->header " << to_string(imu_buf_copy.front()->header) << " ";
                        LOG(INFO) << "mo_img_buf_.front()->header + td " << to_string(mo_img_buf_.front()->header + td) << " ";*/
            mo_img_buf_.pop();
            continue;
        }
        ImgConstPtr img_msg = mo_img_buf_.front();
        mo_img_buf_.pop();

        vector<ImuConstPtr> IMUs;
        while (imu_buf_copy.front()->header < img_msg->header + td) {
            IMUs.emplace_back(imu_buf_copy.front());
            imu_buf_copy.pop();
        }
        IMUs.emplace_back(imu_buf_copy.front());
        if (IMUs.empty()) {
            LOG(ERROR) << "no imu between two image" << endl;
        }

        measurements.emplace_back(IMUs, img_msg);
    }
}


void System::motionOnlyProcess() {
    while (mo_estimate_start) {
        std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;
        unique_lock<mutex> lk(mo_buf_mtx_);
        mo_buf_con_.wait(lk, [&] {
            if (!mo_estimate_start) {
                return true;
            }
            return !(measurements = getMoMeasurements()).empty();
        });
        if (!mo_estimate_start)
            break;

        lk.unlock();
        // LOG(INFO) << "measurements size " << measurements.size() << "------------- ";
        ceres::Problem problem;
        ceres::LossFunction *loss_function;
        // loss_function = new ceres::HuberLoss(1.0);
        loss_function = new ceres::CauchyLoss(1.0);
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        vector<Vector3d> Vs;
        vector<array<double, 7>> poses;
        vector<shared_ptr<IntegrationBase>> imu_integrations;
        vector<ImgConstPtr> frames;
        Matrix3d last_R = Rwbs[vins_estimator::WINDOW_SIZE];
        Vector3d last_P = Pwbs[vins_estimator::WINDOW_SIZE], last_V = Vb_end;
        array<double, 7> pose;
        pose[0] = Pwbs[vins_estimator::WINDOW_SIZE].x();
        pose[1] = Pwbs[vins_estimator::WINDOW_SIZE].y();
        pose[2] = Pwbs[vins_estimator::WINDOW_SIZE].z();
        Quaterniond q(Rwbs[vins_estimator::WINDOW_SIZE]);
        pose[3] = q.x();
        pose[4] = q.y();
        pose[5] = q.z();
        pose[6] = q.w();
        poses.push_back(pose);
        Vs.push_back(Vb_end);
        for (auto &measurement : measurements) {
            /*            for (auto & imu : measurement.first) {
                            LOG(INFO) << "imu ts " << to_string(imu->header) << " ";
                        }
                        LOG(INFO) << "image ts + td " << to_string(measurement.second->header + td) << " ";*/
            Matrix3d R = last_R;
            Vector3d P = last_P, V = last_V;
            auto &imu0 = measurement.first.front();
            shared_ptr<IntegrationBase> pre_integration = make_shared<IntegrationBase>(imu0->linear_acceleration,
                                                                                       imu0->angular_velocity, Ba_end,
                                                                                       Bg_end);
            acc_0 = imu0->linear_acceleration;
            gyr_0 = imu0->angular_velocity;
            double last_t, dt;

            double img_t = measurement.second->header + td;
            frames.push_back(measurement.second);

            for (auto &imu : measurement.first) {
                double t = imu->header;
                if (t <= img_t) {
                    if (init_imu) {
                        init_imu = false;
                        dt = t - (ts_end + td);
                    } else
                        dt = imu->header - last_t;
                    last_t = t;
                } else {
                    dt = img_t - last_t;
                    last_t = img_t;
                }
                pre_integration->push_back(dt, imu->linear_acceleration, imu->angular_velocity);

                Vector3d un_acc_0 = R * (acc_0 - Ba_end) - estimator.g;
                Vector3d un_gyr = 0.5 * (gyr_0 + imu->angular_velocity) - Bg_end;
                R *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
                Vector3d un_acc_1 = R * (imu->linear_acceleration - Ba_end) - estimator.g;
                Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
                P += dt * V + 0.5 * dt * dt * un_acc;
                V += dt * un_acc;

                acc_0 = imu->linear_acceleration;
                gyr_0 = imu->angular_velocity;
            }
            imu_integrations.push_back(pre_integration);

            Vs.push_back(V);
            array<double, 7> pose;
            pose[0] = P.x();
            pose[1] = P.y();
            pose[2] = P.z();
            Quaterniond q(R);
            pose[3] = q.x();
            pose[4] = q.y();
            pose[5] = q.z();
            pose[6] = q.w();
            poses.push_back(pose);

            last_P = P;
            last_R = R;
            last_V = V;
            init_imu = true;
        }

        for (auto &pose : poses) {
            problem.AddParameterBlock(pose.data(), 7, local_parameterization);
        }
        problem.SetParameterBlockConstant(poses[0].data());
        for (auto &v : Vs) {
            problem.AddParameterBlock(v.data(), 3);
        }
        problem.SetParameterBlockConstant(Vs[0].data());

        uint cnt = 0;
        for (int i = 0; i < imu_integrations.size(); i++) {
            MotionOnlyIMUFactor *imu_factor = new MotionOnlyIMUFactor(imu_integrations[i], Ba_end, Bg_end);
            problem.AddResidualBlock(imu_factor, NULL, poses[i].data(), Vs[i].data(), poses[i + 1].data(),
                                     Vs[i + 1].data());

            for (int j = 0; j < frames[i]->points.size(); j++) {
                int feature_id = frames[i]->id_of_point[j];
                unordered_map<int, Vector3d>::iterator feature = feature_end.find(feature_id);
                if (feature != feature_end.end()) {
                    Vector3d norm_coord = frames[i]->points[j];
                    MotionOnlyProjectionFactor *f = new MotionOnlyProjectionFactor(feature->second, norm_coord);
                    problem.AddResidualBlock(f, loss_function, poses[i + 1].data());
                    cnt++;
                }
            }
        }
        LOG(INFO) << "number of feature projections " << cnt << " ";

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::DOGLEG;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // LOG(INFO) << "Motion only ba result " << summary.BriefReport();

        array<double, 7> latest_pose = poses.back();
        Vector3d p_latest(latest_pose[0], latest_pose[1], latest_pose[2]);
        Quaterniond q_lastest(latest_pose[6], latest_pose[3], latest_pose[4], latest_pose[5]);
        // updatePoseForDrawing(p_latest, q_lastest);
        /// pose callback
        slam_pose pose_out;
        pose_out.ts = measurements.back().second->header + td;
        Eigen::Map<Eigen::Vector3f> tran(pose_out.tran);
        tran = p_latest.cast<float>();
        Eigen::Map<Eigen::Quaternionf> rot(pose_out.rot);
        rot = q_lastest.cast<float>(); //Eigen quaternion storage order: xyzw
        Eigen::Map<Eigen::Matrix<float,4,4>> T(pose_out.T);
        T.block<3,3>(0,0) = q_lastest.toRotationMatrix().cast<float>();
        T.block<3,1>(0,3) = p_latest.cast<float>();
        pose_out.type = ACCURATE;
        // blocking version
        poseCallback_(&pose_out);
        // non-blocking version
/*        auto f = [](slam_pose pose, System *sys) {
            sys->poseCallback_(&pose);
        };
        std::thread t(f, std::ref(pose_out), this);
        t.detach();*/
    }

    mo_estimate_exited = true;
}

void System::registerInitialCallback(initial_result_callback cbk) {
    estimator.initial_callback_ = cbk;
}

