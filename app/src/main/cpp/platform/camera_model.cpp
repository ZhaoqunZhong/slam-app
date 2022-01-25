#include <Eigen/Dense>
#include "opencv2/core/eigen.hpp" //#include <Eigen/Dense> must be put before this, otherwise cv2eigen causes error
#include "opencv2/opencv.hpp"
#include "camera_model.h"
#include "native_debug.h"
#include "perf_monitor.h"
#include "glog/logging.h"

/*bool CameraModel::readFromYamlFile(std::string filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    ASSERT(fs.isOpened(), "camera model read config file failed!")

    rgb_w_ = static_cast<int>(fs["rgb_req_width"]);
    rgb_h_ = static_cast<int>(fs["rgb_req_height"]);
    fs["rgb_distortion_parameters"] >> rgb_distort_paras_;
    cv::FileNode n = fs["rgb_projection_parameters"];
    rgb_k_mat_ << static_cast<double>(n["fx"]), 0, static_cast<double>(n["cx"]), 0, static_cast<double>(n["fy"]), static_cast<double>(n["cy"]), 0, 0, 1;
    // LOG(INFO) << "rgb_k_mat_ " << rgb_k_mat_;
    inv_rgb_k_mat_ = rgb_k_mat_.inverse();
    cv::eigen2cv(rgb_k_mat_, rgb_intrinsic_cv_);
    distort_type_ = (static_cast<int>(fs["rgb_model_type"]) == 0) ? RAD_TAN : FISH_EYE;

    fs.release();

    return true;
}*/

void CameraModel::spaceToPlane(const Eigen::Vector3d &P, Eigen::Vector2d &p) {
    Eigen::Vector2d p_u, p_d;
    // Project points to the normalised plane
    p_u << P(0) / P(2), P(1) / P(2);

    if (distort_type_ == RAD_TAN) {
        if (no_distortion_) {
            p_d = p_u;
        } else {
            // Apply distortion
            Eigen::Vector2d d_u;
            distortion(p_u, d_u);
            p_d = p_u + d_u;
        }

        // Apply generalised projection matrix
        Eigen::Vector3d n_p_d{p_d.x(), p_d.y(), 1};
        Eigen::Vector3d uv1 = rgb_k_mat_ * n_p_d;
        p = uv1.topRows(2);

    } else if (distort_type_ == FISH_EYE) {
/*        double theta = acos(P(2) / P.norm());
        double phi = atan2(P(1), P(0));
        Eigen::Vector2d p_u = distortFisheyeViewAngle(rgb_distort_paras_[0], rgb_distort_paras_[1], rgb_distort_paras_[2],
                                rgb_distort_paras_[3], theta) * Eigen::Vector2d(cos(phi), sin(phi));
        Eigen::Vector3d n_p_d{p_u(0), p_u(1), 1};
        Eigen::Vector3d uv1 = rgb_k_mat_ * n_p_d;
        p = uv1.topRows(2);*/
        double n_p_norm = p_u.norm();
        double theta = atan(n_p_norm);
        double theta_d = distortFisheyeViewAngle(rgb_distort_paras_[0], rgb_distort_paras_[1], rgb_distort_paras_[2],
                                      rgb_distort_paras_[3], theta);
        p_d = tan(theta_d) / n_p_norm * p_u;
        Eigen::Vector3d n_p_d{p_d(0), p_d(1), 1};
        Eigen::Vector3d uv1 = rgb_k_mat_ * n_p_d;
        p = uv1.topRows(2);
    }
}

void CameraModel::distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u) const {
    if (rgb_distort_paras_.size() == 4) {
        double k1 = rgb_distort_paras_[0];
        double k2 = rgb_distort_paras_[1];
        double p1 = rgb_distort_paras_[2];
        double p2 = rgb_distort_paras_[3];

        double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

        mx2_u = p_u(0) * p_u(0);
        my2_u = p_u(1) * p_u(1);
        mxy_u = p_u(0) * p_u(1);
        rho2_u = mx2_u + my2_u;
        rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
        d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
                p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
    } else if (rgb_distort_paras_.size() == 5) {
        double k1 = rgb_distort_paras_[0];
        double k2 = rgb_distort_paras_[1];
        double p1 = rgb_distort_paras_[2];
        double p2 = rgb_distort_paras_[3];
        double k3 = rgb_distort_paras_[4];

        double mx2_u, my2_u, mxy_u, rho2_u, rho4_u, rad_dist_u;

        mx2_u = p_u(0) * p_u(0);
        my2_u = p_u(1) * p_u(1);
        mxy_u = p_u(0) * p_u(1);
        rho2_u = mx2_u + my2_u;
        rho4_u = rho2_u * rho2_u;
        rad_dist_u = k1 * rho2_u + k2 * rho4_u + k3 * rho4_u * rho2_u;
        d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
                p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
    }
}

/**
 * @brief Opencv fisheye model {k1, k2, k3, k4} are equal to {k2, k3, k4, k5} here.
 *        Check https://github.com/hengli/camodocal#readme
 * @tparam T
 * @param k2
 * @param k3
 * @param k4
 * @param k5
 * @param theta
 * @return
 */
template<typename T>
T CameraModel::distortFisheyeViewAngle(T k2, T k3, T k4, T k5, T theta) {
    // k1 = 1
    return theta +
           k2 * theta * theta * theta +
           k3 * theta * theta * theta * theta * theta +
           k4 * theta * theta * theta * theta * theta * theta * theta +
           k5 * theta * theta * theta * theta * theta * theta * theta * theta * theta;
}

bool CameraModel::inRange(const cv::Point2f &pt) {
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < rgb_w_ - BORDER_SIZE && BORDER_SIZE <= img_y &&
           img_y < rgb_h_ - BORDER_SIZE;
}

/**
 * \brief Lifts a point from the image plane to its projective ray
 *
 * \param p image coordinates
 * \param P coordinates of the projective ray
 */
void CameraModel::liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P) {
    if (distort_type_ == RAD_TAN) {
        double mx_d, my_d, mx2_d, mxy_d, my2_d, mx_u, my_u;
        double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;

        // Lift points to normalised plane
        Eigen::Vector3d n_p = inv_rgb_k_mat_ * Eigen::Vector3d(p.x(), p.y(), 1);
        mx_d = n_p.x();
        my_d = n_p.y();

        if (no_distortion_) {
            mx_u = mx_d;
            my_u = my_d;
        } else {
            // Recursive distortion model
            int n = 8;
            Eigen::Vector2d d_u;
            distortion(Eigen::Vector2d(mx_d, my_d), d_u);
            // Approximate value
            mx_u = mx_d - d_u(0);
            my_u = my_d - d_u(1);

            for (int i = 1; i < n; ++i) {
                distortion(Eigen::Vector2d(mx_u, my_u), d_u);
                mx_u = mx_d - d_u(0);
                my_u = my_d - d_u(1);
            }

        }
        // Obtain a projective ray
        P << mx_u, my_u, 1.0;

    } else if (distort_type_ == FISH_EYE) {
        // Lift points to normalised plane
        Eigen::Vector2d p_u = (inv_rgb_k_mat_ * Eigen::Vector3d(p.x(), p.y(), 1)).topRows(2);

        // Obtain a projective ray
        double theta, phi;
        backprojectSymmetric(p_u, theta, phi);

        P(0) = sin(theta) * cos(phi);
        P(1) = sin(theta) * sin(phi);
        P(2) = cos(theta);
        P << P(0) / P(2), P(1) / P(2), 1;
    }
}

void
CameraModel::backprojectSymmetric(const Eigen::Vector2d &p_u, double &theta, double &phi) const {
    double tol = 1e-10;
    double p_u_norm = p_u.norm();

    if (p_u_norm < 1e-10) {
        phi = 0.0;
    } else {
        phi = atan2(p_u(1), p_u(0));
    }

    int npow = 9;
    if (rgb_distort_paras_[3] == 0.0) {
        npow -= 2;
    }
    if (rgb_distort_paras_[2] == 0.0) {
        npow -= 2;
    }
    if (rgb_distort_paras_[1] == 0.0) {
        npow -= 2;
    }
    if (rgb_distort_paras_[0] == 0.0) {
        npow -= 2;
    }

    Eigen::MatrixXd coeffs(npow + 1, 1);
    coeffs.setZero();
    coeffs(0) = -p_u_norm;
    coeffs(1) = 1.0;

    if (npow >= 3) {
        coeffs(3) = rgb_distort_paras_[0];
    }
    if (npow >= 5) {
        coeffs(5) = rgb_distort_paras_[1];
    }
    if (npow >= 7) {
        coeffs(7) = rgb_distort_paras_[2];
    }
    if (npow >= 9) {
        coeffs(9) = rgb_distort_paras_[3];
    }

    if (npow == 1) {
        theta = p_u_norm;
    } else {
        // Get eigenvalues of companion matrix corresponding to polynomial.
        // Eigenvalues correspond to roots of polynomial.
        Eigen::MatrixXd A(npow, npow);
        A.setZero();
        A.block(1, 0, npow - 1, npow - 1).setIdentity();
        A.col(npow - 1) = -coeffs.block(0, 0, npow, 1) / coeffs(npow);

        Eigen::EigenSolver<Eigen::MatrixXd> es(A);
        Eigen::MatrixXcd eigval = es.eigenvalues();

        std::vector<double> thetas;
        for (int i = 0; i < eigval.rows(); ++i) {
            if (fabs(eigval(i).imag()) > tol) {
                continue;
            }

            double t = eigval(i).real();

            if (t < -tol) {
                continue;
            } else if (t < 0.0) {
                t = 0.0;
            }

            thetas.push_back(t);
        }

        if (thetas.empty()) {
            theta = p_u_norm;
        } else {
            theta = *std::min_element(thetas.begin(), thetas.end());
        }
    }
}

