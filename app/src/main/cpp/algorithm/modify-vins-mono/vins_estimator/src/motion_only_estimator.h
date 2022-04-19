//
// Created by zhong on 2021/11/24.
//

#ifndef VIO_MOTION_ONLY_ESTIMATOR_H
#define VIO_MOTION_ONLY_ESTIMATOR_H

#include <mutex>
#include <queue>
#include "Eigen/Eigen"
#include "feature_manager.h"
#include <ceres/ceres.h>
#include "factor/projection_factor.h"

class MotionOnlyIMUFactor : public ceres::SizedCostFunction<9, 7, 3, 7, 3> {
public:
    MotionOnlyIMUFactor() = delete;

    MotionOnlyIMUFactor(std::shared_ptr<IntegrationBase> pre_integration, Eigen::Vector3d &ba, Eigen::Vector3d &bg)
            : pre_integration_(pre_integration), ba_(ba), bg_(bg) {
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);

        Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
        Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);

        Eigen::Map<Eigen::Matrix<double, 9, 1>> residual(residuals);
        Eigen::Matrix<double, 15, 1> origin_res = pre_integration_->evaluate(Pi, Qi, Vi, ba_, bg_, Pj, Qj, Vj, ba_,
                                                                             bg_);
        residual = origin_res.topRows(9);

        Eigen::Matrix<double, 15, 15> origin_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(
                pre_integration_->covariance.inverse()).matrixL().transpose();
        Eigen::Matrix<double, 9, 9> sqrt_info = origin_info.topLeftCorner<9, 9>();
        //sqrt_info.setIdentity();
        residual = sqrt_info * residual;

        if (jacobians) {
            double sum_dt = pre_integration_->sum_dt;
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 9, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();

                jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
                jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(
                        Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));
#if 1
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
                Eigen::Quaterniond corrected_delta_q =
                        pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) *
                                                          Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
#endif
                jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

                jacobian_pose_i = sqrt_info * jacobian_pose_i;

                LOG_ASSERT(fabs(jacobian_pose_i.maxCoeff()) < 1e8);
                LOG_ASSERT(fabs(jacobian_pose_i.minCoeff()) < 1e8);
            }
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_speed_i(jacobians[1]);
                jacobian_speed_i.setZero();
                jacobian_speed_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
                jacobian_speed_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
                jacobian_speed_i = sqrt_info * jacobian_speed_i;
                LOG_ASSERT(fabs(jacobian_speed_i.maxCoeff()) < 1e8);
                LOG_ASSERT(fabs(jacobian_speed_i.minCoeff()) < 1e8);
            }
            if (jacobians[2]) {
                Eigen::Map<Eigen::Matrix<double, 9, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
                jacobian_pose_j.setZero();
                jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

#if 1
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
                Eigen::Quaterniond corrected_delta_q =
                        pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(
                        corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
#endif
                jacobian_pose_j = sqrt_info * jacobian_pose_j;
                LOG_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
                LOG_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);
            }
            if (jacobians[3]) {
                Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_speed_j(jacobians[3]);
                jacobian_speed_j.setZero();

                jacobian_speed_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();
                jacobian_speed_j = sqrt_info * jacobian_speed_j;
                LOG_ASSERT(fabs(jacobian_speed_j.maxCoeff()) < 1e8);
                LOG_ASSERT(fabs(jacobian_speed_j.minCoeff()) < 1e8);
            }
        }

        return true;
    }

    std::shared_ptr<IntegrationBase> pre_integration_;
    Eigen::Vector3d ba_, bg_;
};

class MotionOnlyProjectionFactor : public ceres::SizedCostFunction<2, 7> {
public:
    MotionOnlyProjectionFactor(Vector3d & ptw, Vector3d & p_norm) : pt_(ptw), p_norm_(p_norm){}
    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        Vector3d pt_b = Qi.inverse() * (pt_ - Pi);
        Vector3d pt_c = RIC[0].inverse() * (pt_b - TIC[0]);

        Eigen::Map<Eigen::Vector2d> residual(residuals);
        double dep = pt_c.z();
        residual = (pt_c / dep - p_norm_).head<2>();
        residual = ProjectionFactor::sqrt_info * residual;

        if (jacobians) {
            Eigen::Matrix3d Ri = Qi.toRotationMatrix();
            Eigen::Matrix<double, 2, 3> jaco_r_ptc(2, 3);
            jaco_r_ptc << 1. / dep, 0, -pt_c.x() / (dep * dep),
                    0, 1. / dep, -pt_c.y() / (dep * dep);
            jaco_r_ptc = ProjectionFactor::sqrt_info * jaco_r_ptc;
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);

                Eigen::Matrix<double, 3, 6> jaco_ptc_pose;
                jaco_ptc_pose.leftCols<3>() = - RIC[0].transpose() * Ri.transpose();
                jaco_ptc_pose.rightCols<3>() = RIC[0].transpose() * Utility::skewSymmetric(Ri.transpose() * (pt_ - Pi));

                jacobian_pose.leftCols<6>() = jaco_r_ptc * jaco_ptc_pose;
                jacobian_pose.rightCols<1>().setZero();
            }
        }
        return true;
    }
    Vector3d pt_, p_norm_;
};

class MotionOnlyEstimator {
public:
    void updateBackend(Eigen::Matrix3d Rs[], Eigen::Vector3d Ps[], Eigen::Vector3d &Vbk, Eigen::Vector3d &ba,
                       Eigen::Vector3d &bg, double &header_bk, double &td_in, list<FeaturePerId> &feature);

    void updateFrontend(Eigen::Matrix3d Rs[], Eigen::Vector3d Ps[], Eigen::Vector3d &Vbk, Eigen::Vector3d &ba,
                        Eigen::Vector3d &bg, double &header_bk, double &td_out,
                        unordered_map<int, Vector3d> &feature);

    std::mutex backend_mtx_;
    Eigen::Matrix3d Rwbs[WINDOW_SIZE + 1];
    Eigen::Vector3d Pwbs[WINDOW_SIZE + 1];
    Eigen::Vector3d Vb_end, Ba_end, Bg_end;
    double ts_end = -1, td;
    unordered_map<int, Vector3d> feature_end;
};

#endif //VIO_MOTION_ONLY_ESTIMATOR_H
