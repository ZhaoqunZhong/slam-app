#pragma once

#include <Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
// #include <ros/ros.h>
#include <map>
#include "../feature_manager.h"
#include "solve_5pts.h"

using namespace Eigen;
using namespace std;

class ImageFrame {
public:
    ImageFrame() {};

    ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &_points, double _t)
            : t{_t}, is_key_frame{false} {
        points = _points;
    };
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>> > points;
    double t;
    Matrix3d R;
    Vector3d T;
    shared_ptr<IntegrationBase> pre_integration;
    bool is_key_frame;
};

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d *Bgs, Vector3d &g,
                        VectorXd &x);

void RefineGravityAndBias(map<double, ImageFrame> &all_image_frame, Vector3d &g, Vector3d &ba,
                          Vector3d &bg);


class InertialOnlyConstraint
        : public ceres::SizedCostFunction<9, 9, 3, 3, 1, 3, 3> {// Rg, vb_k, vb_k+1, s, ba, bg
public:
    InertialOnlyConstraint(std::shared_ptr<IntegrationBase> imu_pre_integration,
                           const Eigen::Matrix3d R_c0_bk, const Eigen::Matrix3d R_c0_bkplus1,
                           const Eigen::Vector3d t_c0_ck, const Eigen::Vector3d t_c0_ckplus1,
                           const Eigen::Vector3d g0, const Eigen::Vector3d Tic) :
            pre_integration_(imu_pre_integration), R_c0_bk_(R_c0_bk),
            R_c0_bkplus1_(R_c0_bkplus1), t_c0_ck_(t_c0_ck), t_c0_ckplus1_(t_c0_ckplus1), g0_(g0), Tic_(Tic) {}

    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        Eigen::Map<Eigen::Matrix<double, 9, 1>> residual(residuals);
        Eigen::Map<const Eigen::Matrix<double, 3, 3>> Rg(parameters[0]);
        Eigen::Vector3d vb_k(parameters[1]);
        Eigen::Vector3d vb_kplus1(parameters[2]);
        double s = parameters[3][0];
        Eigen::Vector3d ba(parameters[4]);
        Eigen::Vector3d bg(parameters[5]);

        Eigen::Matrix3d dp_dba = pre_integration_->jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = pre_integration_->jacobian.block<3, 3>(O_P, O_BG);
        Eigen::Matrix3d dq_dbg = pre_integration_->jacobian.block<3, 3>(O_R, O_BG);
        Eigen::Matrix3d dv_dba = pre_integration_->jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = pre_integration_->jacobian.block<3, 3>(O_V, O_BG);
        double dt = pre_integration_->sum_dt;

        Eigen::Vector3d dba = ba - pre_integration_->linearized_ba;
        Eigen::Vector3d dbg = bg - pre_integration_->linearized_bg;
        if (dba.norm() > 0.1 || dbg.norm() > 0.1) {
            // repropagate
            pre_integration_->repropagate(ba, bg);
            dp_dba = pre_integration_->jacobian.block<3, 3>(O_P, O_BA);
            dp_dbg = pre_integration_->jacobian.block<3, 3>(O_P, O_BG);
            dq_dbg = pre_integration_->jacobian.block<3, 3>(O_R, O_BG);
            dv_dba = pre_integration_->jacobian.block<3, 3>(O_V, O_BA);
            dv_dbg = pre_integration_->jacobian.block<3, 3>(O_V, O_BG);
            residual.block<3, 1>(0, 0) = pre_integration_->delta_p - Tic_ + R_c0_bk_.transpose() * R_c0_bkplus1_ * Tic_
                                         - R_c0_bk_.transpose() * (t_c0_ckplus1_ - t_c0_ck_) * s - 0.5 * R_c0_bk_.transpose() * dt * dt * Rg * g0_ + vb_k * dt;
            residual.block<3, 1>(3, 0) = pre_integration_->delta_v - R_c0_bk_.transpose() *
                                                                     (R_c0_bkplus1_ * vb_kplus1 -
                                                                      R_c0_bk_ * vb_k + Rg * g0_ * dt);
            residual.block<3, 1>(6, 0) = (pre_integration_->delta_q.inverse() * Eigen::Quaterniond(
                    R_c0_bk_.transpose() * R_c0_bkplus1_)).vec();
        } else {
            // linearize at last bias
            Eigen::Quaterniond corrected_delta_q = pre_integration_->delta_q * Utility::deltaQ(dq_dbg * dbg);
            Eigen::Vector3d corrected_delta_v = pre_integration_->delta_v + dv_dba * dba + dv_dbg * dbg;
            Eigen::Vector3d corrected_delta_p = pre_integration_->delta_p + dp_dba * dba + dp_dbg * dbg;
            residual.block<3, 1>(0, 0) = corrected_delta_p - Tic_ + R_c0_bk_.transpose() * R_c0_bkplus1_ * Tic_
                                         - R_c0_bk_.transpose() * (t_c0_ckplus1_ - t_c0_ck_) * s - 0.5 * R_c0_bk_.transpose() * dt * dt * Rg * g0_ + vb_k * dt;
            residual.block<3, 1>(3, 0) = corrected_delta_v - R_c0_bk_.transpose() *
                                                                     (R_c0_bkplus1_ * vb_kplus1 -
                                                                      R_c0_bk_ * vb_k + Rg * g0_ * dt);
            residual.block<3, 1>(6, 0) = (corrected_delta_q.inverse() * Eigen::Quaterniond(
                    R_c0_bk_.transpose() * R_c0_bkplus1_)).vec();
        }


        /// weighted with imu pre-integration covariance
/*        Matrix<double, 9, 9> cov = pre_integration_->covariance.block<9, 9>(0, 0);
        Matrix<double, 9, 9> cov_inv = cov.inverse();
        Matrix<double, 9, 9> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 9, 9>>(cov_inv).matrixL().transpose();
        residual = sqrt_info * residual * 1e-5;*/
        // LOG(INFO) << "sqrt info \n" << sqrt_info;
        // LOG(INFO) << "residual " << residual.transpose();

        if (jacobians) {
            Eigen::Matrix3d g0_cross_mat;
            g0_cross_mat << 0.0, -g0_.z(), g0_.y(), g0_.z(), 0.0, -g0_.x(), -g0_.y(), g0_.x(), 0.0;
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor>> jacobian_abr_Rg(jacobians[0]);
                jacobian_abr_Rg.setZero();
                jacobian_abr_Rg.block<3,3>(0,0) = 0.5 *  R_c0_bk_.transpose() * dt * dt * Rg * g0_cross_mat;
                jacobian_abr_Rg.block<3,3>(3,0) = R_c0_bk_.transpose() * dt * Rg * g0_cross_mat;
            }
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_abr_vbk(jacobians[1]);
                jacobian_abr_vbk.setZero();
                jacobian_abr_vbk.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * dt;
                jacobian_abr_vbk.block<3,3>(3,0) = Eigen::Matrix3d::Identity();
            }
            if (jacobians[2]) {
                Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_abr_vb_kplus1(jacobians[2]);
                jacobian_abr_vb_kplus1.setZero();
                jacobian_abr_vb_kplus1.block<3,3>(3,0) = -R_c0_bk_.transpose() * R_c0_bkplus1_;
            }
            if (jacobians[3]) {
                Eigen::Map<Eigen::Matrix<double, 9, 1>> jacobian_abr_s(jacobians[3]);
                jacobian_abr_s.setZero();
                jacobian_abr_s.block<3,1>(0,0) = -R_c0_bk_.transpose() * (t_c0_ckplus1_ - t_c0_ck_);
            }
            if (jacobians[4]) {
                Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_abr_ba(jacobians[4]);
                jacobian_abr_ba.setZero();
                jacobian_abr_ba.block<3,3>(0,0) = dp_dba;
                jacobian_abr_ba.block<3,3>(3,0) = dv_dba;
            }
            if (jacobians[5]) {
                Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_abr_bg(jacobians[5]);
                jacobian_abr_bg.setZero();
                jacobian_abr_bg.block<3,3>(0,0) = dp_dbg;
                jacobian_abr_bg.block<3,3>(3,0) = dv_dbg;
                jacobian_abr_bg.block<3,3>(6,0) = 0.5 * dq_dbg;
            }
        }
        LOG_EVERY_N(WARNING, vins_estimator::WINDOW_SIZE) << "DEBUG Inertial only optimization iteration--------------";
        LOG_EVERY_N(WARNING, vins_estimator::WINDOW_SIZE) << "DEBUG acc bias iteration " << ba.transpose() << " norm: " << ba.norm();
        LOG_EVERY_N(WARNING, vins_estimator::WINDOW_SIZE) << "DEBUG gyr bias iteration " << bg.transpose() << " norm: " << bg.norm();
        LOG_EVERY_N(WARNING, vins_estimator::WINDOW_SIZE) << "DEBUG scale iteration " << s;
        LOG_EVERY_N(WARNING, vins_estimator::WINDOW_SIZE) << "DEBUG g iteration " << (Rg * g0_).transpose() << " norm: " << (Rg * g0_).norm();
        LOG_EVERY_N(WARNING, vins_estimator::WINDOW_SIZE) << "DEBUG residual iteration " << residual.norm();
        return true;
    }

private:
    std::shared_ptr<IntegrationBase> pre_integration_;
    Eigen::Matrix3d R_c0_bk_, R_c0_bkplus1_;
    Eigen::Vector3d t_c0_ck_, t_c0_ckplus1_;
    Eigen::Vector3d g0_, Tic_;
};