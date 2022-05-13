#pragma once

#include <vector>
using namespace std;

#include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"
using namespace Eigen;


class MotionEstimator
{
  public:

    bool solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &R, Vector3d &T);

  private:
    double testTriangulation(const vector<cv::Point2f> &l,
                             const vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
};



class RotationMatrixLocalParameterization : public ceres::LocalParameterization {
public:
    ~RotationMatrixLocalParameterization() {}

    bool Plus(const double *x, const double *delta, double *x_plus_delta) const {
        Eigen::Map<const Eigen::Matrix<double, 3, 3>> R(x);
        Eigen::Map<Eigen::Matrix<double, 3, 3>> R_prime(x_plus_delta);
        Eigen::Map<const Eigen::Vector3d> tmp(delta);
        Eigen::AngleAxisd d_theta(tmp.norm(), tmp.normalized());
        Eigen::Matrix3d d_R = d_theta.toRotationMatrix();
        R_prime = R * d_R;
        return true;
    };

    bool ComputeJacobian(const double *x, double *jacobian) const {
        Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> J(jacobian);
        J.topRows(3).setIdentity();
        J.bottomRows(6).setZero();
        return true;
    };

    int GlobalSize() const { return 9; };

    int LocalSize() const { return 3; };
};


class FrameToFrameCoplanarConstraint : public ceres::SizedCostFunction<1, 9, 9> {
public:
    FrameToFrameCoplanarConstraint(const Eigen::Vector3d p1, const Eigen::Vector3d p2,
                                   const Eigen::Vector3d t0) : p1_(p1), p2_(p2), t0_(t0) {}

    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        Eigen::Map<const Eigen::Matrix<double, 3, 3>> R(parameters[0]);
        Eigen::Map<const Eigen::Matrix<double, 3, 3>> Rt(parameters[1]);
        *residuals = p1_.transpose() * (Rt * t0_).cross(R * p2_);

        if (jacobians) {
            Eigen::Vector3d p2_prime = R * p2_;
            Eigen::Matrix3d p2_prime_cross_mat;
            p2_prime_cross_mat
                    << 0.0, -p2_prime.z(), p2_prime.y(), p2_prime.z(), 0.0, -p2_prime.x(), -p2_prime.y(), p2_prime.x(), 0.0;

            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 1, 9>> J_R(jacobians[0]);
                J_R.leftCols(3) = -p1_.transpose() * p2_prime_cross_mat;
                J_R.rightCols(6).setZero();
            }
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 1, 9>> J_Rt(jacobians[1]);
                Eigen::Matrix3d t0_cross_mat;
                t0_cross_mat
                        << 0.0, -t0_.z(), t0_.y(), t0_.z(), 0.0, -t0_.x(), -t0_.y(), t0_.x(), 0.0;
                J_Rt.leftCols(3) = p1_.transpose() * p2_prime_cross_mat * Rt * t0_cross_mat;
                J_Rt.rightCols(6).setZero();
            }
        }
        return true;
    }

private:
    const Eigen::Vector3d p1_, p2_, t0_;
};

class BAsecondViewconstraint : public ceres::SizedCostFunction<2, 9, 9, 3> {
public:
    BAsecondViewconstraint(const Eigen::Vector2d p2, const Eigen::Vector3d t0) : p2_(p2), t0_(t0) {}

    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        Eigen::Map<const Eigen::Matrix<double, 3, 3>> R_transposed(parameters[0]);
        Eigen::Map<const Eigen::Matrix<double, 3, 3>> Rt(parameters[1]);
        Eigen::Map<const Eigen::Vector3d> P1(parameters[2]);
        Eigen::Map<Eigen::Vector2d> r(residuals);
        Eigen::Vector3d P2 = R_transposed * (P1 - Rt * t0_);
        r.x() = P2.x() / P2.z() - p2_.x();
        r.y() = P2.y() / P2.z() - p2_.y();

        Eigen::Matrix<double, 2, 3> J_r_P2;
        J_r_P2 << 1 / P2.z(), 0, -P2.x() / (P2.z() * P2.z()), 0, 1 / P2.z(), -P2.y() /
                                                                             (P2.z() * P2.z());
        if (jacobians) {
            if (jacobians[0]) {/// Should the jacobian be in row major form? -> Yes! Check ceres doc!
                Eigen::Map<Eigen::Matrix<double, 2, 9, RowMajor>> J_r_Rtransposed(jacobians[0]);
                Eigen::Matrix3d J_P2_Rtransposed =
                        -R_transposed * Utility::skewSymmetric(P1 - Rt * t0_);
                J_r_Rtransposed.leftCols(3) = J_r_P2 * J_P2_Rtransposed;
                J_r_Rtransposed.rightCols(6).setZero();
            }
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 2, 9, RowMajor>> J_r_Rt(jacobians[1]);
                Eigen::Matrix3d J_P2_Rt = R_transposed * Rt * Utility::skewSymmetric(t0_);
                J_r_Rt.leftCols(3) = J_r_P2 * J_P2_Rt;
                J_r_Rt.rightCols(6).setZero();
            }
            if (jacobians[2]) {
                Eigen::Map<Eigen::Matrix<double, 2, 3, RowMajor>> J_r_P1(jacobians[2]);
                Eigen::Matrix3d J_P2_P1 = R_transposed;
                J_r_P1 = J_r_P2 * J_P2_P1;
            }
        }
        return true;
    }

private:
    const Eigen::Vector2d p2_;
    const Eigen::Vector3d t0_;
};

class BAfirstViewconstraint : public ceres::SizedCostFunction<2, 3> {
public:
    BAfirstViewconstraint(const Eigen::Vector2d p1) : p1_(p1) {}

    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        Eigen::Map<const Eigen::Vector3d> P1(parameters[0]);
        Eigen::Map<Eigen::Vector2d> r(residuals);
        r.x() = P1.x() / P1.z() - p1_.x();
        r.y() = P1.y() / P1.z() - p1_.y();

        if (jacobians) {
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 2, 3, RowMajor>> J_P1(jacobians[0]);
                J_P1 << 1 / P1.z(), 0, -P1.x() / (P1.z() * P1.z()), 0, 1 / P1.z(), -P1.y() /
                                                                                   (P1.z() *
                                                                                    P1.z());
            }
        }
        return true;
    }

private:
    const Eigen::Vector2d p1_;
};