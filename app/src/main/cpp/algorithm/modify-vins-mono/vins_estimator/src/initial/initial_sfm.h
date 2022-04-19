#pragma once 
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "../factor/projection_factor.h"
#include "../parameters.h"

using namespace Eigen;
using namespace std;


struct SFMFeature
{
    bool state;
    int id;
    vector<pair<int,Vector2d>> observation;
    double position[3];
    double depth;
};

struct ReprojectionError3D
{
	ReprojectionError3D(double observed_u, double observed_v)
		:observed_u(observed_u), observed_v(observed_v)
		{}

	template <typename T>
	bool operator()(const T* const camera_R, const T* const camera_T, const T* point, T* residuals) const
	{
		T p[3];
		ceres::QuaternionRotatePoint(camera_R, point, p);
		p[0] += camera_T[0]; p[1] += camera_T[1]; p[2] += camera_T[2];
		T xp = p[0] / p[2];
    	T yp = p[1] / p[2];
    	residuals[0] = xp - T(observed_u);
    	residuals[1] = yp - T(observed_v);
    	return true;
	}

	static ceres::CostFunction* Create(const double observed_x,
	                                   const double observed_y) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          ReprojectionError3D, 2, 4, 3, 3>(
	          	new ReprojectionError3D(observed_x,observed_y)));
	}

	double observed_u;
	double observed_v;
};

class GlobalSFM
{
public:
	GlobalSFM();
	bool construct(int frame_num, Quaterniond* q, Vector3d* T, int l,
			  const Matrix3d relative_R, const Vector3d relative_T,
			  vector<SFMFeature> &sfm_f, map<int, Vector3d> &sfm_tracked_points);

	bool solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i, vector<SFMFeature> &sfm_f);

	void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
							Vector2d &point0, Vector2d &point1, Vector3d &point_3d);
	void triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0, 
							  int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
							  vector<SFMFeature> &sfm_f);

	int feature_num;
};


class ReprojectionAnalytic : public ceres::SizedCostFunction<2, 7, 3> {
public:
	ReprojectionAnalytic(const Vector3d &pt_norm) : p_norm_(pt_norm) {};

	bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
		Eigen::Vector3d t_cw(parameters[0][0], parameters[0][1], parameters[0][2]);
		Eigen::Quaterniond q_cw(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
		Eigen::Map<const Eigen::Vector3d> world_pt(parameters[1]);
		Eigen::Vector3d pts_cam = q_cw._transformVector(world_pt) + t_cw;

		Eigen::Map<Eigen::Vector2d> residual(residuals);
		double dep = pts_cam.z();
		residual = (pts_cam / dep - p_norm_).head<2>();
		// residual = ProjectionFactor::sqrt_info * residual; // doesn't matter because it's pure visual ba

		if (jacobians) {
			Eigen::Matrix3d R_cw = q_cw.toRotationMatrix();
			Eigen::Matrix<double, 2, 3> jaco_r_ptc(2, 3);
			jaco_r_ptc << 1. / dep, 0, -pts_cam.x() / (dep * dep),
					0, 1. / dep, -pts_cam.y() / (dep * dep);
			// jaco_r_ptc = ProjectionFactor::sqrt_info * jaco_r_ptc;

			if (jacobians[0]) {
				Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
				Eigen::Matrix<double, 3, 6> jaco_ptc_pose;
				jaco_ptc_pose.leftCols<3>() = Matrix3d::Identity();
				jaco_ptc_pose.rightCols<3>() = -R_cw * Utility::skewSymmetric(world_pt);

				jacobian_pose.leftCols<6>() = jaco_r_ptc * jaco_ptc_pose;
				jacobian_pose.rightCols<1>().setZero();
			}
			if (jacobians[1]) {
				Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_pt(jacobians[1]);
				jacobian_pt = jaco_r_ptc * R_cw;
			}
		}
		return true;
	}

private:
	const Vector3d p_norm_;
};