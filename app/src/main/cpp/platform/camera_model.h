#ifndef VID_CAMERA_MODEL_H
#define VID_CAMERA_MODEL_H

#include <string>
#include "Eigen/Core"
#include <Eigen/Dense>
#include "opencv2/core/eigen.hpp" //#include <Eigen/Dense> must be put before this, otherwise cv2eigen causes error
#include "native_debug.h"

enum camera_type {RAD_TAN, FISH_EYE} ;

class CameraModel {
public:
    CameraModel(camera_type type, bool no_distort, int image_w, int image_h, Eigen::Matrix3d intrin_mat,
                std::vector<double> distort_paras) : distort_type_(type), no_distortion_(no_distort), rgb_w_(image_w),
                rgb_h_(image_h), rgb_k_mat_(intrin_mat), rgb_distort_paras_(distort_paras)
    {
        inv_rgb_k_mat_ = rgb_k_mat_.inverse();
        cv::eigen2cv(rgb_k_mat_, rgb_intrinsic_cv_);
        if (distort_type_ == FISH_EYE && !no_distort)
            ASSERT(rgb_distort_paras_.size() == 4, "FISH_EYE distortion paras number should be 4!");
    };
    ~CameraModel() {};

    bool no_distortion_ = false;
    int rgb_w_, rgb_h_;
    Eigen::Matrix3d rgb_k_mat_, inv_rgb_k_mat_;
    cv::Mat rgb_intrinsic_cv_;
    std::vector<double> rgb_distort_paras_;

//    bool readFromYamlFile(std::string config_file);
    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const;
    bool inRange(const cv::Point2f &pt);
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P);
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p);

private:
    camera_type distort_type_;
    template<typename T> static T distortFisheyeViewAngle(T k2, T k3, T k4, T k5, T theta);
    void backprojectSymmetric(const Eigen::Vector2d& p_u, double& theta, double& phi) const;
};

#endif //VID_CAMERA_MODEL_H
