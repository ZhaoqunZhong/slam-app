//
// Created by zhong on 2021/11/24.
//

#include "motion_only_estimator.h"

void MotionOnlyEstimator::updateBackend(Eigen::Matrix3d Rs[], Eigen::Vector3d Ps[],
                                        Eigen::Vector3d &Vbk, Eigen::Vector3d &ba, Eigen::Vector3d &bg,
                                        double &header_bk, double &td_in, list<FeaturePerId> &feature) {
    backend_mtx_.lock();
    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        Rwbs[i] = Rs[i];
        Pwbs[i] = Ps[i];
    }
    Vb_end = Vbk;
    Ba_end = ba;
    Bg_end = bg;
    ts_end = header_bk;
    td = td_in;
    feature_end.clear();
    for (auto &it_per_id : feature)
    {
        if (it_per_id.endFrame() == WINDOW_SIZE && it_per_id.start_frame < WINDOW_SIZE) {
            Eigen::Vector3d p_first_c = it_per_id.feature_per_frame.front().point * it_per_id.estimated_depth;
            Eigen::Vector3d p_first_b = RIC[0] * p_first_c + TIC[0];
            int l = it_per_id.start_frame;
            Eigen::Vector3d p_world = Rwbs[l] * p_first_b + Pwbs[l];
            feature_end[it_per_id.feature_id] = p_world;
        }
    }
    backend_mtx_.unlock();
}

void MotionOnlyEstimator::updateFrontend(Eigen::Matrix3d Rs[], Eigen::Vector3d Ps[],
                                        Eigen::Vector3d &Vbk, Eigen::Vector3d &ba, Eigen::Vector3d &bg,
                                        double &header_bk, double &td_out, unordered_map<int, Vector3d> &feature) {
    backend_mtx_.lock();
    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        Rs[i] = Rwbs[i];
        Ps[i] = Pwbs[i];
    }
    Vbk = Vb_end;
    ba = Ba_end;
    bg = Bg_end;
    header_bk = ts_end;
    td_out = td;
    feature = feature_end;
    backend_mtx_.unlock();
}
