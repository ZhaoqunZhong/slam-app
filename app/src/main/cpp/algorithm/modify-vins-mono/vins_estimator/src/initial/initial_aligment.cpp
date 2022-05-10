#include "initial_alignment.h"
#include "initial_sfm.h"

void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs)
{
    Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    A.setZero();
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        frame_j = next(frame_i);
        MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;

    }
    delta_bg = A.ldlt().solve(b);
    LOG(WARNING) << "gyroscope bias initial calibration: " << delta_bg.transpose();

    for (int i = 0; i <= WINDOW_SIZE; i++)
        Bgs[i] += delta_bg;

    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
    }
}


MatrixXd TangentBasis(Vector3d &g0)
{
    Vector3d b, c;
    Vector3d a = g0.normalized();
    Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}

void RefineGravity(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x)
{
    Vector3d g0 = g.normalized() * G.norm();
    Vector3d lx, ly;
    //VectorXd x;
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 2 + 1;

    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for(int k = 0; k < 4; k++)
    {
        MatrixXd lxly(3, 2);
        lxly = TangentBasis(g0);
        int i = 0;
        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
        {
            frame_j = next(frame_i);

            MatrixXd tmp_A(6, 9);
            tmp_A.setZero();
            VectorXd tmp_b(6);
            tmp_b.setZero();

            double dt = frame_j->second.pre_integration->sum_dt;


            tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
            tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;
            tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;     
            tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0] - frame_i->second.R.transpose() * dt * dt / 2 * g0;

            tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
            tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
            tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity() * lxly;
            tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v - frame_i->second.R.transpose() * dt * Matrix3d::Identity() * g0;


            Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
            //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
            //MatrixXd cov_inv = cov.inverse();
            cov_inv.setIdentity();

            MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
            VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
            b.segment<6>(i * 3) += r_b.head<6>();

            A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
            b.tail<3>() += r_b.tail<3>();

            A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
            A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
        }
            A = A * 1000.0;
            b = b * 1000.0;
            x = A.ldlt().solve(b);
            VectorXd dg = x.segment<2>(n_state - 3);
            g0 = (g0 + lxly * dg).normalized() * G.norm();
            //double s = x(n_state - 1);
    }   
    g = g0;
}


void RefineGravityAndBias(map<double, ImageFrame> &all_image_frame, Vector3d &g, Vector3d &ba, Vector3d &bg)
{
    Vector3d g0 = g.normalized() * G.norm();
    // Vector3d lx, ly;
    //VectorXd x;
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 2 + 1 + 6;
    int m = (all_frame_count - 1) * 3 * 3;

    MatrixXd A{m, n_state};
    A.setZero();
    VectorXd b{m};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for(int k = 0; k < 20; k++)
    {
        MatrixXd lxly(3, 2);
        lxly = TangentBasis(g0);
        int i = 0;
        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
        {
            frame_j = next(frame_i);
            TicToc tt;
            frame_j->second.pre_integration->repropagate(ba, bg); // repropagate imu pre-integration with new biases
            // LOG(WARNING) << "DEBUG imu re propagate time: " << tt.toc();

            // x = [dg v0...vk s dba dbg]
            MatrixXd tmp_A(9, 15);
            tmp_A.setZero();
            VectorXd tmp_b(9);
            tmp_b.setZero();

            Eigen::Matrix3d dp_dba = frame_j->second.pre_integration->jacobian.block<3, 3>(O_P, O_BA);
            Eigen::Matrix3d dp_dbg = frame_j->second.pre_integration->jacobian.block<3, 3>(O_P, O_BG);
            Eigen::Matrix3d dq_dbg = frame_j->second.pre_integration->jacobian.block<3, 3>(O_R, O_BG);
            Eigen::Matrix3d dv_dba = frame_j->second.pre_integration->jacobian.block<3, 3>(O_V, O_BA);
            Eigen::Matrix3d dv_dbg = frame_j->second.pre_integration->jacobian.block<3, 3>(O_V, O_BG);

            double dt = frame_j->second.pre_integration->sum_dt;
            tmp_A.block<3, 3>(0, 0) = frame_i->second.R.transpose() * dt * dt / 2 * lxly;
            tmp_A.block<3, 3>(0, 2) = -dt * Matrix3d::Identity();
            tmp_A.block<3, 3>(0, 5) = Eigen::Matrix3d::Zero();
            tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T);
            tmp_A.block<3, 3>(0, 9) = -dp_dba;
            tmp_A.block<3, 3>(0, 12) = -dp_dbg;
            tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0]
                                      - 0.5 * frame_i->second.R.transpose() * dt * dt * g0;

            tmp_A.block<3, 3>(3, 0) = frame_i->second.R.transpose() * dt * lxly;
            tmp_A.block<3, 3>(3, 2) = -Matrix3d::Identity();
            tmp_A.block<3, 3>(3, 5) = frame_i->second.R.transpose() * frame_j->second.R;
            tmp_A.block<3, 1>(3, 8) = Eigen::Vector3d::Zero();
            tmp_A.block<3, 3>(3, 9) = -dv_dba;
            tmp_A.block<3, 3>(3, 12) = -dv_dbg;
            tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v - frame_i->second.R.transpose() * dt * g0;

            tmp_A.block<3, 3>(6, 12) = dq_dbg / 2;
            tmp_b.block<3, 1>(6, 0) = (frame_j->second.pre_integration->delta_q.inverse() *
                                           Eigen::Quaterniond(frame_i->second.R.transpose() * frame_j->second.R)).vec();

/*            /// weighted with imu pre-integration covariance
            Matrix<double, 9, 9> cov = frame_j->second.pre_integration->covariance.block<9, 9>(0, 0);
            Matrix<double, 9, 9> cov_inv = cov.inverse();
            Matrix<double, 9, 9> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 9, 9>>(cov_inv).matrixL().transpose();
            tmp_A = sqrt_info * tmp_A * 1e-5;
            tmp_b = sqrt_info * tmp_b * 1e-5;*/

            A.block<9, 2>(i * 9, 0) += tmp_A.block<9,2>(0, 0); // dg part
            A.block<9, 3>(i * 9, 2 + 3 * i) += tmp_A.block<9,3>(0, 2); // vb_k part
            A.block<9, 3>(i * 9, 5 + 3 * i) += tmp_A.block<9,3>(0, 5); // vb_k+1 part
            A.block<9, 1>(i * 9, n_state - 7) += tmp_A.block<9,1>(0, 8); // s part
            A.block<9, 3>(i * 9, n_state - 6) += tmp_A.block<9,3>(0, 9); // ba part
            A.block<9, 3>(i * 9, n_state - 3) += tmp_A.block<9,3>(0, 12); // bg part
            b.segment<9>(i * 9) += tmp_b;
        }
        JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
        VectorXd x = svd.solve(b);
        VectorXd dg = x.segment<2>(0);
        g0 = (g0 + lxly * dg).normalized() * G.norm();
        //double s = x(n_state - 1);
        Eigen::Vector3d dba = x.segment<3>(n_state -6);
        Eigen::Vector3d dbg = x.segment<3>(n_state -3);
        ba = ba + dba;
        bg = bg + dbg;
        LOG(WARNING) << "Refined ba " << ba.transpose() << " with norm: " << ba.norm();
        LOG(WARNING )<< "Refined bg " << bg.transpose() << " with norm: " << bg.norm();
        LOG(WARNING) << "Refined g " << g0.transpose();
        LOG(WARNING) << "Ax - b norm: " << (A*x - b).norm();
    }
    g = g0;
}

bool LinearAlignment(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x)
{
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 3 + 1;

    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);

        MatrixXd tmp_A(6, 10);
        tmp_A.setZero();
        VectorXd tmp_b(6);
        tmp_b.setZero();

        double dt = frame_j->second.pre_integration->sum_dt;

        tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
        tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;     
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0];
        //cout << "delta_p   " << frame_j->second.pre_integration->delta_p.transpose() << endl;
        tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
        tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity();
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
        //cout << "delta_v   " << frame_j->second.pre_integration->delta_v.transpose() << endl;

        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
        //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        //MatrixXd cov_inv = cov.inverse();
        cov_inv.setIdentity();

        MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
        b.segment<6>(i * 3) += r_b.head<6>();

        A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
        b.tail<4>() += r_b.tail<4>();

        A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
        A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    double s = x(n_state - 1) / 100.0;
    LOG(INFO) << "estimated scale: " << s;
    g = x.segment<3>(n_state - 4);
    LOG(INFO) << "result g norm: " << g.norm() << " g "  << g.transpose();
    if(fabs(g.norm() - G.norm()) > 1.0 || s < 0)
    {
        return false;
    }

    RefineGravity(all_image_frame, g, x);
    s = (x.tail<1>())(0) / 100.0;
    (x.tail<1>())(0) = s;
    LOG(INFO) << "refined g norm: " << g.norm() << " g "  << g.transpose();
    if(s < 0.0 )
        return false;   
    else
        return true;
}

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x)
{
    // solveGyroscopeBias(all_image_frame, Bgs);

    if(LinearAlignment(all_image_frame, g, x))
        return true;
    else 
        return false;
}
