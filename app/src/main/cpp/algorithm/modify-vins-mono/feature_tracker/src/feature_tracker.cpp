#include "feature_tracker.h"
#include "glog/logging.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


FeatureTracker::FeatureTracker()
{
}

void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}
void FeatureTracker::mySetMask() {
    mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

    for (auto &it : forw_pts) {
        cv::circle(mask, it, MIN_DIST, 0, -1);
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        // ids.push_back(-1);
        ids.push_back(n_id++);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat _img, double _cur_time)
{
    cv::Mat img;
    // TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        LOG(INFO) << "CLAHE costs: "<< t_c.toc() << " ms";
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        /*prev_img = */cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status, backflow_status;
        vector<float> err;
        vector<cv::Point2f> backflow_pts;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
        cv::calcOpticalFlowPyrLK(forw_img, cur_img, forw_pts, backflow_pts, backflow_status, err, cv::Size(21, 21), 3);
        int forw_cnt=0, back_cnt=0;
        for (int i = 0; i < int(forw_pts.size()); i++) {
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
            if (status[i])
                forw_cnt++;
            if (status[i] && backflow_status[i]) {
                if ( (backflow_pts[i].x - cur_pts[i].x)*(backflow_pts[i].x - cur_pts[i].x)
                    +(backflow_pts[i].y - cur_pts[i].y)*(backflow_pts[i].y - cur_pts[i].y)
                    > 1) // pixel error
                    status[i] = 0;
                else
                    back_cnt++;
            }
        }
        // LOG(WARNING) << "DEBUG forw_cnt " << forw_cnt << " back_cnt " << back_cnt;
        // LOG(WARNING) << "DEBUG backward optical flow reject " << 100.0*(forw_cnt - back_cnt) / forw_cnt << "%";
        // reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        // reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        //LOG(INFO) << "temporal optical flow costs: " << t_o.toc() << " ms";

    }
    // LOG(WARNING) << "DEBUG forw_pts size " << forw_pts.size();
    // LOG(WARNING) << "DEBUG cur_pts size " << cur_pts.size();
    // LOG(WARNING) << "DEBUG ids size " << ids.size();
    // LOG(WARNING) << "DEBUG track_cnt size " << track_cnt.size();

    for (auto &n : track_cnt)
        n++;

    int feature_drop = MAX_CNT - static_cast<int>(forw_pts.size());
    // if (PUB_THIS_FRAME)
    if (feature_drop > 0.25 * MAX_CNT)
    {
        // rejectWithF();
        //LOG(INFO) << "set mask begins";
        // TicToc t_m;
        // setMask();
        mySetMask();
        // LOG(INFO) << "set mask costs " << t_m.toc() << " ms";

        //LOG(INFO) << "detect feature begins ";
        // TicToc t_t;

        if (feature_drop > 0)
        {
            if (mask.empty())
                LOG(WARNING) << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                LOG(WARNING) << "mask type wrong " << endl;
            if (mask.size() != forw_img.size()) {
                LOG(WARNING) << "wrong size " << endl;
                LOG(WARNING) << "mask size " << mask.size;
                LOG(WARNING) << "forw_img size " << forw_img.size;
            }
            cv::goodFeaturesToTrack(forw_img, n_pts, feature_drop, 0.01, MIN_DIST, mask);
            LOG(INFO) << "Need to detect new features: " << feature_drop;
        }
        else
            n_pts.clear();
        //LOG(INFO) << "detect feature costs: " << t_t.toc() << " ms";

        //LOG(INFO) << "add feature begins ";
        // TicToc t_a;
        addPoints();
        //LOG(INFO) << "selectFeature costs: " << t_a.toc() << " ms";
    }
    // prev_img = cur_img;
    // prev_pts = cur_pts;
    // prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    undistortedPoints();
    // LOG(WARNING) << "DEBUG ids outside size " << ids.size();
    TicToc tt;
    rollingShutter_F_reject();
    // LOG(WARNING) << "DEBUG rollingShutter_F_reject cost " << tt.toc() << "ms";

/*    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        completed |= updateID(i);

        if (!completed)
            break;
    }*/
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        //LOG(INFO) << "FM ransac begins ";
        // TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        // int size_a = cur_pts.size();
        // reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        // reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        //LOG(INFO) << "FM ransac: " << size_a << " -> " << forw_pts.size() << ": " << 1.0 * forw_pts.size() / size_a;
        //LOG(INFO) << "FM ransac costs: " << t_f.toc() << " ms";
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    //LOG(INFO) << "reading paramerter of camera " << calib_file.c_str();
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}
/*
void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}*/

void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    cur_rs_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
/*            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }*/
            std::map<int, cv::Point2f>::iterator it;
            it = prev_un_pts_map.find(ids[i]);
            if (it != prev_un_pts_map.end())
            {
                double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));

                double feature_dt = (cur_pts[i].y / ROW) * image_readout_ns * 1e-9;
                double rs_x = cur_un_pts[i].x - v_x * feature_dt;
                double rs_y = cur_un_pts[i].y - v_y * feature_dt;
                cur_rs_un_pts_map.insert(make_pair(ids[i], cv::Point2f(rs_x, rs_y)));
                // LOG(INFO) << "feature dt " << feature_dt;
                // LOG(INFO) << "cur_un_pts[i].x " << cur_un_pts[i].x << " v_x * feature_dt " << v_x * feature_dt;
                // LOG(INFO) << "cur_un_pts[i].y " << cur_un_pts[i].y << " v_y * feature_dt " << v_y * feature_dt;
                /// consider rolling shutter from here
                cur_un_pts[i].x = rs_x;
                cur_un_pts[i].y = rs_y;
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
}

void FeatureTracker::rollingShutter_F_reject() {
    if (PUB_THIS_FRAME) {
    //     LOG(WARNING) << "DEBUG ids inside size " << ids.size();
        if (!prev_rs_un_pts_map.empty()) {
            if (cur_rs_un_pts_map.size() >= 8)
            {
                vector<cv::Point2f> un_rs_cur_pts, un_rs_forw_pts;
                vector<int> rs_pts_ids;
                for (auto & p : cur_rs_un_pts_map)
                {
                    std::map<int, cv::Point2f>::iterator it;
                    it = prev_rs_un_pts_map.find(p.first);
                    if (it != prev_rs_un_pts_map.end()) {
 /*                       cv::Point2f un_rs_cur_pt_backproject, un_rs_forw_pt_backproject;
                        un_rs_cur_pt_backproject.x = FOCAL_LENGTH * p.second.x + COL / 2.0;
                        un_rs_cur_pt_backproject.y = FOCAL_LENGTH * p.second.y + ROW / 2.0;
                        un_rs_cur_pts.push_back(un_rs_cur_pt_backproject);
                        un_rs_forw_pt_backproject.x = FOCAL_LENGTH * it->second.x + COL / 2.0;
                        un_rs_forw_pt_backproject.y = FOCAL_LENGTH * it->second.y + ROW / 2.0;
                        un_rs_forw_pts.push_back(un_rs_forw_pt_backproject);*/
                        un_rs_cur_pts.push_back(p.second);
                        un_rs_forw_pts.push_back(it->second);
                        rs_pts_ids.push_back(p.first);
                    }
                }
                // LOG(WARNING) << "DEBUG rs pts size " << rs_pts_ids.size();
                if (rs_pts_ids.size() >= 8) {
                    vector<uchar> rs_status;
                    queue<int> invalid_ids;
                    // LOG(WARNING) << "DEBUG POINT!";
                    cv::findFundamentalMat(un_rs_cur_pts, un_rs_forw_pts, cv::FM_RANSAC,
                                           F_THRESHOLD / FOCAL_LENGTH, 0.99, rs_status);
                    for (int i = 0; i < rs_status.size(); i++) {
                        if (!rs_status[i]) {
                            invalid_ids.push(rs_pts_ids[i]);
                            cur_rs_un_pts_map.erase(rs_pts_ids[i]);
                            // LOG(WARNING) << "DEBUG invalid id " << rs_pts_ids[i];
                        }
                    }
                    // LOG(WARNING) << "DEBUG  ----------------------" ;
                    // LOG(WARNING) << "DEBUG rolling shutter F reject " << 100.0 * invalid_ids.size() / rs_pts_ids.size() << "%";
                    int j = 0;
                    int smallest_id = -1;
                    // LOG(WARNING) << "DEBUG ids size " << ids.size();
                    for (int i = 0; i < ids.size(); i++) {
                        if (!invalid_ids.empty())
                            smallest_id = invalid_ids.front();
                        // LOG(WARNING) << "DEBUG smallest_id " << smallest_id;
                        if (ids[i] != smallest_id) {
                            ids[j] = ids[i];
                            cur_pts[j] = cur_pts[i];
                            track_cnt[j] = track_cnt[i];
                            cur_un_pts[j] = cur_un_pts[i];
                            pts_velocity[j] = pts_velocity[i];
                            j++;
                        } else {
                            if (!invalid_ids.empty())
                                invalid_ids.pop();
                        }

                    }
                    // LOG(WARNING) << "DEBUG j " << j;
                    ids.resize(j);
                    cur_pts.resize(j);
                    track_cnt.resize(j);
                    cur_un_pts.resize(j);
                    pts_velocity.resize(j);
                }
            }
        }

        prev_rs_un_pts_map = cur_rs_un_pts_map;
    }
}
