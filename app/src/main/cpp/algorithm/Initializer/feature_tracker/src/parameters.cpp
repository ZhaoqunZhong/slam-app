#include "parameters.h"

namespace Initializer::feature_tracker {
    std::string IMAGE_TOPIC;
    std::string IMU_TOPIC;
    std::vector<std::string> CAM_NAMES;
    std::string FISHEYE_MASK;
    int MAX_CNT;
    int MIN_DIST;
    int WINDOW_SIZE;
    int FREQ;
    double F_THRESHOLD;
    int SHOW_TRACK;
    int STEREO_TRACK;
    int EQUALIZE;
    int ROW;
    int COL;
    int FOCAL_LENGTH;
    int FISHEYE;
    bool PUB_THIS_FRAME;

    void readParameters(std::string config)
    {
        cv::FileStorage fsSettings(config, cv::FileStorage::READ);
/*    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }*/

        // std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

        fsSettings["image_topic"] >> IMAGE_TOPIC;
        fsSettings["imu_topic"] >> IMU_TOPIC;
        MAX_CNT = fsSettings["max_cnt"];
        MIN_DIST = fsSettings["min_dist"];
        ROW = fsSettings["image_height"];
        COL = fsSettings["image_width"];
        FREQ = fsSettings["freq"];
        F_THRESHOLD = fsSettings["F_threshold"];
        SHOW_TRACK = fsSettings["show_track"];
        EQUALIZE = fsSettings["equalize"];
        FISHEYE = fsSettings["fisheye"];
        if (FISHEYE == 1)
            FISHEYE_MASK = "sdcard/orbbec-vio-data/vins-mono/fisheye_mask.jpg";
        CAM_NAMES.push_back(config);

        WINDOW_SIZE = 10;
        STEREO_TRACK = false;
        FOCAL_LENGTH = 460;
        PUB_THIS_FRAME = false;

        if (FREQ == 0)
            FREQ = 10;

        fsSettings.release();
    }
}
