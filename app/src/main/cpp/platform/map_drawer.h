
#ifndef VINSONANDROID_MAP_DRAWER_H
#define VINSONANDROID_MAP_DRAWER_H

//#include <Eigen/Eigen>
#include <GLES3/gl32.h>
#include <vector>
#include <pthread.h>
#include <array>
#include "native_debug.h"

class MapDrawer {
public:
    MapDrawer(){};
    ~MapDrawer(){};

    void init();
    void draw();

    float vp_matrix_[16];

private:
    void drawCurrentBody(float Twb[], float color[]);

    void drawMapPoint(std::vector<std::array<float,6>> &);

    void drawKeyFrames(std::vector<Eigen::Matrix4d> & kfs, float color[]);

    void drawTrajectory(std::vector<std::array<float,3>> &traj, float color[]);

    void drawMonoMapPoint(std::vector<std::array<float,3>> &map, float color[]);

    void drawWorld();

    GLuint mProg;
    GLuint mVBOs[7], mVAOs[3];
    GLint mVPMtrx;
    GLint mBodyMtrx;
    GLint mPos;
    GLint mColor, mColorMap;
    GLint mPerVertexColor;

    float mIdentity[16]{
            1.0f, 0, 0, 0,
            0, 1.0f, 0, 0,
            0, 0, 1.0f, 0,
            0, 0, 0, 1.0f
    };
    float cam_init_pose_[16]{ //column-major storage
//            1, 0, 0, 0,
//            0, 0, -1, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 1
            1.0f, 0, 0, 0,
            0, 1.0f, 0, 0,
            0, 0, 1.0f, 0,
            0, 0, 0, 1.0f
    };
    std::vector<float> grid_, assist_grid_;
    std::vector<std::array<float,3>> body_;
};



#endif //VINSONANDROID_MAP_DRAWER_H
