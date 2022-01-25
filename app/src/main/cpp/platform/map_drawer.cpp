#include "map_drawer.h"
#include <vector>
#include <string>
#include <algorithm>
#include "imu_publisher.h"
#include "opencv2/core.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include "glog/logging.h"

/*Pay attention that opengl uniform mat4 uses column major storage.*/
const char* vertexShaderSrc = R"(#version 320 es
    precision lowp float;
    uniform float per_vertex_color;
    uniform mat4 vp;
    uniform mat4 body;
    layout (location = 0) in vec3 pos;
    layout (location = 1) in vec3 color;
    out vec3 vertex_color;

    void main()
    {
        gl_PointSize = 6.0;
        gl_Position = vp * body * vec4(pos, 1.0);
        if (per_vertex_color == 1.0) {
            gl_PointSize = 5.0;
            vertex_color = color;
        }
    }
)";
const char* fragShaderSrc = R"(#version 320 es
    precision lowp float;
    uniform vec3 color;
    uniform float per_vertex_color;
    in vec3 vertex_color;
    out vec4 frag_color;

    void main() {
        if (per_vertex_color == 1.0) {
            frag_color = vec4(vertex_color, 1.0);
        } else {
            frag_color = vec4(color, 1.0);
        }
    }
)";



bool checkGlError(const char* funcName) {
    GLint err = glGetError();
    if (err != GL_NO_ERROR) {
        LOGE("GL error after %s(): 0x%08x\n", funcName, err);
        return true;
    }
    return false;
}

GLuint createShader(GLenum shaderType, const char* src) {
    GLuint shader = glCreateShader(shaderType);
    if (!shader) {
        checkGlError("glCreateShader");
        return 0;
    }
    glShaderSource(shader, 1, &src, NULL);

    GLint compiled = GL_FALSE;
    glCompileShader(shader);
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
        GLint infoLogLen = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogLen);
        if (infoLogLen > 0) {
            GLchar* infoLog = (GLchar*)malloc(infoLogLen);
            if (infoLog) {
                glGetShaderInfoLog(shader, infoLogLen, NULL, infoLog);
                LOGE("Could not compile %s shader:\n%s\n",
                      shaderType == GL_VERTEX_SHADER ? "vertex" : "fragment",
                      infoLog);
                free(infoLog);
            }
        }
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

GLuint createProgram(const char* vtxSrc, const char* fragSrc) {
    GLuint vtxShader = 0;
    GLuint fragShader = 0;
    GLuint program = 0;
    GLint linked = GL_FALSE;

    vtxShader = createShader(GL_VERTEX_SHADER, vtxSrc);
    if (!vtxShader)
        goto exit;

    fragShader = createShader(GL_FRAGMENT_SHADER, fragSrc);
    if (!fragShader)
        goto exit;

    program = glCreateProgram();
    if (!program) {
        checkGlError("glCreateProgram");
        goto exit;
    }
    glAttachShader(program, vtxShader);
    glAttachShader(program, fragShader);

    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (!linked) {
        LOGE("Could not link program");
        GLint infoLogLen = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &infoLogLen);
        if (infoLogLen) {
            GLchar* infoLog = (GLchar*)malloc(infoLogLen);
            if (infoLog) {
                glGetProgramInfoLog(program, infoLogLen, NULL, infoLog);
                LOGE("Could not link program:\n%s\n", infoLog);
                free(infoLog);
            }
        }
        glDeleteProgram(program);
        program = 0;
    }

    exit:
    glDeleteShader(vtxShader);
    glDeleteShader(fragShader);
    return program;
}

//size: one-side size, e.g. for 10 * 10 grid, this para is 5.
//space: distance between grid lines.
std::vector<float> genGridVertices(float size, float space) {
    std::vector<float> grid;
    grid.resize((2*size/space + 1)*2*2*3 );
    float b = space;
    while (b <= size) {
        float line[] = {-size, b, 0, size, b, 0,
                        -size, -b, 0, size, -b, 0,
                        -b, -size, 0, -b, size, 0,
                        b, -size, 0, b, size, 0};
        grid.insert(grid.end(), line, line + 24);
        b = b + space;
    }
    float axis[] = {-size, 0, 0, size, 0, 0,
                    0, -size, 0, 0, size, 0};
    grid.insert(grid.end(), axis, axis + 12);
    return grid;
}

std::vector<std::array<float,3>> genBodyVertices() {
    std::vector<std::array<float,3>> body;
    const float w = 0.5;      // width
    const float h = 0.75*w; // height
    const float t = 0.6*w;  //thickness
    std::array<float,3> origin {0, 0, 0};
    std::array<float,3> ur_corner {w/2, -h/2, t};
    std::array<float,3> ul_corner {-w/2, -h/2, t};
    std::array<float,3> bl_corner {-w/2, h/2, t};
    std::array<float,3> br_corner {w/2, h/2, t};
    std::vector<std::array<float,3>> body_indices {origin, ur_corner, ul_corner, bl_corner, br_corner};
    //std::vector<int> idx { 0,1, 0,2, 0,3, 0,4, 1,2, 2,3, 3,4, 4,1 };
    std::vector<int> idx { 0,1, 0,2, 0,3, 0,4, 1,2, 2,3, 3,4};
    for (int &i : idx) {
        body.push_back(body_indices[i]);
    }
    return body;
}

void MapDrawer::init() {
    mProg = createProgram(vertexShaderSrc, fragShaderSrc);
    mPerVertexColor = glGetUniformLocation(mProg, "per_vertex_color");
    mVPMtrx = glGetUniformLocation(mProg, "vp");
    mBodyMtrx = glGetUniformLocation(mProg, "body");
    mColor = glGetUniformLocation(mProg, "color");
    glGenBuffers(7, mVBOs);
    glGenVertexArrays(3, mVAOs);
    //camera body
/*    glBindVertexArray(mVAOs[0]);
    const float w = 0.5;      // width
    const float h = 0.75*w; // height
    const float t = 0.6*w;  //thickness
    float cam_vertices[] {
            0, 0, 0,    //origin
            w/2, -h/2, t,   //up-right corner
            -w/2, -h/2, t,  //up-left corner
            -w/2, h/2, t,   //bottom-left corner
            w/2, h/2, t,    //bottom-right corner
    };
    glBindBuffer(GL_ARRAY_BUFFER, mVBOs[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cam_vertices), cam_vertices, GL_STATIC_DRAW);
    GLuint cam_indices[] { 0,1, 0,2, 0,3, 0,4, 1,2, 2,3, 3,4, 4,1 };
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mVBOs[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cam_indices), cam_indices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3* sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);*/
    glBindVertexArray(mVAOs[0]);
    body_ = genBodyVertices();
    glBindBuffer(GL_ARRAY_BUFFER, mVBOs[0]);
    glBufferData(GL_ARRAY_BUFFER, body_.size() * 3 * sizeof(float), body_.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3* sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    //world grid
    glBindVertexArray(mVAOs[1]);
    grid_ = genGridVertices(50, 1);
    glBindBuffer(GL_ARRAY_BUFFER, mVBOs[1]);
    glBufferData(GL_ARRAY_BUFFER, grid_.size() * sizeof(float), grid_.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3* sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
    //coarser world grid for visual assist
    glBindVertexArray(mVAOs[2]);
    assist_grid_ = genGridVertices(50, 5);
    glBindBuffer(GL_ARRAY_BUFFER, mVBOs[6]);
    glBufferData(GL_ARRAY_BUFFER, assist_grid_.size() * sizeof(float), assist_grid_.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3* sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void MapDrawer::draw() {
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(1.f, 1.f, 1.f, 1.0f);
    glUniformMatrix4fv(mVPMtrx, 1, false, vp_matrix_);

    pthread_mutex_lock(&pose_mtx);
    Eigen::Matrix4f twb = pose_to_draw;
    Eigen::RowVector3f cam_color = pose_color;
    std::vector<std::array<float,3>> traj = traj_to_draw;
    Eigen::RowVector3f t_color = traj_color;
    std::vector<std::array<float,6>> map = map_points;
    std::vector<Eigen::Matrix4d> key_frames_to_draw = key_frames;
    std::vector<std::array<float,3>> mono_map = mono_map_points;
    pthread_mutex_unlock(&pose_mtx);

    glUseProgram(mProg);
    float color_black[3] {0, 0, 0};
    float color_green[3] {0, 1, 0};
    float color_blue[3] {0, 0, 1};
    float color_pink[3] {160.0/255, 32.0/255, 240.0/255};

    drawWorld();
    drawMapPoint(map);
    drawMonoMapPoint(mono_map, color_black);
    drawKeyFrames(key_frames_to_draw, color_pink);
    drawCurrentBody(twb.data(), color_blue);
    drawTrajectory(traj, color_green);

    // drawCurrentBody(cam_init_pose_, color_black);
    // drawTrajectory(traj_gt, color_pink);
}

void MapDrawer::drawCurrentBody(float Twb[], float color[]) {
    glBindVertexArray(mVAOs[0]);
    glLineWidth(4);
    glUniformMatrix4fv(mBodyMtrx, 1, false, Twb);
    glUniform1f(mPerVertexColor, 0);
    glUniform3fv(mColor, 1, color); // sets the uniform on the currently active shader program
    // glDrawElements(GL_LINES, 16, GL_UNSIGNED_INT, 0); //used for indices drawing, opposed to glDrawArrays()
    glDrawArrays(GL_LINES, 0, body_.size());
    glBindVertexArray(0);
}


void MapDrawer::drawWorld() {
    glBindVertexArray(mVAOs[1]);
    glLineWidth(4);
    //For stuff drawn in world coords, mBodyMtrx is always identity.
    glUniformMatrix4fv(mBodyMtrx, 1, false, mIdentity);
    glUniform1f(mPerVertexColor, 0);

    float grid_color[3] {0.5, 0.5, 0.5};
    glUniform3fv(mColor, 1, grid_color);
//    glDrawElements(GL_LINES, 4, GL_UNSIGNED_INT, 0);
    glDrawArrays(GL_LINES, 0, grid_.size()/3);
    glBindVertexArray(0);
    // draw assist grid
    glBindVertexArray(mVAOs[2]);
    glLineWidth(4);
    glUniformMatrix4fv(mBodyMtrx, 1, false, mIdentity);
    glUniform1f(mPerVertexColor, 0);
    float assist_grid_color[3] {0.8, 0.5, 0.2};
    glUniform3fv(mColor, 1, assist_grid_color);
    glDrawArrays(GL_LINES, 0, assist_grid_.size()/3);
    glBindVertexArray(0);

    //Draw world coord xyz axis.
    glLineWidth(6);
    float x_color[3] = {1.0, 0, 0};
    float x_axis[6] = {0, 0, 0, 0.5, 0, 0};
    glUniform3fv(mColor, 1, x_color);
    glBindBuffer(GL_ARRAY_BUFFER, mVBOs[2]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(x_axis), x_axis, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3* sizeof(float), 0);
    glEnableVertexAttribArray(0);
    glDrawArrays(GL_LINES, 0, 2);
    float y_color[3] = {0, 1, 0};
    float y_axis[6] = {0, 0, 0, 0, 0.5, 0};
    glUniform3fv(mColor, 1, y_color);
    glBufferData(GL_ARRAY_BUFFER, sizeof(y_axis), y_axis, GL_DYNAMIC_DRAW);
    glDrawArrays(GL_LINES, 0, 2);
    float z_color[3] = {0, 0, 1};
    float z_axis[6] = {0, 0, 0, 0, 0, 0.5};
    glUniform3fv(mColor, 1, z_color);
    glBufferData(GL_ARRAY_BUFFER, sizeof(z_axis), z_axis, GL_DYNAMIC_DRAW);
    glDrawArrays(GL_LINES, 0, 2);
}


void MapDrawer::drawMapPoint(std::vector<std::array<float,6>> & map_pts) {
    if (map_pts.empty())
        return;
    glUniformMatrix4fv(mBodyMtrx, 1, false, cam_init_pose_);

    glBindBuffer(GL_ARRAY_BUFFER, mVBOs[3]);
    glBufferData(GL_ARRAY_BUFFER, map_pts.size() * 6 * sizeof(float), map_pts.data(), GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6* sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6* sizeof(float),(void*)(3* sizeof(float)) );
    glEnableVertexAttribArray(1);

    glUniform1f(mPerVertexColor, 1);

    glDrawArrays(GL_POINTS, 0, map_pts.size());
}


void MapDrawer::drawTrajectory(std::vector<std::array<float,3>> &traj, float color[]) {
    glUniformMatrix4fv(mBodyMtrx, 1, false, mIdentity);

    glBindBuffer(GL_ARRAY_BUFFER, mVBOs[5]);
    glBufferData(GL_ARRAY_BUFFER, traj.size()*3* sizeof(float), traj.data(), GL_DYNAMIC_DRAW);

    glUniform1f(mPerVertexColor, 0);
    glUniform3fv(mColor, 1, color);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3* sizeof(float), 0);
    glEnableVertexAttribArray(0);

    glDrawArrays(GL_POINTS, 0, traj.size());
}

void MapDrawer::drawKeyFrames(std::vector<Eigen::Matrix4d> & kfs, float color[]) {
    glBindVertexArray(mVAOs[0]);
    glLineWidth(4);
    for (auto & pose : kfs) {
        Eigen::Matrix4f twb = pose.cast<float>();
        glUniformMatrix4fv(mBodyMtrx, 1, false, twb.data());
        glUniform1f(mPerVertexColor, 0);
        glUniform3fv(mColor, 1, color); // sets the uniform on the currently active shader program
        // glDrawElements(GL_LINES, 16, GL_UNSIGNED_INT, 0); //used for indices drawing, opposed to glDrawArrays()
        glDrawArrays(GL_LINES, 0, body_.size());
    }
    glBindVertexArray(0);
}

void MapDrawer::drawMonoMapPoint(std::vector<std::array<float,3>> &map, float color[]) {
    if (map.empty())
        return;
    glUniformMatrix4fv(mBodyMtrx, 1, false, mIdentity);

    glBindBuffer(GL_ARRAY_BUFFER, mVBOs[4]);
    glBufferData(GL_ARRAY_BUFFER, map.size()*3* sizeof(float), map.data(), GL_DYNAMIC_DRAW);

    glUniform1f(mPerVertexColor, 0);
    glUniform3fv(mColor, 1, color);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3* sizeof(float), 0);
    glEnableVertexAttribArray(0);

    glDrawArrays(GL_POINTS, 0, map.size());
}
