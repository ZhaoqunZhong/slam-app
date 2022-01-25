
#ifndef VINSONANDROID_PREVIEWER_H
#define VINSONANDROID_PREVIEWER_H

#include <android/native_window.h>
#include "opencv2/core.hpp"
#include "cam_publisher.h"

extern bool rgb_single_shot;
extern void takeRgbSingleShot(rgb_msg &rgb);
extern ANativeWindow* preview_native_window;
extern int preview_w, preview_h;

class ImagePreviewer {
public:
    void start();
    void stop();
    pthread_t main_th_;
    void mainThreadFunction();
    bool thread_run_;
};

#endif //VINSONANDROID_PREVIEWER_H
