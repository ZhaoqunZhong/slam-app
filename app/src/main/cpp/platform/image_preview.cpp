#include <opencv2/imgproc.hpp>
#include <dirent.h>
#include <opencv2/imgcodecs.hpp>
#include "image_preview.h"
#include "native_debug.h"
#include "perf_monitor.h"
#include "glog/logging.h"
#include <thread>

ANativeWindow* preview_native_window = nullptr;
int preview_w, preview_h;
bool rgb_single_shot = false;

void takeRgbSingleShot(rgb_msg &rgb) {
    if (!rgb_single_shot)
        return;

    std::string path = "/sdcard/orbbec-vio-data/rgb_single_shots/";
    DIR *dir = opendir(path.c_str());
    if (dir) {

    } else {
        std::string cmd = "mkdir -p ";
        system((cmd + path).c_str());
    }

    cv::imwrite(path + std::to_string(rgb.ts) + ".png", rgb.yMat);

    rgb_single_shot = false;
}

void* ipThreadRunner(void *ptr) {
    ImagePreviewer* classptr = (ImagePreviewer*)ptr;
    classptr->mainThreadFunction();
    return nullptr;
}

void ImagePreviewer::mainThreadFunction() {
    while (thread_run_) {
        pthread_mutex_lock(&preview_mtx);
        pthread_cond_wait(&preview_cond, &preview_mtx);
        cv::Mat mat_to_show = preview_mat;
        bool rot = rotate;
        pthread_mutex_unlock(&preview_mtx);

        if (mat_to_show.empty())
            continue;

        cv::Mat rotated_frame;
        if (rot) {
            cv::rotate(mat_to_show, rotated_frame, cv::ROTATE_90_CLOCKWISE);
        } else {
            rotated_frame = mat_to_show;
        }
        cv::Mat mat_rgba;
        if (rotated_frame.channels() == 1) {
            cvtColor(rotated_frame, mat_rgba, cv::COLOR_GRAY2RGBA);
        } else if (rotated_frame.channels() == 3) {
//            cvtColor(rotated_frame, mat_rgba, cv::COLOR_RGB2RGBA);
            cvtColor(rotated_frame, mat_rgba, cv::COLOR_BGR2RGBA); //Opencv uses BGR by default.
        }

        while (preview_native_window == nullptr) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ANativeWindow_Buffer buf;
        if (ANativeWindow_lock(preview_native_window, &buf, nullptr) < 0) {
            LOG(WARNING) << "ANativeWindow_lock preview_native_window failed.";
            continue;
        } else if (buf.bits == nullptr) {
            LOG(WARNING) << "Image preview buffer is empty.";
            ANativeWindow_unlockAndPost(preview_native_window);
            continue;
        } else if (buf.width * buf.height != preview_w * preview_h) {
            LOG(WARNING) << "Image preview buffer's size is not equal to camera stream's.";
            ANativeWindow_unlockAndPost(preview_native_window);
            continue;
        }
        // LOG(INFO) << "ANativeWindow_lock buf.bits " << buf.bits << " ";
        // LOG_FIRST_N(INFO,1) << "buf width " << buf.width << " buf height " << buf.height;

        uint32_t *out = static_cast<uint32_t *>(buf.bits);
        for (int32_t y = 0; y < mat_rgba.rows; y++) {
            for (int32_t x = 0; x < mat_rgba.cols; x++) {
                out[x] = mat_rgba.at<uint32_t>(y, x);
            }
            out += buf.stride;
        }
        ANativeWindow_unlockAndPost(preview_native_window);
    }
}

void ImagePreviewer::start() {
    thread_run_ = true;
    pthread_create(&main_th_, nullptr, ipThreadRunner, this);
}

void ImagePreviewer::stop() {
    thread_run_ = false;
    pthread_cond_signal(&preview_cond);
    pthread_join(main_th_, nullptr);
    ANativeWindow_release(preview_native_window);
}


