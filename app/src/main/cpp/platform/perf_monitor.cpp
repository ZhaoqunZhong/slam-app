/*
 * Copyright 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "perf_monitor.h"

PerfMonitor::PerfMonitor()
        : current_FPS_(0),
          tv_last_sec_(0),
          last_tick_(0.f),
          tickindex_(0),
          ticksum_(0) {
    for (int32_t i = 0; i < kNumSamples; ++i) ticklist_[i] = 0;
}

PerfMonitor::~PerfMonitor() {}

double PerfMonitor::UpdateTick(double currentTick) {
    ticksum_ -= ticklist_[tickindex_];
    ticksum_ += currentTick;
    ticklist_[tickindex_] = currentTick;
    tickindex_ = (tickindex_ + 1) % kNumSamples;

    return ((double) ticksum_ / kNumSamples);
}

void PerfMonitor::update() {
    count_sample_++;
    struct timeval Time;
    gettimeofday(&Time, NULL);

    double time = Time.tv_sec + Time.tv_usec * 1.0 / 1000000.0;
    double tick = time - last_tick_;
    double d = UpdateTick(tick);
    last_tick_ = time;

    if (count_sample_ < kNumSamples)
        return;
    if (Time.tv_sec - tv_last_sec_ >= 0.5) { //refresh fps value every 0.5s
        current_FPS_ = 1.0 / d;
        tv_last_sec_ = Time.tv_sec;
    }
}

void PerfMonitor::reset() {
    current_FPS_ = 0;
    tv_last_sec_ = 0;
    last_tick_ = .0;
    tickindex_ = 0;
    ticksum_ = 0;
    for (int32_t i = 0; i < kNumSamples; ++i) ticklist_[i] = 0;
}

/*
 * ---------------------------------------------------------------
 * */

TimeLagMeasurer::TimeLagMeasurer(int clock_macro) {
    clock_ = clock_macro;
    clock_gettime(clock_, &ts_start_);
    ts_start_s_ = ts_start_.tv_sec + ts_start_.tv_nsec / 1e9;
    ts_last_s_ = ts_start_s_;
}

double TimeLagMeasurer::lagFromStartSecond() {
    clock_gettime(clock_, &ts_cur_);
    ts_cur_s_ = ts_cur_.tv_sec + ts_cur_.tv_nsec / 1e9;
    double lag = ts_cur_s_ - ts_start_s_;

    ts_last_s_ = ts_cur_s_;
    return lag;
}

double TimeLagMeasurer::lagFromLastSecond() {
    clock_gettime(clock_, &ts_cur_);
    ts_cur_s_ = ts_cur_.tv_sec + ts_cur_.tv_nsec / 1e9;
    double lag = ts_cur_s_ - ts_last_s_;

    ts_last_s_ = ts_cur_s_;
    return lag;
}

double TimeLagMeasurer::getCurrentTimeSecond() {
    clock_gettime(clock_, &ts_cur_);
    return ts_cur_.tv_sec + ts_cur_.tv_nsec/1e9;
}

void TimeLagMeasurer::restart() {
    clock_gettime(clock_, &ts_start_);
    ts_start_s_ = ts_start_.tv_sec + ts_start_.tv_nsec / 1e9;
    ts_last_s_ = ts_start_s_;
}