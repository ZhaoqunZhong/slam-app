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

#ifndef PERFMONITOR_H_
#define PERFMONITOR_H_

#include <errno.h>
#include <time.h>


/******************************************************************
 * Helper class for a performance monitoring and get current tick time
 */
namespace Initializer {

    class PerfMonitor {
    public:
        PerfMonitor();

        virtual ~PerfMonitor();

        void update();

        inline int getFPS() { return static_cast<int>(current_FPS_); }

        void reset();

    private:
        static const int kNumSamples = 30;
        float current_FPS_;
        time_t tv_last_sec_;
        double last_tick_;
        int32_t tickindex_;
        double ticksum_;
        double ticklist_[kNumSamples];

        double UpdateTick(double current_tick);

        int count_sample_ = 0;
    };


    class TimeLagMeasurer {
    public:
        TimeLagMeasurer(int clock_macro = CLOCK_BOOTTIME);

        ~TimeLagMeasurer() {};

        double lagFromStartSecond();

        double lagFromLastSecond();

        double getCurrentTimeSecond();

        void restart();

    private:
        int clock_;
        struct timespec ts_start_;
        double ts_start_s_;
        struct timespec ts_cur_;
        double ts_cur_s_;
        double ts_last_s_;
    };
}

#endif /* PERFMONITOR_H_ */
