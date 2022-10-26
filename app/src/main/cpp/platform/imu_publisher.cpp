#include <string>
#include <unistd.h>
#include "imu_publisher.h"
#include <pthread.h>
#include <vector>
#include <fstream>
#include <dirent.h>
#include <android/hardware_buffer.h>
#include "glog/logging.h"

const int LOOPER_ID_USER = 3;

/// Assemble imu finite automata
enum Input {acc,gyr} cur_input;
enum State {WAIT_FOR_MSG, ACC, ACC_GYR, ACC_GYR_ACC, ACC_GYRs, ACC_GYR_ACCs} cur_state;

bool checkAccValue(acc_msg & accMsg) {
	static acc_msg last_acc;
	static bool init = true;
	if (init) {
		init = false;
		last_acc = accMsg;
		return true;
	} else {
		if (fabs(accMsg.ax - last_acc.ax) > 8 || fabs(accMsg.ay - last_acc.ay) > 8 || fabs(accMsg.az - last_acc.az) > 8)
			return false;
		else {
			last_acc = accMsg;
			return true;
		}
	}
}
bool checkGyrValue(gyr_msg & gyroMsg) {
	static gyr_msg last_gyro;
	static bool init = true;
	if (init) {
		init = false;
		last_gyro = gyroMsg;
		return true;
	} else {
		if (fabs(gyroMsg.rx - last_gyro.rx) > 2 || fabs(gyroMsg.ry - last_gyro.ry) > 2 || fabs(gyroMsg.rz - last_gyro.rz) > 2)
			return false;
		else {
			last_gyro = gyroMsg;
			return true;
		}
	}
}

void* looperThreadWrapper(void *ptr) {
    ImuPublisher* classptr = (ImuPublisher*)ptr;
    classptr->run();
    return nullptr;
}
void* accThreadWrapper(void *ptr) {
    ImuPublisher* classptr = (ImuPublisher*)ptr;
    classptr->runAcc();
    return nullptr;
}
void* gyroThreadWrapper(void *ptr) {
    ImuPublisher* classptr = (ImuPublisher*)ptr;
    classptr->runGyro();
    return nullptr;
}
void* magThreadWrapper(void *ptr) {
    ImuPublisher* classptr = (ImuPublisher*)ptr;
    classptr->runMagnetic();
    return nullptr;
}

void ImuPublisher::init() {
    if (use_direct_channel_) {
        AHardwareBuffer_Desc bufferDesc {
                .width = 104,
                .height = 1,
                .layers = 1,
                .format = AHARDWAREBUFFER_FORMAT_BLOB,
                .usage = AHARDWAREBUFFER_USAGE_SENSOR_DIRECT_DATA
        };

        int acc_hb_allo = AHardwareBuffer_allocate(&bufferDesc, &acc_buffer_);
        ASSERT(acc_hb_allo == 0, "acc allocate hardware buffer failed.");
        acc_channel_ = ASensorManager_createHardwareBufferDirectChannel(sensorManager_, acc_buffer_, 104);
        ASSERT(acc_channel_ > 0, "acc create direct channel failed.");
        int acc_cf_dr = ASensorManager_configureDirectReport(sensorManager_, accelerometer_, acc_channel_, direct_report_level_);
        ASSERT(acc_cf_dr > 0, "acc config direct report failed.");
        int gyro_hb_allo = AHardwareBuffer_allocate(&bufferDesc, &gyro_buffer_);
        ASSERT(gyro_hb_allo == 0, "gyro allocate hardware buffer failed.");
        gyro_channel_ = ASensorManager_createHardwareBufferDirectChannel(sensorManager_, gyro_buffer_, 104);
        ASSERT(gyro_channel_ > 0, "gyro create direct channel failed.");
        int gyro_cf_dr = ASensorManager_configureDirectReport(sensorManager_, gyro_, gyro_channel_, direct_report_level_);
        ASSERT(gyro_cf_dr > 0, "gyro config direct report failed.");
        // Magnetic field
        int mag_hb_allo = AHardwareBuffer_allocate(&bufferDesc, &mag_buffer_);
        ASSERT(mag_hb_allo == 0, "magnetic allocate hardware buffer failed.");
        mag_channel_ = ASensorManager_createHardwareBufferDirectChannel(sensorManager_, mag_buffer_, 104);
        ASSERT(mag_channel_ > 0, "magnetic create direct channel failed.");
        int mag_cf_dr = ASensorManager_configureDirectReport(sensorManager_, magnetic_, mag_channel_, direct_report_level_);
        ASSERT(mag_cf_dr > 0, "magnetic config direct report failed.");

    } else {
/*        std::string file_name = "sdcard/orbbec-vio-data/config.yaml";
        cv::FileStorage fs(file_name, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            LOG(ERROR) << "cv::FileStorage open file failed -> " << file_name;
            return;
        }
        int fps_req = static_cast<int>(fs["imu_fps"]);
        fs.release();*/

        int32_t SENSOR_REFRESH_PERIOD_US;
        SENSOR_REFRESH_PERIOD_US = int32_t(1e5 / imu_fps_);
        looper_ = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
        sensorEventQueue_ = ASensorManager_createEventQueue(sensorManager_, looper_,LOOPER_ID_USER, NULL, NULL);
        ASensorEventQueue_enableSensor(sensorEventQueue_,accelerometer_);
        ASensorEventQueue_setEventRate(sensorEventQueue_,accelerometer_,SENSOR_REFRESH_PERIOD_US);
        ASensorEventQueue_enableSensor(sensorEventQueue_,gyro_);
        ASensorEventQueue_setEventRate(sensorEventQueue_,gyro_,SENSOR_REFRESH_PERIOD_US);
        ASensorEventQueue_enableSensor(sensorEventQueue_,magnetic_);
        ASensorEventQueue_setEventRate(sensorEventQueue_,magnetic_,SENSOR_REFRESH_PERIOD_US);
    }

    cur_state = WAIT_FOR_MSG;
}

void ImuPublisher::start(int imu_freq, bool sync_acc_gyr) {
    if (started_) {
        LOG(WARNING) << "ImuPublisher already started, can't call start() again!";
        return;
    }
    started_ = true;
    LOG(INFO) << "ImuPublisher started.";

    sync_acc_gyr_ = sync_acc_gyr;
    if (imu_freq == 0) {
        imu_fps_ = 200;
        use_direct_channel_ = true;
        direct_report_level_ = 2;
    } else if (imu_freq == 1)  {
        imu_fps_ = 400;
        use_direct_channel_ = false;
    } else {
        imu_fps_ = 800;
        use_direct_channel_ = true;
        direct_report_level_ = 3;
    }

    sensorManager_ = ASensorManager_getInstanceForPackage("com.zhaoqun.slam_app");
    accelerometer_ = ASensorManager_getDefaultSensor(sensorManager_, ASENSOR_TYPE_ACCELEROMETER_UNCALIBRATED);
    gyro_ = ASensorManager_getDefaultSensor(sensorManager_, ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED);
    magnetic_ = ASensorManager_getDefaultSensor(sensorManager_, ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED);
/*	ASensorList sensor_list = nullptr;
	int sensor_count = ASensorManager_getSensorList(sensorManager_, &sensor_list);
	LOGI("Found %d sensors.", sensor_count);
	for (int i = 0; i < sensor_count; i++) {
		LOGI("Supports sensor %s.", ASensor_getName(sensor_list[i]));
	}*/
    int acc_mode = ASensor_getReportingMode(accelerometer_);
    LOGI("acc reporting mode : %d", acc_mode); // 0: continuous
    bool acc_dc_hw = ASensor_isDirectChannelTypeSupported(accelerometer_, ASENSOR_DIRECT_CHANNEL_TYPE_HARDWARE_BUFFER);
    LOGI("acc direct channel hardware buffer %s", acc_dc_hw ? "supported" : "not supported");
    bool acc_dc_sm = ASensor_isDirectChannelTypeSupported(accelerometer_, ASENSOR_DIRECT_CHANNEL_TYPE_SHARED_MEMORY);
    LOGI("acc direct channel shared memory %s", acc_dc_sm ? "supported" : "not supported");
    if (!acc_dc_hw && !acc_dc_sm)
        use_direct_channel_ = false;

    imu_publish_on_ = true;
    if(use_direct_channel_) {
        init();
        pthread_create(&acc_thread_, nullptr, accThreadWrapper, this);
        pthread_create(&gyro_thread_, nullptr, gyroThreadWrapper, this);
        pthread_create(&mag_thread_, nullptr, magThreadWrapper, this);
    } else
        pthread_create(&looper_thread_, nullptr, looperThreadWrapper, this);
}

void ImuPublisher::stop() {
    if (!started_) {
        LOG(WARNING) << "ImuPublisher hasn't started yet, can't call stop().";
        return;
    }
    started_ = false;
    LOG(INFO) << "ImuPublisher stopped.";
	imu_publish_on_ = false;
    if(use_direct_channel_) {
        pthread_join(acc_thread_, nullptr);
        pthread_join(gyro_thread_, nullptr);
        pthread_join(mag_thread_, nullptr);
    } else
        pthread_join(looper_thread_, nullptr);

    acc_cache_.clear();
    gyro_cache_.clear();
}

void ImuPublisher::run() {
    // LOGI("imu publisher run() enter.");
    init(); //ALooper_prepare() should be in the same scope with ALooper_pollAll()

	std::vector<ASensorEvent> event_buffer;
	event_buffer.resize(0);
	while (imu_publish_on_) {
		// useconds_t thread_sleep_time = static_cast<useconds_t>(100); //0.1ms
		// usleep(thread_sleep_time);
		int ident = ALooper_pollAll(
				-1,
				NULL /* no output file descriptor */,
				NULL /* no output event */,
				NULL /* no output data */);
		if (ident != LOOPER_ID_USER) {
			continue;
		}

		ASensorEvent event;
		while (ASensorEventQueue_getEvents(sensorEventQueue_, &event, 1) > 0) {
			if (event.type != ASENSOR_TYPE_ACCELEROMETER_UNCALIBRATED && event.type != ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED
                && event.type != ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED)
				continue;
			if (event.type == ASENSOR_TYPE_ACCELEROMETER_UNCALIBRATED) {
				acc_msg accMsg;
				accMsg.ts = event.timestamp;
				if (accMsg.ts - last_acc_ts_ < 1e5) {//0.0001s
                    LOG(WARNING) << "acc timestamp error";
                    LOG(INFO) << "current acc ts " << accMsg.ts;
                    LOG(INFO) << "last acc ts " << last_acc_ts_;
                    continue;
                }
				last_acc_ts_ = accMsg.ts;
				accMsg.ax = event.acceleration.x;
				accMsg.ay = event.acceleration.y;
				accMsg.az = event.acceleration.z;
//				if (!checkAccValue(accMsg))
//					continue;
				acc_callback_(accMsg);
                if (sync_acc_gyr_) {
                    acc_cache_.push_back(accMsg);
                    cur_input = acc;
                    constructImuInterpolateAcc();
                }
			}
			else if (event.type == ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED) {
				gyr_msg gyroMsg;
				gyroMsg.ts = event.timestamp;
                if (gyroMsg.ts - last_gyro_ts_ < 1e5) {
                    LOG(WARNING) << "gyr timestamp error";
                    LOG(INFO) << "current gyr ts " << gyroMsg.ts;
                    LOG(INFO) << "last gyr ts " << last_gyro_ts_;
                    continue;
                }
				last_gyro_ts_ = gyroMsg.ts;
				gyroMsg.rx = event.uncalibrated_gyro.x_uncalib;
				gyroMsg.ry = event.uncalibrated_gyro.y_uncalib;
				gyroMsg.rz = event.uncalibrated_gyro.z_uncalib;
//				if (!checkGyrValue(gyroMsg))
//					continue;
				gyro_callback_(gyroMsg);
                if (sync_acc_gyr_) {
                    gyro_cache_.push_back(gyroMsg);
                    cur_input = gyr;
                    constructImuInterpolateAcc();
                }
			}
            else if (event.type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED) {
                mag_msg magMsg;
                magMsg.ts = event.timestamp;
                if (magMsg.ts - last_mag_ts_ < 1e5) {
                    LOG(WARNING) << "magnetic timestamp error";
                    LOG(INFO) << "current mag ts " << magMsg.ts;
                    LOG(INFO) << "last mag ts " << last_mag_ts_;
                    continue;
                }
                last_mag_ts_ = magMsg.ts;
                magMsg.x = event.uncalibrated_magnetic.x_uncalib;
                magMsg.y = event.uncalibrated_magnetic.y_uncalib;
                magMsg.z = event.uncalibrated_magnetic.z_uncalib;
                mag_callback_(magMsg);
            }
//			LOGI("acc gyr cache sizes %d, %d", acc_cache_.size(), gyro_cache_.size());
		}
	}

    ASensorEventQueue_disableSensor(sensorEventQueue_, accelerometer_);
    ASensorEventQueue_disableSensor(sensorEventQueue_, gyro_);
    ASensorEventQueue_disableSensor(sensorEventQueue_, magnetic_);
    ASensorManager_destroyEventQueue(sensorManager_, sensorEventQueue_);

}

/*        offset   type                    name
        ------------------------------------------------------------------------
        0x0000   int32_t                 size (always 104)
        0x0004   int32_t                 sensor report token
        0x0008   int32_t                 type (see SensorType)
        0x000C   uint32_t                atomic counter
        0x0010   int64_t                 timestamp (see Event)
        0x0018   float[16]/int64_t[8]    data (data type depends on sensor type)
        0x0058   int32_t[4]              reserved (set to zero)*/

void ImuPublisher::runAcc() {
    while (imu_publish_on_) {
        // The sleep here is because the sonsor reporting mode is continuous, not on_change.
        // Means you will get the same message multiple times until it is replaced by new message.
        useconds_t thread_sleep_time = static_cast<useconds_t>(100); //0.1ms
        usleep(thread_sleep_time);

        void *acc_mem;
        int acc_l_rs = AHardwareBuffer_lock(acc_buffer_, AHARDWAREBUFFER_USAGE_CPU_READ_MASK, -1, NULL, &acc_mem);
        // ASSERT(acc_l_rs == 0, "acc lock hardware buffer failed.");
        if (acc_l_rs != 0) {
            LOG(WARNING) << "Acc lock hardware buffer failed.";
            continue;
        }
//        LOGI("acc timestamp %f", *(static_cast<int64_t*>(acc_mem) + 2)/1e5);

        auto data = static_cast<float16_t*>(acc_mem) + 12;
        ASensorVector *acc_data = reinterpret_cast<ASensorVector*>(data);
//        LOGI("acc data %f, %f, %f", acc_data->x, acc_data->y, acc_data->z);
        acc_msg accMsg;
        accMsg.ts = *(static_cast<int64_t*>(acc_mem) + 2);
        if (accMsg.ts - last_acc_ts_ < 1e5) {// 0.1ms
            continue;
        }

        last_acc_ts_ = accMsg.ts;
        accMsg.ax = acc_data->x;
        accMsg.ay = acc_data->y;
        accMsg.az = acc_data->z;
/*        if (!checkAccValue(accMsg))
            continue;*/
        acc_callback_(accMsg);
        if (sync_acc_gyr_) {
            pthread_mutex_lock(&cache_mtx_);
            acc_cache_.push_back(accMsg);
            cur_input = acc;
            constructImuInterpolateAcc();
            pthread_mutex_unlock(&cache_mtx_);
        }
        int acc_ul_rs = AHardwareBuffer_unlock(acc_buffer_, NULL);
        // ASSERT(acc_ul_rs == 0, "acc unlock hardware buffer failed.");
        if (acc_ul_rs != 0)
            LOG(WARNING) << "Acc unlock hardware buffer failed.";

    }

    ASensorManager_destroyDirectChannel(sensorManager_, acc_channel_);
    AHardwareBuffer_release(acc_buffer_);
}


void ImuPublisher::runGyro() {
    while (imu_publish_on_) {
        // The sleep here is because the sonsor reporting mode is continuous, not on_change.
        // Means you will get the same message multiple times until it is replaced by new message.
        useconds_t thread_sleep_time = static_cast<useconds_t>(100); //0.1ms
        usleep(thread_sleep_time);

        void *gyro_mem;
        int gyro_l_rs = AHardwareBuffer_lock(gyro_buffer_, AHARDWAREBUFFER_USAGE_CPU_READ_MASK, -1, NULL, &gyro_mem);
        // ASSERT(gyro_l_rs == 0, "gyro lock hardware buffer failed.");
        if (gyro_l_rs != 0) {
            LOG(WARNING) << "Gyro lock hardware buffer failed.";
            continue;
        }
//        LOGI("gyro timestamp %f", *(static_cast<int64_t*>(gyro_mem) + 2)/1e5);

        auto data = static_cast<float16_t*>(gyro_mem) + 12;
        AUncalibratedEvent *gyro_data = reinterpret_cast<AUncalibratedEvent*>(data);
//        LOGI("gyro data %f, %f, %f", gyro_data->x_uncalib, gyro_data->y_uncalib, gyro_data->z_uncalib);
        gyr_msg gyroMsg;
        gyroMsg.ts = *(static_cast<int64_t*>(gyro_mem) + 2);
        if (gyroMsg.ts - last_gyro_ts_ < 1e5) {
            continue;
        }
        last_gyro_ts_ = gyroMsg.ts;
        gyroMsg.rx = gyro_data->x_uncalib;
        gyroMsg.ry = gyro_data->y_uncalib;
        gyroMsg.rz = gyro_data->z_uncalib;
/*        if (!checkGyrValue(gyroMsg))
            continue;*/
        gyro_callback_(gyroMsg);
        if (sync_acc_gyr_) {
            pthread_mutex_lock(&cache_mtx_);
            gyro_cache_.push_back(gyroMsg);
            cur_input = gyr;
            constructImuInterpolateAcc();
            pthread_mutex_unlock(&cache_mtx_);
        }
        int gyro_ul_rs = AHardwareBuffer_unlock(gyro_buffer_, NULL);
        // ASSERT(gyro_ul_rs == 0, "gyro unlock hardware buffer failed.");
        if (gyro_ul_rs != 0)
            LOG(WARNING) << "Gyro unlock hardware buffer failed.";
    }

    ASensorManager_destroyDirectChannel(sensorManager_, gyro_channel_);
    AHardwareBuffer_release(gyro_buffer_);
}
void ImuPublisher::runMagnetic() {
    while (imu_publish_on_) {
        // The sleep here is because the sonsor reporting mode is continuous, not on_change.
        // Means you will get the same message multiple times until it is replaced by new message.
        useconds_t thread_sleep_time = static_cast<useconds_t>(100); //0.1ms
        usleep(thread_sleep_time);

        void *mag_mem;
        int mag_l_rs = AHardwareBuffer_lock(mag_buffer_, AHARDWAREBUFFER_USAGE_CPU_READ_MASK, -1, NULL, &mag_mem);
        // ASSERT(mag_l_rs == 0, "mag lock hardware buffer failed.");
        if (mag_l_rs != 0) {
            LOG(WARNING) << "mag lock hardware buffer failed.";
            continue;
        }
//        LOGI("mag timestamp %f", *(static_cast<int64_t*>(mag_mem) + 2)/1e5);

        auto data = static_cast<float16_t*>(mag_mem) + 12;
        AUncalibratedEvent *mag_data = reinterpret_cast<AUncalibratedEvent*>(data);
//        LOGI("mag data %f, %f, %f", mag_data->x_uncalib, mag_data->y_uncalib, mag_data->z_uncalib);
        mag_msg magMsg;
        magMsg.ts = *(static_cast<int64_t*>(mag_mem) + 2);
        if (magMsg.ts - last_mag_ts_ < 1e5) {
            continue;
        }
        last_mag_ts_ = magMsg.ts;
        magMsg.x = mag_data->x_uncalib;
        magMsg.y = mag_data->y_uncalib;
        magMsg.z = mag_data->z_uncalib;
        mag_callback_(magMsg);

        int mag_ul_rs = AHardwareBuffer_unlock(mag_buffer_, NULL);
        // ASSERT(mag_ul_rs == 0, "mag unlock hardware buffer failed.");
        if (mag_ul_rs != 0)
            LOG(WARNING) << "mag unlock hardware buffer failed.";
    }

    ASensorManager_destroyDirectChannel(sensorManager_, mag_channel_);
    AHardwareBuffer_release(mag_buffer_);
}

void ImuPublisher::constructImuInterpolateAcc() {
    switch (cur_state) {
        case WAIT_FOR_MSG:
            if (cur_input == acc) {
                cur_state = ACC;
            } else {
                gyro_cache_.clear();
            }
            break;
        case ACC:
            if (cur_input == acc) {
                acc_cache_.erase(acc_cache_.begin());
            } else {
                if (gyro_cache_.front().ts <= acc_cache_.front().ts) {
                    gyro_cache_.clear();
                } else {
                    cur_state = ACC_GYR;
                }
            }
            break;
        case ACC_GYR:
            if (cur_input == acc) {
                if (acc_cache_.back().ts <= gyro_cache_.front().ts) {
                    acc_cache_.erase(acc_cache_.begin());
                } else {
                    imu_msg imu;
                    imu.ts = gyro_cache_.front().ts;
                    imu.gyro_part = gyro_cache_.front();
                    double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                    imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                    acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                    acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                    imu_callback_(imu);
                    cur_state = ACC_GYR_ACC;
                }
            } else {
                cur_state = ACC_GYRs;
            }
            break;
        case ACC_GYRs:
            if (cur_input == acc) {
                if (acc_cache_.back().ts <= gyro_cache_.front().ts) {
                    acc_cache_.erase(acc_cache_.begin());
                } else if (acc_cache_.back().ts > gyro_cache_.front().ts && acc_cache_.back().ts < gyro_cache_.back().ts) {
                    int i;
                    for (i = 0; i < gyro_cache_.size(); i++) {
                        if (gyro_cache_[i].ts < acc_cache_.back().ts) {
                            imu_msg imu;
                            imu.ts = gyro_cache_[i].ts;
                            imu.gyro_part = gyro_cache_[i];
                            double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                            imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                            acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                            acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                            imu_callback_(imu);
                        } else {
                            break;
                        }
                    }
                    acc_cache_.erase(acc_cache_.begin());
                    gyro_cache_.erase(gyro_cache_.begin(), gyro_cache_.begin() + i);
                } else {
                    for (gyr_msg & gyro : gyro_cache_) {
                        imu_msg imu;
                        imu.ts = gyro.ts;
                        imu.gyro_part = gyro;
                        double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                        imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                        acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                        acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                        imu_callback_(imu);
                    }
                    gyro_cache_.erase(gyro_cache_.begin(), gyro_cache_.begin() + gyro_cache_.size() - 1);
                    cur_state = ACC_GYR_ACC;
                }
            } else {}
            break;
        case ACC_GYR_ACC:
            if (cur_input == acc) {
                cur_state = ACC_GYR_ACCs;
            } else {
                if (gyro_cache_.back().ts > acc_cache_.back().ts) {
                    acc_cache_.erase(acc_cache_.begin());
                    gyro_cache_.erase(gyro_cache_.begin());
                    cur_state = ACC_GYR;
                } else {
                    imu_msg imu;
                    imu.ts = gyro_cache_.back().ts;
                    imu.gyro_part = gyro_cache_.back();
                    double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                    imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                    acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                    acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                    imu_callback_(imu);
                    gyro_cache_.erase(gyro_cache_.begin());
                }
            }
            break;
        case ACC_GYR_ACCs:
            if (cur_input == acc) {

            } else {
                if (gyro_cache_.back().ts <= acc_cache_[1].ts) {
                    imu_msg imu;
                    imu.ts = gyro_cache_.back().ts;
                    imu.gyro_part = gyro_cache_.back();
                    double factor = (imu.ts - acc_cache_.front().ts)/(acc_cache_.back().ts - acc_cache_.front().ts);
                    imu.acc_part = {imu.ts, acc_cache_.front().ax * (1-factor) + acc_cache_.back().ax *factor,
                                    acc_cache_.front().ay * (1-factor) + acc_cache_.back().ay *factor,
                                    acc_cache_.front().az * (1-factor) + acc_cache_.back().az *factor};
                    imu_callback_(imu);
                    gyro_cache_.erase(gyro_cache_.begin());
                } else if (gyro_cache_.back().ts >= acc_cache_.back().ts) {
                    gyro_cache_.erase(gyro_cache_.begin());
                    acc_cache_.erase(acc_cache_.begin(), acc_cache_.begin() + acc_cache_.size() - 1);
                    cur_state = ACC_GYR;
                } else {
                    int i;
                    for (i = 0; i < acc_cache_.size(); i++) {
                        if (acc_cache_[i].ts >= gyro_cache_.back().ts)
                            break;
                    }
                    imu_msg imu;
                    imu.ts = gyro_cache_.back().ts;
                    imu.gyro_part = gyro_cache_.back();
                    double factor = (imu.ts - acc_cache_[i-1].ts)/(acc_cache_[i].ts - acc_cache_[i-1].ts);
                    imu.acc_part = {imu.ts, acc_cache_[i-1].ax * (1-factor) + acc_cache_[i].ax *factor,
                                    acc_cache_[i-1].ay * (1-factor) + acc_cache_[i].ay *factor,
                                    acc_cache_[i-1].az * (1-factor) + acc_cache_[i].az *factor};
                    imu_callback_(imu);
                    gyro_cache_.erase(gyro_cache_.begin());
                    acc_cache_.erase(acc_cache_.begin(), acc_cache_.begin() + i-1);
                }
            }
            break;
    }
}

std::vector<std::string> ImuPublisher::getAvailableImuFreqs() {
    std::vector<std::string> imu_freqs{"200", "400"};
    ASensorManager *sensor_manager = ASensorManager_getInstanceForPackage("com.zhaoqun.slam_app");
    const ASensor *acc = ASensorManager_getDefaultSensor(sensor_manager, ASENSOR_TYPE_ACCELEROMETER_UNCALIBRATED);
    int report_level = ASensor_getHighestDirectReportRateLevel(acc);
    if (report_level == ASENSOR_DIRECT_RATE_VERY_FAST)
        imu_freqs.emplace_back("800");
    return imu_freqs;
}






