#ifndef VINSONANDROID_IMU_PUBLISHER_H
#define VINSONANDROID_IMU_PUBLISHER_H

#include <android/sensor.h>
#include <pthread.h>
#include <vector>
#include "perf_monitor.h"
#include "native_debug.h"

#define USE_DIRECT_REPORT
#define ASSEMBLE_IMU

struct acc_msg {
	uint64_t ts;
	double ax;
	double ay;
	double az;
};

struct gyr_msg {
	uint64_t ts;
	double rx;
	double ry;
	double rz;
};

struct imu_msg {
	uint64_t ts;
	acc_msg acc_part;
	gyr_msg gyro_part;
};


class ImuPublisher {
	ALooper *looper_;
	ASensorManager *sensorManager_;
	ASensorEventQueue *sensorEventQueue_;
	const ASensor *accelerometer_;
	const ASensor *gyro_;

	void init();

	void (*imu_callback_) (imu_msg &); //callback function from algorithm
	pthread_t looper_thread_, acc_thread_, gyro_thread_;
    bool imu_publish_on_= false;

	void (*acc_callback_) (acc_msg &);
	void (*gyro_callback_) (gyr_msg &);

    void constructImuInterpolateAcc();
	std::vector<gyr_msg> gyro_cache_;
	std::vector<acc_msg> acc_cache_;
	double last_acc_ts_, last_gyro_ts_;
	pthread_mutex_t cache_mtx_ = PTHREAD_MUTEX_INITIALIZER;

	AHardwareBuffer *acc_buffer_;
	int acc_channel_;
	AHardwareBuffer *gyro_buffer_;
	int gyro_channel_;

public:
	ImuPublisher(void (*fimu) (imu_msg &), void (*facc)(acc_msg &), void (*fgyro)(gyr_msg &)) {
		imu_callback_ = fimu;
		acc_callback_ = facc;
		gyro_callback_ = fgyro;
		LOGI("ImuPublisher constructed.");
	};
	~ImuPublisher(){
		LOGI("ImuPublisher destroyed."); //never called
	};
	void run(); //make public for wrapper
	void runAcc();
	void runGyro();

	//public interface
	void start();
	void stop();

	static std::vector<std::string> getAvailableImuFreqs();
};



#endif //VINSONANDROID_IMU_PUBLISHER_H
