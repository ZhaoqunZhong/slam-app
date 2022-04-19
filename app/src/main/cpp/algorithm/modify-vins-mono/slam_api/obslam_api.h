#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** define the slam handle */
#define SLAM_HANDLE void*

void create_slam(SLAM_HANDLE *slam_handle, const char *volcabulary_path, const char *config_path);

void release_slam(SLAM_HANDLE slam_handle);

int32_t start_slam(SLAM_HANDLE slam_handle);

int32_t stop_slam(SLAM_HANDLE slam_handle);


typedef enum {
    INITIALIZING = 0,
    TRACKING,
} slam_status;
typedef void (*slam_status_callback) (slam_status *status);
void slam_register_status_callback(SLAM_HANDLE slam_handle, slam_status_callback cbkfun);


typedef enum {
    PRE_ACCURATE = 0,
    ACCURATE
} slam_pose_type;
/*
 * Pose is T_world_to_imu
 */
struct slam_pose {
    uint64_t ts;
    float tran[3];
    float rot[4]; //xyzw as in Eigen quaternion storage
    float T[16]; //column major
    slam_pose_type type;
};
typedef void (*slam_pose_callback)(slam_pose* pose);
void slam_register_pose_callback(SLAM_HANDLE slam_handle, slam_pose_callback cbkfun);



int32_t slam_add_image(SLAM_HANDLE slam_handle, uint64_t timestamp,
                       uint32_t width, uint32_t height, uint8_t *data);

int32_t slam_add_imu(SLAM_HANDLE slam_handle, uint64_t timestamp,
                     double linear_acceleration_x, double linear_acceleration_y, double linear_acceleration_z,
                     double angular_velocity_x, double angular_velocity_y, double angular_velocity_z);

int32_t slam_add_accel(SLAM_HANDLE slam_handle, uint64_t timestamp,
                       double linear_acceleration_x, double linear_acceleration_y, double linear_acceleration_z);

int32_t slam_add_gyro(SLAM_HANDLE slam_handle, uint64_t timestamp,
                      double angular_velocity_x, double angular_velocity_y, double angular_velocity_z);


/*-------------------private API--------------------*/
/*
 * The image process callback is non-blocking function. The client side needs to
 * check the timestamp disorder by themselves. Although the possibility of disorder
 * is pretty small.
 *
 * The image format is 3-channel rgb by default, since it's overlaid with processing result.
 */
struct slam_processed_image {
    uint64_t ts;
    uint32_t image_width;
    uint32_t image_height;
    uint8_t *image_data;
    float processing_time;
};
typedef void (*image_process_callback)(slam_processed_image *image);
void slam_register_image_process_callback(SLAM_HANDLE slam_handle, image_process_callback cbkfun);


/*
 * Inorder to support this keyFrame interface, the slam module has to maintain a number of image buffers,
 * instead of ditching the images as soon as processing is done.
 *
 * This interface is also non-blocking.
 *
 * This interface will return a frame when it's first categorized as keyFrame,
 * even if it would be culled later.
 *
 * The image format is also 3-channel rgb by default, since it's overlaid with processing result.
 */
struct slam_keyFrame {
    uint64_t ts;
    uint32_t image_width;
    uint32_t image_height;
    uint8_t *image_data;
};
typedef void (*keyFame_callback)(slam_keyFrame *frame);
void slam_register_keyFrame_callback(SLAM_HANDLE slam_handle, keyFame_callback cbkfun);


#ifdef __cplusplus
}
#endif
