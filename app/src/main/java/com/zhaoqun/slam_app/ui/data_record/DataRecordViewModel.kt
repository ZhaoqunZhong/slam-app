package com.zhaoqun.slam_app.ui.data_record

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import kotlinx.serialization.Serializable

class DataRecordViewModel : ViewModel() {
    val _record_on = MutableLiveData<Boolean>().apply { value = false }

    val _folder_name = MutableLiveData<String>().apply { value = "" }

    val _post_with_time = MutableLiveData<Boolean>().apply { value = false }

    val _record_cam = MutableLiveData<Boolean>().apply { value = true }

    val _save_imgs = MutableLiveData<Boolean>().apply { value = false }

    val _cam_res = MutableLiveData<Int>().apply { value = 0 } // 0: 640*480, 1: 1280*720, 2: 1920*1080

    val _ts_format = MutableLiveData<Int>().apply { value = 0 } // 0: .csv, 1: .txt

    val _record_imu = MutableLiveData<Boolean>().apply { value = true }

    val _sync_acc_gyr = MutableLiveData<Boolean>().apply { value = true }

    val _imu_format = MutableLiveData<Int>().apply { value = 0 } // 0: .csv, 1: .txt

    val _imu_order = MutableLiveData<Int>().apply { value = 0 } // 0: acc in front, 1: gyr in front

    val _pack_rosbag = MutableLiveData<Boolean>().apply { value = true }

    val _enable_60hz = MutableLiveData<Boolean>().apply { value = false }

    val _cam_freq = MutableLiveData<String>()

    val _acc_freq = MutableLiveData<String>()

    val _gyr_freq = MutableLiveData<String>()

    val _imu_freq = MutableLiveData<String>()

    val _mag_freq = MutableLiveData<String>()

    val _data_size = MutableLiveData<String>()
}

@Serializable
data class RecordConfig (
    val folder_name: String,
    val post_with_time: Boolean,
    val time_postfix: String,
    val record_camera: Boolean,
    val save_image: Boolean,
    val camera_id: String,
    val camera_resolution: Int,
    val enable60hz: Boolean,
    val image_ts_file_type: String,
    val record_imu: Boolean,
    val sync_acc_gyr: Boolean,
    val imu_freq: Int,
    val imu_file_type: String,
    val acc_gyr_order: Int,
    val pack_rosbag: Boolean
)