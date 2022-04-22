package com.zhaoqun.slam_app.ui.slam

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import kotlinx.serialization.Serializable

class SlamViewModel : ViewModel() {
    val _cam_res = MutableLiveData<Int>().apply { value = 0 } // 0: 640*480, 1: 1280*720, 2: 1920*1080
    val _enable_60hz = MutableLiveData<Boolean>().apply { value = false }
    val _cam_stream_freq = MutableLiveData<String>()
    val _imu_stream_freq = MutableLiveData<String>()
    val _pose_freq = MutableLiveData<String>()
    val _slam_boot_time = MutableLiveData<String>()
    val _slam_init_time = MutableLiveData<String>()
}

@Serializable
data class SlamDataConfig (
    val internal_folder_path: String,
    val camera_id: String,
    val camera_resolution: Int,
    val enable60hz: Boolean,
    val imu_freq: Int
)