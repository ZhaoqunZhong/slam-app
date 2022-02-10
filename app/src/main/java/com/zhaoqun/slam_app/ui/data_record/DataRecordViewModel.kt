package com.zhaoqun.slam_app.ui.data_record

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel

class DataRecordViewModel : ViewModel() {

    val _folder_name = MutableLiveData<String>()

    val _post_with_time = MutableLiveData<Boolean>().apply { value = true }

    val _record_cam = MutableLiveData<Boolean>().apply { value = true }

    val _save_imgs = MutableLiveData<Boolean>().apply { value = false }

    val _cam_res = MutableLiveData<Int>().apply { value = 0 } // 0: 640*480, 1: 1280*720, 2: 1920*1080

    val _ts_format = MutableLiveData<Int>().apply { value = 0 } // 0: .csv, 1: .txt

    val _record_imu = MutableLiveData<Boolean>().apply { value = true }

    val _sync_acc_gyr = MutableLiveData<Boolean>().apply { value = true }

    val _imu_format = MutableLiveData<Int>().apply { value = 0 } // 0: .csv, 1: .txt

    val _imu_order = MutableLiveData<Int>().apply { value = 0 } // 0: acc in front, 1: gyr in front

    val _pack_rosbag = MutableLiveData<Boolean>().apply { value = true }

}