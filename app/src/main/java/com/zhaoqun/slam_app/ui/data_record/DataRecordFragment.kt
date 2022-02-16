package com.zhaoqun.slam_app.ui.data_record

import android.R
import android.R.attr
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.view.inputmethod.EditorInfo
import android.widget.TextView
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import androidx.lifecycle.Observer
import androidx.lifecycle.ViewModelProvider
import com.zhaoqun.slam_app.databinding.FragmentDataBinding
import com.zhaoqun.slam_app.ui.image_processing.GalleryViewModel
import android.R.attr.data
import android.content.Context
import android.util.Log

import android.widget.ArrayAdapter
import com.zhaoqun.slam_app.file_server.FileSynchronizer
import kotlinx.serialization.Serializable
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import java.io.File


class DataRecordFragment : Fragment() {
    private val dataRecordViewModel: DataRecordViewModel by activityViewModels()
    private var _binding: FragmentDataBinding? = null

    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!
    private val debug_tag = "DataRecordFragment"

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        _binding = FragmentDataBinding.inflate(inflater, container, false)
        val root: View = binding.root

        binding.recordButton.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) startDumpJNI() else stopDumpJNI()
        }

        prepareDefaultOptions()

        return root
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    fun prepareDefaultOptions () {
        dataRecordViewModel._folder_name.observe(viewLifecycleOwner, Observer {
            binding.folderName.setText(it)
        })
        binding.folderName.setOnEditorActionListener{
                v, actionId, event ->
            return@setOnEditorActionListener when (actionId) {
                EditorInfo.IME_ACTION_DONE -> {
                    dataRecordViewModel._folder_name.value = v.text.toString()
                    true
                }
                else -> false
            }
        }
        dataRecordViewModel._post_with_time.observe(viewLifecycleOwner, {binding.postTime.isChecked = it })
        dataRecordViewModel._record_cam.observe(viewLifecycleOwner, {binding.recordCam.isChecked = it})
        dataRecordViewModel._save_imgs.observe(viewLifecycleOwner, {binding.saveImgs.isChecked = it})
        dataRecordViewModel._cam_res.observe(viewLifecycleOwner, {binding.camResolution.setSelection(it)})
        dataRecordViewModel._ts_format.observe(viewLifecycleOwner, {binding.tsFormat.setSelection(it)})
        dataRecordViewModel._record_imu.observe(viewLifecycleOwner, {binding.recordImu.isChecked = it})
        dataRecordViewModel._sync_acc_gyr.observe(viewLifecycleOwner, {binding.syncAccGyr.isChecked = it})
        dataRecordViewModel._imu_format.observe(viewLifecycleOwner, {binding.imuFile.setSelection(it)})
        dataRecordViewModel._imu_order.observe(viewLifecycleOwner, {binding.imuOrder.setSelection(it)})
        dataRecordViewModel._pack_rosbag.observe(viewLifecycleOwner, {binding.packRosbag.isChecked = it})
        dataRecordViewModel._enable_60hz.observe(viewLifecycleOwner, {binding.enable60hz.isChecked = it})

        var cam_ids = arrayListOf<String>("-1")
        var imu_freqs = arrayListOf<String>("0")
        @Serializable
        data class ViSensorInfo(
            val cam_ids: ArrayList<String>,
            val imu_freqs: ArrayList<String>
        )
        val json = Json {
            prettyPrint = true
            prettyPrintIndent = "  "
            coerceInputValues = true
        }
        val app_dir = activity?.filesDir.toString()
        val sensor_info_file = File("$app_dir/vi_sensor_info.json")
        if (sensor_info_file.exists()) {
            Log.i(debug_tag, "Found vi sensor info file, read from file.")
            val sensor_info_string = sensor_info_file.readText()
            val sensor_info_data = json.decodeFromString<ViSensorInfo>(sensor_info_string)
            cam_ids = sensor_info_data.cam_ids
            imu_freqs = sensor_info_data.imu_freqs
        } else {
            cam_ids = getBackCamIDs().toMutableList() as ArrayList<String>
            imu_freqs = getImuFreqs().toMutableList() as ArrayList<String>
            var sensor_info_data = ViSensorInfo(cam_ids, imu_freqs)
            val sensor_info_string = json.encodeToString(sensor_info_data)
            sensor_info_file.writeText(sensor_info_string)
        }

        val aa = ArrayAdapter(requireContext(), android.R.layout.simple_spinner_item, cam_ids)
        aa.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        binding.camId.setAdapter(aa)

        val aa1 = ArrayAdapter(requireContext(), android.R.layout.simple_spinner_item, imu_freqs)
        aa1.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        binding.imuFps.setAdapter(aa1)
    }

    external fun getBackCamIDs() : Array<String>
    external fun getImuFreqs() : Array<String>
    // Kotlin alternatives
//    fun getBackCamIDs() : Array<String> { }
//    fun getImuFreqs() : Array<String> { }

    external fun startDumpJNI()
    external fun stopDumpJNI()

    companion object {
        // Used to load the 'native-lib' library on application startup.
        init {
            System.loadLibrary("data_record")
        }
    }
}