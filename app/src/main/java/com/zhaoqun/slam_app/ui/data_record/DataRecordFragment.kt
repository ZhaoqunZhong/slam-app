package com.zhaoqun.slam_app.ui.data_record

import android.R
import android.R.attr
import android.os.Bundle
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
import android.icu.text.SimpleDateFormat
import android.os.Environment
import android.util.Log
import android.view.*
import android.widget.AdapterView

import android.widget.ArrayAdapter
import androidx.core.view.marginStart
import com.zhaoqun.slam_app.file_server.FileSynchronizer
import kotlinx.serialization.Serializable
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import java.io.File
import java.util.*
import kotlin.collections.ArrayList


class DataRecordFragment : Fragment() {
    private val dataRecordViewModel: DataRecordViewModel by activityViewModels()
    private var _binding: FragmentDataBinding? = null

    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!
    val json = Json {
        prettyPrint = true
        prettyPrintIndent = "  "
        coerceInputValues = true
    }
    private val debug_tag = "DataRecordFragment"

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        _binding = FragmentDataBinding.inflate(inflater, container, false)
        val root: View = binding.root

        prepareDefaultOptions()

        binding.camResolution.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(parent: AdapterView<*>?, view: View?, position: Int, id: Long) {
//                Log.i(debug_tag, "selected $position")
                val lp: ViewGroup.LayoutParams = binding.camPreview.layoutParams
                lp.width = binding.dividerVertical.marginStart
//                Log.i(debug_tag, "surfaceview size before ${lp.width} ${lp.height}")
                if (position == 0)
                    lp.height = (640.0 / 480 * lp.width).toInt()
                if (position == 1)
                    lp.height = (1280.0 / 720 * lp.width).toInt()
                if (position == 2)
                    lp.height = (1920.0 / 1080 * lp.width).toInt()
//                Log.i(debug_tag, "surfaceview size after ${lp.width} ${lp.height}")
                binding.camPreview.layoutParams = lp
            }
            override fun onNothingSelected(p0: AdapterView<*>?) { }
        }

        binding.camPreview.holder.addCallback(
            object : SurfaceHolder.Callback {
                override fun surfaceCreated(holder: SurfaceHolder) { }
                override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {
//                    Log.i(debug_tag, "cam preview surfacechanged, width $width, height $height")
                    sendSurfaceToJNI(holder.surface)
                }
                override fun surfaceDestroyed(holder: SurfaceHolder) { }
            }
        )

        binding.recordButton.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) {
                val config_string = serializeRecordConfig()
                startDumpJNI(config_string)
            } else
                stopDumpJNI()
        }

/*        binding.recordCam.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) openPreview() else closePreview()
        }*/

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

    fun serializeRecordConfig/*AndCreateDataFolder*/(): String {
        var record_config = RecordConfig(
            binding.folderName.text.toString(),
            SimpleDateFormat("'-D'yyyy-MM-dd'-T'HH:mm:ss").format(Date()),
            binding.recordCam.isChecked,
            binding.saveImgs.isChecked,
            binding.camId.selectedItem.toString(),
            binding.camResolution.selectedItemPosition,
            binding.enable60hz.isChecked,
            binding.tsFormat.selectedItem.toString(),
            binding.recordImu.isChecked,
            binding.syncAccGyr.isChecked,
            binding.imuFps.selectedItemPosition,
            binding.imuFile.selectedItem.toString(),
            binding.imuOrder.selectedItemPosition,
            binding.packRosbag.isChecked
        )
        val record_config_string = json.encodeToString(record_config)
//        Log.i(debug_tag, "record_config string $record_config_string")
        return record_config_string

/*        val data_folder = File("/sdcard/slam_app/RecordedData/")
        if (!data_folder.exists())
            data_folder.mkdir()

        val cur_record_folder = File("${data_folder.path}/" + record_config.folder_name + record_config.time_postfix)
        if (cur_record_folder.exists())
            cur_record_folder.deleteRecursively()
        cur_record_folder.mkdir()

        val record_config_file = File("${cur_record_folder.path}/" +
                "record_config.json")
        record_config_file.writeText(record_config_string)*/
    }

    external fun getBackCamIDs() : Array<String>
    external fun getImuFreqs() : Array<String>
    // Kotlin alternatives
//    fun getBackCamIDs() : Array<String> { }
//    fun getImuFreqs() : Array<String> { }

    external fun startDumpJNI(config: String)
    external fun stopDumpJNI()
//    external fun openPreview()
//    external fun closePreview()
    external fun sendSurfaceToJNI(cam_sf: Surface)

    companion object {
        // Used to load the 'native-lib' library on application startup.
        init {
            System.loadLibrary("data_record")
        }
    }
}