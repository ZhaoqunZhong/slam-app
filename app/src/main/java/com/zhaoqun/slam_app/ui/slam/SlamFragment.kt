package com.zhaoqun.slam_app.ui.slam

import android.R
import android.icu.text.SimpleDateFormat
import android.os.Bundle
import android.os.Environment
import android.util.Log
import android.view.*
import android.widget.AdapterView
import android.widget.ArrayAdapter
import android.widget.TextView
import androidx.core.view.marginStart
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import androidx.lifecycle.Observer
import androidx.lifecycle.ViewModelProvider
import androidx.lifecycle.lifecycleScope
import com.zhaoqun.slam_app.databinding.FragmentSlamBinding
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.serialization.Serializable
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import java.io.File
import java.util.*
import kotlin.collections.ArrayList

class SlamFragment : Fragment() {
    private val slamViewModel: SlamViewModel by activityViewModels()
    private var _binding: FragmentSlamBinding? = null
    private val debug_tag = "SlamFragment"
    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!
    val json = Json {
        prettyPrint = true
        prettyPrintIndent = "  "
        coerceInputValues = true
    }

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        _binding = FragmentSlamBinding.inflate(inflater, container, false)
        val root: View = binding.root

        prepareDefaultOptions()
        backgroundDataTask()

        binding.camResSlam.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(parent: AdapterView<*>?, view: View?, position: Int, id: Long) {
//                Log.i(debug_tag, "selected $position")
                val lp: ViewGroup.LayoutParams = binding.previewSlam.layoutParams
//                lp.width = binding.dividerVertical.marginStart
//                Log.i(debug_tag, "surfaceview size before ${lp.width} ${lp.height}")
                if (position == 0)
                    lp.height = (640.0 / 480 * lp.width).toInt()
                if (position == 1)
                    lp.height = (1280.0 / 720 * lp.width).toInt()
                if (position == 2)
                    lp.height = (1920.0 / 1080 * lp.width).toInt()
//                Log.i(debug_tag, "surfaceview size after ${lp.width} ${lp.height}")
                binding.previewSlam.layoutParams = lp
            }
            override fun onNothingSelected(p0: AdapterView<*>?) { }
        }

        binding.previewSlam.holder.addCallback(
            object : SurfaceHolder.Callback {
                override fun surfaceCreated(holder: SurfaceHolder) { }
                override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {
//                    Log.i(debug_tag, "cam preview surfacechanged, width $width, height $height")
                    sendSurfaceToJNI(holder.surface)
                }
                override fun surfaceDestroyed(holder: SurfaceHolder) { }
            }
        )

        binding.startSlam.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) {
                val config_string = serializeSlamDataConfig()
                startSlamJNI(config_string)
            } else
                stopSlamJNI()
        }

        return root
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
        stopSlamJNI()
    }



    fun prepareDefaultOptions () {
        slamViewModel._cam_res.observe(viewLifecycleOwner, {binding.camResSlam.setSelection(it)})
        slamViewModel._enable_60hz.observe(viewLifecycleOwner, {binding.cam60hzSlam.isChecked = it})
        slamViewModel._cam_stream_freq.observe(viewLifecycleOwner, {binding.camStreamFps.text = it})
        slamViewModel._imu_stream_freq.observe(viewLifecycleOwner, {binding.imuStreamFps.text = it})
        slamViewModel._pose_freq.observe(viewLifecycleOwner, {binding.poseFps.text = it})

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

        val aa = ArrayAdapter(requireContext(), R.layout.simple_spinner_item, cam_ids)
        aa.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        binding.camIdSlam.setAdapter(aa)

        val aa1 = ArrayAdapter(requireContext(), R.layout.simple_spinner_item, imu_freqs)
        aa1.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        binding.imuFreqSlam.setAdapter(aa1)
    }

    fun serializeSlamDataConfig(): String {
        var slam_config = SlamDataConfig(
//            "${context?.filesDir.toString()}/",
            "${context?.getExternalFilesDir(Environment.DIRECTORY_DOWNLOADS).toString()}/",
            binding.camIdSlam.selectedItem.toString(),
            binding.camResSlam.selectedItemPosition,
            binding.cam60hzSlam.isChecked,
            binding.imuFreqSlam.selectedItemPosition
        )
        val slam_config_string = json.encodeToString(slam_config)
        return slam_config_string
    }

    fun backgroundDataTask() {
        lifecycleScope.launch(context = Dispatchers.IO){
            while(true){
                Thread.sleep(100)
                slamViewModel._cam_stream_freq.postValue(getCamFps().toString() + "hz")
                slamViewModel._imu_stream_freq.postValue(getImuFps().toString() + "hz")
                slamViewModel._pose_freq.postValue(getPoseFps().toString() + "hz")
            }
        }
    }

    external fun getBackCamIDs() : Array<String>
    external fun getImuFreqs() : Array<String>
    external fun sendSurfaceToJNI(cam_sf: Surface)
    external fun startSlamJNI(config: String)
    external fun stopSlamJNI()
    external fun getCamFps() : Int
    external fun getImuFps() : Int
    external fun getPoseFps() : Int

    companion object {
        // Used to load the 'native-lib' library on application startup.
        init {
            System.loadLibrary("slam")
        }
    }
}