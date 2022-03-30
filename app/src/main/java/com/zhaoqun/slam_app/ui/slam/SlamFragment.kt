package com.zhaoqun.slam_app.ui.slam

import android.R
import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.AdapterView
import android.widget.ArrayAdapter
import android.widget.TextView
import androidx.core.view.marginStart
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import androidx.lifecycle.Observer
import androidx.lifecycle.ViewModelProvider
import com.zhaoqun.slam_app.databinding.FragmentSlamBinding
import com.zhaoqun.slam_app.ui.data_record.DataRecordViewModel
import kotlinx.serialization.Serializable
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import java.io.File

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

        return root
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }



    fun prepareDefaultOptions () {
        slamViewModel._cam_res.observe(viewLifecycleOwner, {binding.camResSlam.setSelection(it)})
        slamViewModel._enable_60hz.observe(viewLifecycleOwner, {binding.cam60hzSlam.isChecked = it})

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

    external fun getBackCamIDs() : Array<String>
    external fun getImuFreqs() : Array<String>

    companion object {
        // Used to load the 'native-lib' library on application startup.
        init {
            System.loadLibrary("slam")
        }
    }
}