package com.zhaoqun.slam_app.ui.data_record

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

class DataRecordFragment : Fragment() {
    private val dataRecordViewModel: DataRecordViewModel by activityViewModels()
    private var _binding: FragmentDataBinding? = null

    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!

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
    }

    external fun getBackCamIDs() : ArrayList<Int>
    external fun getImuFreqs() : ArrayList<Int>
    external fun startDumpJNI()
    external fun stopDumpJNI()

    companion object {
        // Used to load the 'native-lib' library on application startup.
        init {
            System.loadLibrary("data_record")
        }
    }
}