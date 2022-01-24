package com.zhaoqun.slam_app.ui.data_record

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.fragment.app.Fragment
import androidx.lifecycle.Observer
import androidx.lifecycle.ViewModelProvider
import com.zhaoqun.slam_app.databinding.FragmentDataBinding

class DataFragment : Fragment() {

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
        return root
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    fun prepareDefaultOptions () {

    }

    external fun startDumpJNI()
    external fun stopDumpJNI()

    companion object {
        // Used to load the 'native-lib' library on application startup.
        init {
            System.loadLibrary("native_lib")
        }
    }
}