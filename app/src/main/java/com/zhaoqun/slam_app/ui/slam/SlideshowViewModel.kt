package com.zhaoqun.slam_app.ui.slam

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel

class SlideshowViewModel : ViewModel() {

    private val _text = MutableLiveData<String>().apply {
        value = "Coming soon!"
    }
    val text: LiveData<String> = _text
}