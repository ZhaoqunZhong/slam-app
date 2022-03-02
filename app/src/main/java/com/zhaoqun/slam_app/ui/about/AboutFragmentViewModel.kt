package com.zhaoqun.slam_app.ui.about

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel

class AboutFragmentViewModel : ViewModel() {

    private val _text = MutableLiveData<String>().apply {
        value = "Author: ZhaoqunZhong \n" +
                "Email: zhongzhaoqun1991@outlook.com \n" +
                "\n" +
                "This app is meant to be a tool for collecting data, testing and visualizing SLAM algorithms(especially vio) " +
                "on Android devices. Minimum supported Android version is 10. \n" +
                "\n" +
                "Wiki page about how to use the app can be found at \n" +
                "--- https://www.cnblogs.com/ZhaoqunZhong/p/15949472.html --- \n" +
                "You can also leave your questions in the comment section. I will happily answer them. \n" +
                "\n" +
                "If you benefit from the app in your work or study, and want to kindly \"buy me a cup of coffee\", " +
                "there are sponsor links at the end of wiki page. Thanks~\n"

    }
    val text: LiveData<String> = _text
}