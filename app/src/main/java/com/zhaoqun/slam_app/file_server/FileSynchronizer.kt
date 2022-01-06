package com.zhaoqun.slam_app.file_server

import android.os.Build
import android.util.Log
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import kotlinx.coroutines.*

class FileSynchronizer {
    var access_token = "121.9825db96b99baafd8f8b89c0aa6c2d6d.YmKLp7XkQAUh1BRRr5UzOkFVNr0u8Kl0Tk6UEES.19LFNQ"
    var refresh_token = "122.50b33fb54bf4c4bf9e2fbec28551591c.YlxWrZtiKLILd-RP5vuts9I9eTaQyHxFr64tCQT.Sh07bg"

    private val netDiskAPI by lazy {
        NetDiskAPI.create()
    }

    private fun apiErrorHandler() {
        Log.e("BaiduNetDiskAPI", "API call error!")
    }
    /// template
    /*        call.enqueue(object : Callback< > {
            override fun onResponse(call: Call< >, response: Response< >) {
                if (response.code() == 200) {
                    val res = response.body()!!
                    // Do stuff with res.
                } else
                    apiErrorHandler()
            }
            override fun onFailure(call: Call< >, t: Throwable) {
                apiErrorHandler()
            }
        })*/

    public fun run() {
        val phone_model: String = Build.MODEL
        var supported: Boolean = false

        val getSupportModels = netDiskAPI.getFileList(access_token, "list","/apps/SLAM APP")
        getSupportModels.enqueue(object : Callback<FileListResponse> {
            override fun onResponse(call: Call<FileListResponse>, response: Response<FileListResponse>) {
                if (response.code() == 200) {
                    val res = response.body()!!
                    // Do stuff with res.
                    for (f in res.list) {
//                        if (f.server_filename.equals(phone_model))
//                            supported = true
                        println("---" + f.server_filename + "\n")
                    }


/*                    val sync_call = netDiskAPI.getRecursiveFileList(access_token, "listall", 1, "/apps/SLAM APP")
                    val sync_res = sync_call.execute()
                    println("---sync call test----" + sync_res)*/

                } else
                    apiErrorHandler()
            }
            override fun onFailure(call: Call<FileListResponse>, t: Throwable) {
                apiErrorHandler()
            }
        })

/*        while (!getSupportModels.isExecuted) {
            Thread.sleep(1)
        }
        println("phone supported? $supported")*/
    }
}