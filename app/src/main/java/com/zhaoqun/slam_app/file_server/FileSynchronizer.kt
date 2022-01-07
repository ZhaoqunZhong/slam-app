package com.zhaoqun.slam_app.file_server

import android.content.Context
import android.os.Build
import android.os.Environment
import android.util.Log
import androidx.appcompat.app.AppCompatActivity
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import kotlinx.coroutines.*
import androidx.lifecycle.ViewModel
//import com.squareup.moshi.JsonAdapter
//import com.squareup.moshi.JsonClass
//import com.squareup.moshi.Moshi
import com.zhaoqun.slam_app.MainActivity
import java.io.File
import kotlinx.serialization.*
import kotlinx.serialization.json.*


class FileSynchronizer(val path: String) {
    var access_token = "121.9825db96b99baafd8f8b89c0aa6c2d6d.YmKLp7XkQAUh1BRRr5UzOkFVNr0u8Kl0Tk6UEES.19LFNQ"
    var refresh_token = "122.50b33fb54bf4c4bf9e2fbec28551591c.YlxWrZtiKLILd-RP5vuts9I9eTaQyHxFr64tCQT.Sh07bg"

    private val netDiskAPI by lazy {
        NetDiskAPI.create()
    }

    private fun apiErrorHandler() {
        Log.e("BaiduNetDiskAPI", "API call error!")
    }

    /// Retrofit async call template
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

    val io_scope = CoroutineScope(Job() + Dispatchers.IO)

    public fun run() {
        val phone_model: String = Build.MODEL
        var supported: Boolean = false

        io_scope.launch {
            /// Refresh token if necessary.

            /// Check if current phone model is supported.
            val folder_list_call = netDiskAPI.getFileList(access_token, "list","/apps/SLAM APP")
            val folder_list_res = folder_list_call.execute()
            if (folder_list_res.code() == 200) {
                val res = folder_list_res.body()!!
                for (f in res.list) {
                    if (f.server_filename.equals(phone_model))
                        supported = true
                }
            }
            if (!supported) {
                // Hint that current phone is not supported, maybe send phone_model to server.

            }

            /// Sync files.
/*            class FileEntry(
                val name: String,
                val m_time: UInt
            )
            class FileTable(
                val files: List<FileEntry>
            )
            val moshi: Moshi = Moshi.Builder().build()
            val jsonAdapter: JsonAdapter<FileTable> = moshi.adapter(FileTable::class.java)
            val local_json: String = "[{name : file1, m_time: 122121}, {name: file2, m_time: 3434343}]"
            val local_files = jsonAdapter.fromJson(local_json)
            println(local_files)*/
            @Serializable
            data class Person(val name:String = "nobody",val age:Int = 0)

            val json = Json {
                prettyPrint = true //是否格式化，默认false
                prettyPrintIndent = "  " //除首行外各行缩进，默认四个空格
                coerceInputValues = true
            }
            println("decode result: " + json.decodeFromString<List<Person>>("[{}, {}]"))

            println("internal files dir: " + path)
//            File(path + "test.json").writeText("[{'name': 'file1', 'm_time' : 123212}, {'name' : 'file2', 'm_time' : 43434343}]")
            File(path + "test.json").writeText(json.encodeToString(Person("小明",18)))
            val file_list_call = netDiskAPI.getRecursiveFileList(access_token, "listall", 1, "/apps/SLAM APP/${phone_model}")
            val file_list_res = file_list_call.execute()
            if (file_list_res.code() == 200) {
                val res = file_list_res.body()!!
                for (f in res.list) {
                    println("file name: ${f.server_filename}, modify time: ${f.server_mtime}")
                }
            }
        }

    }
}