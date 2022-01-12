package com.zhaoqun.slam_app.file_server

import android.graphics.Color
import android.os.Build
import android.util.Log
import kotlinx.coroutines.*
import kotlinx.serialization.*
import kotlinx.serialization.json.*
import android.os.Environment
import android.view.View
import com.google.android.material.snackbar.Snackbar
import com.zhaoqun.slam_app.R
import kotlinx.serialization.Serializable
import java.lang.Exception
import okhttp3.ResponseBody
import java.io.*


class FileSynchronizer(val path: String, val view: View) {
    var access_token = ""

    private val netDiskAPI by lazy {
        NetDiskAPI.create()
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

    @Serializable
    data class FileEntry(
        val name: String = "no file",
        var fs_id: ULong = 0u,
        val m_time: UInt = 0u
    )
    val json = Json {
        prettyPrint = true
        prettyPrintIndent = "  "
        coerceInputValues = true
    }

    val io_scope = CoroutineScope(Job() + Dispatchers.IO)
    val tag = "FileSynchronizer"
    public fun run() {
        io_scope.launch {
            var mySnackbar = Snackbar.make(view.findViewById(R.id.filesyncprompt), "Syncing files with server...", Snackbar.LENGTH_INDEFINITE)
            mySnackbar.show()

            val acquire_token_call = netDiskAPI.downloadFileWithUrl("https://gitee.com/zhaoqun-zhong/slam_app_file_server/raw/master/access_token.txt")
            val acquire_token_res = acquire_token_call.execute()
            if (acquire_token_res.code() == 200) {
                writeResponseBodyToDisk(acquire_token_res.body()!!, "access_token")
                access_token = File(path + "access_token").readText()
            } else {
                Log.e(tag, "Acquire token failed!")
                println(acquire_token_res.toString())
            }

            /// Check if current phone model is supported.
            val phone_model: String = Build.MODEL
            var supported: Boolean = false
            val folder_list_call = netDiskAPI.getFileList(access_token, "list","/apps/SLAM APP")
            val folder_list_res = folder_list_call.execute()
            if (folder_list_res.code() == 200) {
                val res = folder_list_res.body()!!
                for (f in res.list) {
                    if (f.server_filename.equals(phone_model) && f.isdir == 1u)
                        supported = true
                }
            } else {
                Log.e(tag, "Get supported models failed!")
            }
            if (!supported) {
                // Hint that current phone is not supported, maybe send phone_model to server.
                
            }

            /// Sync files.
            // get file list from server side
            var server_file_list_data = mutableListOf<FileEntry>()
            val file_list_call = netDiskAPI.getRecursiveFileList(access_token, "listall", 1, "/apps/SLAM APP/${phone_model}")
            val file_list_res = file_list_call.execute()
            if (file_list_res.code() == 200) {
                val res = file_list_res.body()!!
                for (f in res.list) {
                    //println("file name: ${f.server_filename}, modify time: ${f.server_mtime}")
                    server_file_list_data.add(FileEntry(f.server_filename, f.fs_id, f.server_mtime))
                }
            } else {
                Log.e(tag, "Get server file list failed!")
                println(file_list_res.toString())
            }
            val server_file_list_string = json.encodeToString(server_file_list_data)
//            println(server_file_list_string)

            // get local file list
            val local_list_file = File(path + "file_list.json")
            var local_file_list_data = mutableListOf<FileEntry>()
            if (local_list_file.exists()) {
                val local_list_string = local_list_file.readText()
                //println("local list string ${local_list_string}")
                local_file_list_data = json.decodeFromString<MutableList<FileEntry>>(local_list_string)
                //println(local_list_data)
            } else {
                //println("local file doesn't exists.")
            }
            syncWithServer(local_file_list_data, server_file_list_data)
            local_list_file.writeText(server_file_list_string)

            mySnackbar.dismiss()
            mySnackbar = Snackbar.make(view.findViewById(R.id.filesyncprompt), "File sync success.", Snackbar.LENGTH_SHORT)
            mySnackbar.setBackgroundTint(Color.parseColor("#FF03DAC5"))
            mySnackbar.show()
        }
    }

    private fun syncWithServer(local: MutableList<FileEntry>, server: MutableList<FileEntry>) {
        var local_filenames = mutableListOf<String>()
        for (lf in local) {
            local_filenames.add(lf.name)
        }
        var files_to_download = ArrayList<ULong>()
        for (sf in server) {
            if (local_filenames.contains(sf.name)) {
                //println(local[local_filenames.indexOf(sf.name)])
                if (local[local_filenames.indexOf(sf.name)].m_time == sf.m_time) {
                    println("File: ${sf.name} up to dates.")
                } else {
                    // Delete local file and download a new one from server
                    File(path + sf.name).delete()
                    files_to_download.add(sf.fs_id)
                }
            } else {
                // Download this file from server to local
                files_to_download.add(sf.fs_id)

            }
        }
        downloadServerFiles(files_to_download)
    }

    private fun downloadServerFiles(fs_ids: ArrayList<ULong>) {
        val file_meta_call = netDiskAPI.getFileMetas(access_token, "filemetas", fs_ids.toString(), 1)
        val file_meta_res = file_meta_call.execute()
        var links = mutableListOf<String>()
        var names = mutableListOf<String>()
        if (file_meta_res.code() == 200) {
            val res = file_meta_res.body()!!
            for (f in res.list) {
                links.add(f.dlink)
                names.add(f.filename)
            }
        } else {
            Log.e(tag, "Get file download links failed!")
            return
        }
//        println("links: " + links)

        for (lk in links) {
            val download_call = netDiskAPI.downloadFileWithUrl(lk + "&access_token=${access_token}")
            val download_res = download_call.execute()
            if (download_res.code() == 200) {
                val fname = names[links.indexOf(lk)]
                if (writeResponseBodyToDisk(download_res.body()!!, fname))
                    println("Successfully downloaded file: ${fname}")
                else
                    Log.e(tag, "Download file: ${fname} failed!")
            } else {
                Log.e(tag, "Download file API error!")
                return
            }

        }
    }

    private fun writeResponseBodyToDisk(body: ResponseBody, filename: String): Boolean {
        return try {
            val file = File(path, filename)
            var inputStream: InputStream? = null
            var outputStream: OutputStream? = null
            try {
                val fileReader = ByteArray(1024*8)
                val fileSize = body.contentLength()
                var fileSizeDownloaded: Long = 0
                inputStream = body.byteStream()
                outputStream = FileOutputStream(file)
                while (true) {
                    val read: Int = inputStream.read(fileReader)
                    if (read == -1) {
                        break
                    }
                    outputStream.write(fileReader, 0, read)
                    fileSizeDownloaded += read.toLong()
                    Log.i("writeResponseBodyToDisk", "Download ${filename}: $fileSizeDownloaded of $fileSize")
                }
                outputStream.flush()
                true
            } catch (e: IOException) {
                false
            } finally {
                inputStream?.close()
                outputStream?.close()
            }
        } catch (e: IOException) {
            false
        }
    }
}

