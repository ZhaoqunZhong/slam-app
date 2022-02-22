package com.zhaoqun.slam_app.file_server

import android.content.Context
import android.content.Intent
import android.graphics.Color
import android.net.Uri
import android.os.Build
import android.util.Log
import kotlinx.coroutines.*
import kotlinx.serialization.*
import kotlinx.serialization.json.*
import android.view.View
import android.widget.Toast
import androidx.core.content.FileProvider
import com.google.android.material.snackbar.Snackbar
import com.zhaoqun.slam_app.BuildConfig
import com.zhaoqun.slam_app.R
import kotlinx.serialization.Serializable
import okhttp3.ResponseBody
import java.io.*
import java.lang.Exception
import java.math.BigInteger
import java.security.MessageDigest
import kotlin.coroutines.CoroutineContext


class FileSynchronizer(val context: Context, val app_path: String, val download_path: String, val view: View) {
    var access_token = ""

    private val netDiskAPI by lazy {
        NetDiskAPI.create()
    }
/*    private val netDiskUploadAPI by lazy {
        NetDiskUploadAPI.create()
    }*/

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
            var file_snackbar : Snackbar

            file_snackbar = Snackbar.make(view.findViewById(R.id.filesyncprompt), "Syncing files with server...", Snackbar.LENGTH_INDEFINITE)
            file_snackbar.setBackgroundTint(Color.parseColor("#FF0000FF"))
            file_snackbar.show()

            //-----------------Get Baidu Netdisk access token from gitee--------------------
            val acquire_token_call = netDiskAPI.downloadFileWithUrl("https://gitee.com/zhaoqun-zhong/slam_app_file_server/raw/master/access_token.txt")
            val acquire_token_res = acquire_token_call.execute()
            if (acquire_token_res.code() == 200) {
                writeResponseBodyToDisk(acquire_token_res.body()!!, app_path, "access_token")
                access_token = File(app_path + "access_token").readText()
            } else {
                Log.e(tag, "Acquire token failed! \n" + acquire_token_res.toString())
            }

            /// Check if current phone model is supported, and get dlinks for version and apk files.
            val phone_model: String = Build.MODEL
            var supported: Boolean = false
            var version_file_dlink = ""
            var apk_file_dlink = ""
            val folder_list_call = netDiskAPI.getFileList(access_token, "list","/apps/SLAM_APP")
            val folder_list_res = folder_list_call.execute()
            if (folder_list_res.code() == 200) {
                val res = folder_list_res.body()!!
                for (f in res.list) {
                    if (f.server_filename.equals(phone_model) && f.isdir == 1u)
                        supported = true
                    if (f.server_filename == "version_name.txt") {
                        val version_dlink_call = netDiskAPI.getFileMetas(access_token, "filemetas", "[${f.fs_id}]", 1)
                        val version_dlink_res = version_dlink_call.execute()
                        if (version_dlink_res.code() == 200)
                            version_file_dlink = version_dlink_res.body()!!.list.first().dlink
                        else
                            Log.e(tag, "Get version file dlink failed!")
                    }
                    if (f.server_filename == "slam_app.apk") {
                        val apk_dlink_call = netDiskAPI.getFileMetas(access_token, "filemetas", "[${f.fs_id}]", 1)
                        val apk_dlink_res = apk_dlink_call.execute()
                        if (apk_dlink_res.code() == 200)
                            apk_file_dlink = apk_dlink_res.body()!!.list.first().dlink
                        else
                            Log.e(tag, "Get apk file dlink failed!")
                    }
                }
            } else {
                Log.e(tag, "Get supported models failed!")
            }
            if (!supported) {
                // Hint that current phone is not supported.
                file_snackbar = Snackbar.make(view.findViewById(R.id.filesyncprompt), "This phone model is not supported yet.",
                    Snackbar.LENGTH_INDEFINITE)
                file_snackbar.setBackgroundTint(Color.parseColor("#FFFF0000"))
                file_snackbar.show()

                // Send phone model to server.
                val upload_model_call = netDiskAPI.uploadCreate("create", access_token,
                    "/apps/SLAM_APP/${phone_model}-unsupported", "0", "1", 0)
                val upload_model_res = upload_model_call.execute()
                if (upload_model_res.code() == 200) {
                    Log.i(tag, "Successfully sent phone model to server.")
                } else
                    Log.e(tag, "Send phone model to server failed!")
//                println(upload_model_res.toString() + "  " + upload_model_res.body()!!.string())
                delay(2000)
                file_snackbar = Snackbar.make(view.findViewById(R.id.filesyncprompt), "This phone model has been sent to server, check back later.",
                    Snackbar.LENGTH_INDEFINITE)
                file_snackbar.setBackgroundTint(Color.parseColor("#FFFF0000"))
                file_snackbar.show()
/*
                // Test upload file to server
                val upload_file_call = netDiskUploadAPI.uploadFile("upload", access_token, "apps/SLAM_APP/config.yaml",
                    File(app_path + "config.yaml").readBytes())
                val upload_file_res = upload_file_call.execute()
                if (upload_file_res.code() == 200) {
                    Log.i(tag, "Upload file to server success.")
                } else {
                    Log.e(tag, "Upload file to server failed!")
//                    println(upload_file_res.body()!!.string())
                    println(upload_file_res.toString())
                }

                // Test upload precreate part
                val file_md5 = getMD5(File(app_path + "config.yaml").readText())
                println(file_md5)
*//*                val pre_req = NetDiskAPI.preCreateReq("apps/SLAM_APP/config.yaml",
                    "645", "0", 1, 0,  listOf<String>(file_md5).toString())
                val upload_pre_call = netDiskAPI.uploadPrecreate("precreate", access_token, pre_req)*//*
                val file_url = java.net.URLEncoder.encode("apps/SLAM_APP/config.yaml", "utf-8")
                println(file_url)
                val upload_pre_call = netDiskAPI.uploadPrecreate("precreate", access_token, file_url,
                    "645", "0", 1, 3,  listOf<String>(file_md5).toString())
                val upload_pre_res = upload_pre_call.execute()
                println(upload_pre_res.toString())
                if (upload_pre_res.code() == 200) {
                    Log.i(tag, "upload_pre_call success.")
                    val up_id = upload_pre_res.body()!!.uploadid
                    val blocklist = upload_pre_res.body()!!.block_list
                    println("upload id: ${up_id}")
                    println("block list: ${blocklist}")
                } else {
                    Log.e(tag, "upload_pre_call failed!")
                }*/

                awaitCancellation()
            }

            /// Check App version from server.
            var version_name = ""
            val version_name_call = netDiskAPI.downloadFileWithUrl(version_file_dlink + "&access_token=${access_token}")
            val version_name_res = version_name_call.execute()
            if (version_name_res.code() == 200) {
                writeResponseBodyToDisk(version_name_res.body()!!, app_path, "version_name")
                version_name = File(app_path + "version_name").readText()
            } else {
                Log.e(tag, "Acquire version name failed!")
                println(version_name_res.toString())
            }
            if (version_name != BuildConfig.VERSION_NAME) {
                file_snackbar = Snackbar.make(view.findViewById(R.id.filesyncprompt), "New App version available, downloading apk for you...",
                    Snackbar.LENGTH_INDEFINITE)
                file_snackbar.setBackgroundTint(Color.parseColor("#FF0000FF"))
                file_snackbar.show()
                // Download apk from server, and save to Download folder.
                val download_apk_call = netDiskAPI.downloadFileWithUrl(apk_file_dlink + "&access_token=${access_token}")
                val download_apk_res = download_apk_call.execute()
                if (download_apk_res.code() == 200) {
                    writeResponseBodyToDisk(download_apk_res.body()!!, download_path, "slam_app.apk")
/*                    file_snackbar = Snackbar.make(view.findViewById(R.id.filesyncprompt), "New apk file has been downloaded to app's folder.",
                        Snackbar.LENGTH_SHORT)
                    file_snackbar.setBackgroundTint(Color.parseColor("#FF00FF00"))
                    file_snackbar.show()*/
                    val apk_path = "${download_path}/slam_app.apk"
                    installApk(context, apk_path)
                } else {
                    Log.e(tag, "Download new version apk failed!")
                    println(download_apk_res.toString())
//                    file_snackbar.dismiss()
                    file_snackbar = Snackbar.make(view.findViewById(R.id.filesyncprompt), "Download new apk file failed.",
                        Snackbar.LENGTH_SHORT)
                    file_snackbar.setBackgroundTint(Color.parseColor("#FFFF0000"))
                    file_snackbar.show()
                    delay(2000)
                }

            }

            /// Sync files.
            // get file list from server side
            var server_file_list_data = mutableListOf<FileEntry>()
            val file_list_call = netDiskAPI.getRecursiveFileList(access_token, "listall", 1, "/apps/SLAM_APP/${phone_model}")
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
            val local_list_file = File(app_path + "file_list.json")
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

//            file_snackbar.dismiss()
            file_snackbar = Snackbar.make(view.findViewById(R.id.filesyncprompt), "File sync success.", Snackbar.LENGTH_SHORT)
            file_snackbar.setBackgroundTint(Color.parseColor("#FF00FF00"))
            file_snackbar.show()
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
                    File(app_path + sf.name).delete()
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
                if (writeResponseBodyToDisk(download_res.body()!!, app_path, fname))
                    println("Successfully downloaded file: ${fname}")
                else
                    Log.e(tag, "Download file: ${fname} failed!")
            } else {
                Log.e(tag, "Download file API error!")
                return
            }

        }
    }

    private fun writeResponseBodyToDisk(body: ResponseBody, downloadPath: String, filename: String): Boolean {
        return try {
            val file = File(downloadPath, filename)
            var inputStream: InputStream? = null
            var outputStream: OutputStream? = null
            try {
                val fileReader = ByteArray(1024*4)
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
//                outputStream.flush()
                true
            } catch (e: IOException) {
                e.printStackTrace();
                false
            } finally {
                inputStream?.close()
                outputStream?.close()
            }
        } catch (e: IOException) {
            e.printStackTrace();
            false
        }
    }

/*    fun getMD5(input:String): String {
        val md = MessageDigest.getInstance("MD5")
        return BigInteger(1, md.digest(input.toByteArray())).toString(16).padStart(32, '0')
    }
    fun md5(content: String): String {
        val hash = MessageDigest.getInstance("MD5").digest(content.toByteArray())
        val hex = StringBuilder(hash.size * 2)
        for (b in hash) {
            var str = Integer.toHexString(b.toInt())
            if (b < 0x10) {
                str = "0$str"
            }
            hex.append(str.substring(str.length -2))
        }
        return hex.toString()
    }*/
    /**
     * 安装apk
     *
     * @param context
     * @param apkPath
     */
    fun installApk(context: Context, apkPath: String?) {
        try {
            /**
             * provider
             * 处理android 7.0 及以上系统安装异常问题
             */
            val file = File(apkPath)
            val install = Intent()
            install.action = Intent.ACTION_VIEW
            install.addCategory(Intent.CATEGORY_DEFAULT)
            install.flags = Intent.FLAG_ACTIVITY_NEW_TASK
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                val apkUri = FileProvider.getUriForFile(
                    context,
                    "com.zhaoqun.slam_app.fileprovider",
                    file
                ) //在AndroidManifest中的android:authorities值
                install.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION) //添加这一句表示对目标应用临时授权该Uri所代表的文件
                install.setDataAndType(apkUri, "application/vnd.android.package-archive")
            } else {
                install.setDataAndType(
                    Uri.fromFile(file),
                    "application/vnd.android.package-archive"
                )
            }
            context.startActivity(install)
        } catch (e: Exception) {
            Toast.makeText(context, "文件解析失败", Toast.LENGTH_SHORT).show()
        }
    }
}

