package com.zhaoqun.slam_app.file_server

import retrofit2.Call
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import retrofit2.http.GET
import retrofit2.http.Header
import retrofit2.http.Headers
import retrofit2.http.Query

interface NetDiskAPI {

    companion object {
        fun create(): NetDiskAPI {
            val retrofit = Retrofit.Builder()
                .addConverterFactory(GsonConverterFactory.create())
                .baseUrl("https://pan.baidu.com/rest/2.0/xpan/")
                .build()

            return retrofit.create(NetDiskAPI::class.java)
        }
    }


    @GET("file?")
    fun getRecursiveFileList(@Query("access_token") access_token: String, @Query("method") method: String,
                             @Query("recursion") recursion: Int, @Query("path") path: String): Call<FileListResponse>

//    @Headers("method:list")
    @GET("file?")
    fun getFileList(@Query("access_token") access_token: String, @Query("method")method: String, @Query("dir") dir: String): Call<FileListResponse>
}