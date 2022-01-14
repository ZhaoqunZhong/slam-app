package com.zhaoqun.slam_app.file_server

import retrofit2.Call
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory

import okhttp3.ResponseBody
import retrofit2.http.*


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

    @GET("multimedia?")
    fun getFileMetas(@Query("access_token") access_token: String, @Query("method") method: String,
                        @Query("fsids", encoded = true) fsids: String, @Query("dlink")dlink: Int): Call<FileMetasResponse>

    @Streaming
    @GET    // This method is irrelevant to the baseUrl, so can be used with any internet file.
    fun downloadFileWithUrl(@Url fileUrl: String): Call<ResponseBody>


/*    class preCreateReq (var path: String, var size: String, var isdir: String, var autoinit: Int, var rtype: Int, var block_list: String)
    @POST("file?")
    fun uploadPrecreate(@Query("method")method: String, @Query("access_token") access_token: String, @Body req: preCreateReq): Call<ResponseBody>*/
    @FormUrlEncoded
    @POST("file?")
    fun uploadPrecreate(@Query("method")method: String, @Query("access_token") access_token: String, @Field("path") path: String,
                     @Field("size")size: String, @Field("isdir")isdir: String, @Field("autoinit")autoinit: Int,
                        @Field("rtype")rtype: Int, @Field("block_list")block_list: String): Call<precreateResponse>

    @FormUrlEncoded
    @POST("file?")
    fun uploadCreate(@Query("method")method: String, @Query("access_token") access_token: String, @Field("path") path: String,
                     @Field("size")size: String, @Field("isdir")isdir: String, @Field("rtype")rtype: Int): Call<ResponseBody>
}

interface NetDiskUploadAPI {

    companion object {
        fun create(): NetDiskUploadAPI {
            val retrofit = Retrofit.Builder()
                .addConverterFactory(GsonConverterFactory.create())
                .baseUrl("https://c.pcs.baidu.com/rest/2.0/pcs/")
                .build()

            return retrofit.create(NetDiskUploadAPI::class.java)
        }
    }

    @FormUrlEncoded
    @POST("file?")
    fun uploadFile(@Query("method")method: String, @Query("access_token") access_token: String,
                   @Query("path") path: String, @Field("file")file: ByteArray): Call<ResponseBody>
}