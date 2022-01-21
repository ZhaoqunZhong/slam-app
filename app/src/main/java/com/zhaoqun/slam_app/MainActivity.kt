package com.zhaoqun.slam_app

import android.content.Context
import android.os.Bundle
import android.os.Environment
import android.view.Menu
import android.view.MenuItem
import com.google.android.material.snackbar.Snackbar
import com.google.android.material.navigation.NavigationView
import androidx.navigation.findNavController
import androidx.navigation.ui.AppBarConfiguration
import androidx.navigation.ui.navigateUp
import androidx.navigation.ui.setupActionBarWithNavController
import androidx.navigation.ui.setupWithNavController
import androidx.drawerlayout.widget.DrawerLayout
import androidx.appcompat.app.AppCompatActivity
import com.zhaoqun.slam_app.databinding.ActivityMainBinding
import com.zhaoqun.slam_app.file_server.FileSynchronizer
import android.content.Intent
import android.net.Uri
import android.provider.DocumentsContract
import androidx.core.net.toUri
import androidx.core.app.ActivityCompat.startActivityForResult
import androidx.core.content.FileProvider
import java.io.File
import androidx.core.content.FileProvider.getUriForFile
import android.widget.Toast

import android.os.Build
import android.util.Log
import java.lang.Exception


class MainActivity : AppCompatActivity() {
    private lateinit var appBarConfiguration: AppBarConfiguration
    private lateinit var binding: ActivityMainBinding
    private lateinit var fsync: FileSynchronizer

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        setSupportActionBar(binding.appBarMain.toolbar)

        val drawerLayout: DrawerLayout = binding.drawerLayout
        val navView: NavigationView = binding.navView
        val navController = findNavController(R.id.nav_host_fragment_content_main)
        // Passing each menu ID as a set of Ids because each
        // menu should be considered as top level destinations.
        appBarConfiguration = AppBarConfiguration(
            setOf(
                R.id.nav_home, R.id.nav_gallery, R.id.nav_slideshow
            ), drawerLayout
        )
        setupActionBarWithNavController(navController, appBarConfiguration)
        navView.setupWithNavController(navController)

        fsync = FileSynchronizer("${filesDir}/",
            "${getExternalFilesDir(Environment.DIRECTORY_DOWNLOADS).toString()}/", binding.root.rootView)
        fsync.run()
    }

    override fun onCreateOptionsMenu(menu: Menu): Boolean {
        // Inflate the menu; this adds items to the action bar if it is present.
        menuInflater.inflate(R.menu.main, menu)
        return true
    }

    override fun onOptionsItemSelected(item: MenuItem): Boolean {
        if (item.itemId == R.id.action_settings) {
            val apk_path = "${getExternalFilesDir(Environment.DIRECTORY_DOWNLOADS).toString()}/slam_app.apk"
//        Log.w("apk file : ", apk_path)
            installApk(applicationContext, apk_path)
        }
        return super.onOptionsItemSelected(item)
    }

    override fun onSupportNavigateUp(): Boolean {
        val navController = findNavController(R.id.nav_host_fragment_content_main)
        return navController.navigateUp(appBarConfiguration) || super.onSupportNavigateUp()
    }

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
                val apkUri = getUriForFile(
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