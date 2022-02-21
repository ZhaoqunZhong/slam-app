package com.zhaoqun.slam_app

import android.Manifest
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
import android.content.pm.PackageManager
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
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.lifecycle.ViewModelProvider
import com.zhaoqun.slam_app.ui.data_record.DataRecordViewModel
import com.zhaoqun.slam_app.ui.image_processing.GalleryViewModel
import java.lang.Exception


class MainActivity : AppCompatActivity() {
    private lateinit var appBarConfiguration: AppBarConfiguration
    private lateinit var binding: ActivityMainBinding
    private lateinit var fsync: FileSynchronizer

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

//        checkPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE, STORAGE_PERMISSION_CODE)
//        checkPermission(Manifest.permission.CAMERA, CAMERA_PERMISSION_CODE)
        if (allPermissionsGranted()) {
            Log.i("permission_check", "All permissions granted.")
        } else {
            Log.w("permission_check", "Request permissions.")
            ActivityCompat.requestPermissions(
                this, REQUIRED_PERMISSIONS, REQUEST_CODE_PERMISSIONS)
        }

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

        fsync = FileSynchronizer(applicationContext, "${filesDir}/",
            "${getExternalFilesDir(Environment.DIRECTORY_DOWNLOADS).toString()}/", binding.root.rootView)
        fsync.run()
//        Log.i("slam_app", "java path ${getExternalFilesDir(Environment.DIRECTORY_DOWNLOADS)}")
    }

    override fun onCreateOptionsMenu(menu: Menu): Boolean {
        // Inflate the menu; this adds items to the action bar if it is present.
        menuInflater.inflate(R.menu.main, menu)
        return true
    }

    override fun onOptionsItemSelected(item: MenuItem): Boolean {
        if (item.itemId == R.id.action_settings) {
/*            val apk_path = "${getExternalFilesDir(Environment.DIRECTORY_DOWNLOADS).toString()}/slam_app.apk"
            installApk(applicationContext, apk_path)*/
        }
        return super.onOptionsItemSelected(item)
    }

    override fun onSupportNavigateUp(): Boolean {
        val navController = findNavController(R.id.nav_host_fragment_content_main)
        return navController.navigateUp(appBarConfiguration) || super.onSupportNavigateUp()
    }

    private fun allPermissionsGranted() = REQUIRED_PERMISSIONS.all {
        ContextCompat.checkSelfPermission(baseContext, it) == PackageManager.PERMISSION_GRANTED
    }

/*    private fun checkPermission(permission: String, requestCode: Int) {
        if (ContextCompat.checkSelfPermission(this@MainActivity, permission) == PackageManager.PERMISSION_DENIED) {
            // Requesting the permission
            ActivityCompat.requestPermissions(this@MainActivity, arrayOf(permission), requestCode)
        } else {
//            Toast.makeText(this@MainActivity, permission + " permission already granted", Toast.LENGTH_SHORT).show()
        }
    }*/

    companion object {
        private const val REQUEST_CODE_PERMISSIONS = 666
        private val REQUIRED_PERMISSIONS = arrayOf(
            Manifest.permission.INTERNET,
            Manifest.permission.CAMERA,
            Manifest.permission.REQUEST_INSTALL_PACKAGES,
            Manifest.permission.MANAGE_EXTERNAL_STORAGE,
//            Manifest.permission.MOUNT_UNMOUNT_FILESYSTEMS,
//            Manifest.permission.WRITE_EXTERNAL_STORAGE
        )
//        private const val CAMERA_PERMISSION_CODE = 100
//        private const val STORAGE_PERMISSION_CODE = 200
        // Used to load the 'native-lib' library on application startup.
        init {
//            System.loadLibrary("slam_app")
        }
    }


}