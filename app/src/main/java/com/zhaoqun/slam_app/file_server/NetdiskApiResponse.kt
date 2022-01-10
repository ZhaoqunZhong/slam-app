package com.zhaoqun.slam_app.file_server

import java.util.ArrayList

class CloudFile {
    var fs_id: ULong = 0u
    var path: String = ""
    var server_filename: String = ""
    var server_mtime: UInt = 0u
}

class FileListResponse {
    var list = ArrayList<CloudFile>()
}

class FileMeta {
    var dlink: String = ""
    var filename: String = ""
}
class FileMetasResponse {
    var list = ArrayList<FileMeta>()
}