package com.zhaoqun.slam_app.ui.slam

import android.content.Context
import android.opengl.GLES30
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.os.SystemClock.elapsedRealtime
import android.util.AttributeSet
import android.util.Log
import android.view.GestureDetector
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sqrt

private const val DEBUG_TAG = "MapViewer"
private var viewMatrix = FloatArray(16)
private val projectionMatrix = FloatArray(16)
private var vPMatrix = FloatArray(16)
private val initViewMat = FloatArray(16)
private var camXshift: Float = 0f
private var camYshift: Float = 0f

private var pivotToWorld = FloatArray(16)
private var camToPivot = FloatArray(16)
private var mScaleFactor = 1f

class MapViewer(context: Context?, attrs: AttributeSet) : GLSurfaceView(context, attrs) {

    private var mapRenderer: MyGLRenderer

    private var fingerNumber: Int = 0
    private var lastScaleTime: Long = 0
    private val scaleLockTime: Long = 500

    private val scaleListener = object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
        override fun onScale(detector: ScaleGestureDetector): Boolean { //scale
            //mScaleFactor *= detector.scaleFactor

            var sf = detector.scaleFactor
            sf = Math.max(0.2f, Math.min(sf, 4.0f))
//            Log.i(DEBUG_TAG, "scale factor: $sf")
            val thred = sf - 1f
            if (thred.absoluteValue < 0.015) {
                return true
            } else {
                lastScaleTime = elapsedRealtime()
            }

            val temp = camToPivot.clone()
            temp[12] = camToPivot[12] / sf
            temp[13] = camToPivot[13] / sf
            temp[14] = camToPivot[14] / sf
            camToPivot = temp.clone()

            Matrix.multiplyMM(viewMatrix,0, camToPivot,0, pivotToWorld,0)

            Matrix.multiplyMM(vPMatrix, 0, projectionMatrix, 0, viewMatrix, 0)
            //invalidate()

            return true
        }
    }
    private val mScaleDetector = ScaleGestureDetector(context, scaleListener)
    
    private val mGestureListener = object : GestureDetector.SimpleOnGestureListener() {
        override fun onScroll(
            e1: MotionEvent?,
            e2: MotionEvent?,
            distanceX: Float,
            distanceY: Float
        ): Boolean {
//            Log.i(DEBUG_TAG, "scroll distance X $distanceX, Y $distanceY")
//            Log.i(DEBUG_TAG, "scroll events: $e1 $e2")

            if (elapsedRealtime() - lastScaleTime < scaleLockTime) {
                return true
            }

            if (fingerNumber == 1) { //rotation
//                Log.i(DEBUG_TAG, "fingerNumber: $fingerNumber")
                val dx = -distanceX
                val dy = distanceY
                val scrollLength = 0.5f *sqrt(dx.pow(2) + dy.pow(2))

                val Tcp2 = camToPivot.clone()
                Tcp2[12] -= camXshift
                Tcp2[13] -= camYshift
                val t_cp2 = floatArrayOf(Tcp2[12], Tcp2[13], Tcp2[14])

                val Tcp2prime = Tcp2.clone()
                Tcp2prime[12] -= t_cp2[0]
                Tcp2prime[13] -= t_cp2[1]
                Tcp2prime[14] -= t_cp2[2]
                val rotMat = FloatArray(16)
                Matrix.setRotateM(rotMat,0,scrollLength, -dy, dx, 0f)
                Matrix.multiplyMM(Tcp2prime,0,rotMat,0,Tcp2prime,0)
                Tcp2prime[12] += t_cp2[0]
                Tcp2prime[13] += t_cp2[1]
                Tcp2prime[14] += t_cp2[2]

                val Tp1w = pivotToWorld.clone()
                val Tp2p1 = camToPivot.clone()
                Tp2p1[12] += camXshift
                Tp2p1[13] += camYshift
                val Tcp1_inverse = FloatArray(16)
                Matrix.invertM(Tcp1_inverse,0,camToPivot,0)
                Matrix.multiplyMM(Tp2p1, 0, Tcp1_inverse,0, Tp2p1,0)
                val Tp2w = FloatArray(16)
                Matrix.multiplyMM(Tp2w,0, Tp2p1,0, Tp1w,0)

                val Tcw_prime = FloatArray(16)
                Matrix.multiplyMM(Tcw_prime, 0, Tcp2prime, 0, Tp2w, 0)
                viewMatrix = Tcw_prime.clone()

                camToPivot = Tcp2prime.clone()
                pivotToWorld = Tp2w.clone()
                camXshift = 0f
                camYshift = 0f

                Matrix.multiplyMM(vPMatrix, 0, projectionMatrix, 0, viewMatrix, 0)
                fingerNumber = 0

            } else if (fingerNumber == 2) {//drag
//                Log.i(DEBUG_TAG, "fingerNumber: $fingerNumber")
                if (e2 != null) {
                    if (e2.eventTime - e2.downTime < 100)
                        return true
                }
//                return true //disable the drag function
                val factor = 0.03f
                val dx = -distanceX * factor
                val dy = distanceY * factor
//                Log.i(DEBUG_TAG,"dx = $dx, dy = $dy" )
                camToPivot[12] += dx
                camToPivot[13] += dy
                Matrix.multiplyMM(viewMatrix,0, camToPivot,0, pivotToWorld,0)
                camXshift += dx
                camYshift += dy

                Matrix.multiplyMM(vPMatrix, 0, projectionMatrix, 0, viewMatrix, 0)
                fingerNumber = 0

            }

            return true
        }

        override fun onDoubleTap(e: MotionEvent?): Boolean { //reset view
//            Log.i(DEBUG_TAG, "double tap.")
            viewMatrix = initViewMat.clone()
            Matrix.multiplyMM(vPMatrix, 0, projectionMatrix, 0, viewMatrix, 0)
            camXshift = 0f
            camYshift = 0f
            mScaleFactor = 1f
            camToPivot = viewMatrix.clone()
            Matrix.setIdentityM(pivotToWorld,0)
            return true
        }
    }
    private val mGestureDector = GestureDetector(context, mGestureListener)

    init {
        setEGLContextClientVersion(3)
        mapRenderer = MyGLRenderer()
        setRenderer(mapRenderer)
        //renderMode = RENDERMODE_WHEN_DIRTY
    }


    override fun onTouchEvent(event: MotionEvent): Boolean {
        /*The sequence of the || seperated terms matters.
        * Function will return immediently once current term returns true.*/
        //return mScaleDetector.onTouchEvent(event) || mGestureDector.onTouchEvent(event)
        fingerNumber = event.pointerCount
        mScaleDetector.onTouchEvent(event)
        mGestureDector.onTouchEvent(event)
        return true
    }

}


class MyGLRenderer : GLSurfaceView.Renderer {
    override fun onSurfaceCreated(unused: GL10?, config: EGLConfig?) {
        Log.i(DEBUG_TAG, "MyGLrenderer onSurfaceCreated")
        GLES30.glClearColor(1f, 1f, 1f, 1.0f)
        initMapDrawer()
    }

    override fun onDrawFrame(unused: GL10?) {
        drawFrameJNI(vPMatrix)
    }

    override fun onSurfaceChanged(unused: GL10?, width: Int, height: Int) {
        Log.i(DEBUG_TAG, "MyGLrenderer onSurfaceChanged")
        GLES30.glViewport(0, 0, width, height)
        val ratio: Float = width.toFloat() / height.toFloat()
        val tan_half_vectical_fov: Float = 1f
        val near_clip: Float = 0.1f
        val far_clip: Float = 1000f
        val t: Float = tan_half_vectical_fov*near_clip
        val l = ratio * t
        Matrix.frustumM(projectionMatrix, 0, -l, l, -t, t, near_clip, far_clip)
        Matrix.setLookAtM(initViewMat, 0, 0f, 0f, 4f, 0f,0f, 0f, 0f, 1f, 1f)
        viewMatrix = initViewMat.clone()
        Matrix.multiplyMM(vPMatrix, 0, projectionMatrix, 0, viewMatrix, 0)
        camToPivot = viewMatrix.clone()
        Matrix.setIdentityM(pivotToWorld,0)
    }

    external fun drawFrameJNI(vPMatrix : FloatArray)
    external fun initMapDrawer()
}

