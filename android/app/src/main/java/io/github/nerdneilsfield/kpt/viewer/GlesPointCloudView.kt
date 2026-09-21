package io.github.nerdneilsfield.kpt.viewer

import android.content.Context
import android.opengl.GLES30
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.ViewConfiguration
import java.io.Closeable
import java.nio.ByteOrder
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.max

class GlesPointCloudView(context: Context) : GLSurfaceView(context), Closeable {
    private val pointRenderer = PointRenderer()
    private var displayedCloud: NativeCloud? = null
    private val scaleDetector = ScaleGestureDetector(
        context,
        object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
            override fun onScale(detector: ScaleGestureDetector): Boolean {
                pointRenderer.zoom(detector.scaleFactor)
                requestRender()
                return true
            }
        },
    )
    private var lastX = 0.0f
    private var lastY = 0.0f
    private var moved = false
    private val touchSlop = ViewConfiguration.get(context).scaledTouchSlop

    init {
        setEGLContextClientVersion(3)
        setRenderer(pointRenderer)
        renderMode = RENDERMODE_WHEN_DIRTY
        preserveEGLContextOnPause = true
    }

    fun setCloud(cloud: NativeCloud?) {
        if (displayedCloud === cloud) return
        displayedCloud = cloud
        pointRenderer.setCloud(cloud)
        requestRender()
    }

    fun setColorMode(mode: ColorMode) {
        pointRenderer.colorMode = mode
        requestRender()
    }

    fun setPointSize(size: Float) {
        pointRenderer.pointSize = size
        requestRender()
    }

    fun fit() {
        pointRenderer.requestFit()
        requestRender()
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        parent?.requestDisallowInterceptTouchEvent(true)
        scaleDetector.onTouchEvent(event)
        val x = if (event.pointerCount == 1) event.x else centroidX(event)
        val y = if (event.pointerCount == 1) event.y else centroidY(event)
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN, MotionEvent.ACTION_POINTER_DOWN -> {
                lastX = x
                lastY = y
                moved = false
            }
            MotionEvent.ACTION_MOVE -> {
                val dx = x - lastX
                val dy = y - lastY
                if (kotlin.math.abs(dx) > touchSlop || kotlin.math.abs(dy) > touchSlop) {
                    moved = true
                }
                if (event.pointerCount >= 2) {
                    pointRenderer.pan(dx, dy, width, height)
                } else if (!scaleDetector.isInProgress) {
                    pointRenderer.orbit(dx, dy, width, height)
                }
                lastX = x
                lastY = y
                requestRender()
            }
            MotionEvent.ACTION_UP -> {
                if (!moved) performClick()
                parent?.requestDisallowInterceptTouchEvent(false)
            }
            MotionEvent.ACTION_CANCEL ->
                parent?.requestDisallowInterceptTouchEvent(false)
        }
        return true
    }

    override fun performClick(): Boolean {
        super.performClick()
        return true
    }

    override fun close() {
        displayedCloud = null
        pointRenderer.clearPendingOnUiThread()
        queueEvent { pointRenderer.releaseOnGlThread() }
        requestRender()
    }

    private fun centroidX(event: MotionEvent): Float =
        (0 until event.pointerCount).sumOf { event.getX(it).toDouble() }.toFloat() /
            event.pointerCount

    private fun centroidY(event: MotionEvent): Float =
        (0 until event.pointerCount).sumOf { event.getY(it).toDouble() }.toFloat() /
            event.pointerCount

    private class PointRenderer : Renderer {
        private val cloudLock = Any()
        private var pendingLease: NativeCloud.Lease? = null
        private var pendingUpdate = false
        private var currentLease: NativeCloud.Lease? = null
        private var program = 0
        private var vertexArray = 0
        private var vertexBuffer = 0
        private var viewportWidth = 1
        private var viewportHeight = 1
        private var pointCount = 0
        @Volatile var colorMode: ColorMode = ColorMode.INTENSITY
        @Volatile var pointSize = 3.0f
        @Volatile private var yaw = 0.0f
        @Volatile private var pitch = 0.0f
        @Volatile private var panX = 0.0f
        @Volatile private var panY = 0.0f
        @Volatile private var distanceScale = 1.0f
        @Volatile private var fitRequested = true

        fun setCloud(cloud: NativeCloud?) {
            val replacement = cloud?.retain()
            synchronized(cloudLock) {
                pendingLease?.close()
                pendingLease = replacement
                pendingUpdate = true
            }
        }

        fun clearPendingOnUiThread() {
            synchronized(cloudLock) {
                pendingLease?.close()
                pendingLease = null
                pendingUpdate = true
            }
        }

        fun requestFit() {
            fitRequested = true
        }

        fun orbit(dx: Float, dy: Float, width: Int, height: Int) {
            yaw += dx / max(width, 1) * 180.0f
            pitch = (pitch + dy / max(height, 1) * 180.0f).coerceIn(-89.0f, 89.0f)
        }

        fun pan(dx: Float, dy: Float, width: Int, height: Int) {
            val radius = currentLease?.cloud?.metadata?.radius ?: 1.0f
            panX += dx / max(width, 1) * radius * 2.0f * distanceScale
            panY -= dy / max(height, 1) * radius * 2.0f * distanceScale
        }

        fun zoom(scaleFactor: Float) {
            if (scaleFactor.isFinite() && scaleFactor > 0.0f) {
                distanceScale = (distanceScale / scaleFactor).coerceIn(0.05f, 20.0f)
            }
        }

        override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
            GLES30.glClearColor(0.015f, 0.02f, 0.03f, 1.0f)
            GLES30.glEnable(GLES30.GL_DEPTH_TEST)
            program = createProgram(VERTEX_SHADER, FRAGMENT_SHADER)
            vertexArray = 0
            vertexBuffer = 0
            currentLease?.let(::upload)
            checkGl("surface creation")
        }

        override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
            viewportWidth = max(width, 1)
            viewportHeight = max(height, 1)
            GLES30.glViewport(0, 0, viewportWidth, viewportHeight)
        }

        override fun onDrawFrame(gl: GL10?) {
            applyPendingCloud()
            if (fitRequested) {
                yaw = 0.0f
                pitch = 0.0f
                panX = 0.0f
                panY = 0.0f
                distanceScale = 1.0f
                fitRequested = false
            }
            GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT or GLES30.GL_DEPTH_BUFFER_BIT)
            val lease = currentLease ?: return
            if (program == 0 || vertexArray == 0 || pointCount == 0) return

            val metadata = lease.cloud.metadata
            val model = FloatArray(16)
            val view = FloatArray(16)
            val projection = FloatArray(16)
            val modelView = FloatArray(16)
            val mvp = FloatArray(16)
            Matrix.setIdentityM(model, 0)
            Matrix.translateM(model, 0, panX, panY, 0.0f)
            Matrix.rotateM(model, 0, pitch, 1.0f, 0.0f, 0.0f)
            Matrix.rotateM(model, 0, yaw, 0.0f, 1.0f, 0.0f)
            val center = metadata.center
            Matrix.translateM(model, 0, -center[0], -center[1], -center[2])
            val radius = metadata.radius
            val distance = radius * 2.8f * distanceScale
            Matrix.setLookAtM(
                view, 0,
                0.0f, 0.0f, distance,
                0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
            )
            Matrix.perspectiveM(
                projection,
                0,
                45.0f,
                viewportWidth.toFloat() / viewportHeight.toFloat(),
                (radius * 0.005f).coerceAtLeast(0.0001f),
                (distance + radius * 8.0f).coerceAtLeast(1.0f),
            )
            Matrix.multiplyMM(modelView, 0, view, 0, model, 0)
            Matrix.multiplyMM(mvp, 0, projection, 0, modelView, 0)

            GLES30.glUseProgram(program)
            GLES30.glUniformMatrix4fv(
                GLES30.glGetUniformLocation(program, "uMvp"),
                1,
                false,
                mvp,
                0,
            )
            GLES30.glUniform1i(
                GLES30.glGetUniformLocation(program, "uColorMode"),
                colorMode.ordinal,
            )
            GLES30.glUniform1f(
                GLES30.glGetUniformLocation(program, "uPointSize"),
                pointSize,
            )
            GLES30.glUniform2f(
                GLES30.glGetUniformLocation(program, "uIntensityRange"),
                metadata.intensityP05,
                metadata.intensityP90,
            )
            GLES30.glUniform2f(
                GLES30.glGetUniformLocation(program, "uZRange"),
                metadata.bounds[2],
                metadata.bounds[5],
            )
            GLES30.glBindVertexArray(vertexArray)
            GLES30.glDrawArrays(GLES30.GL_POINTS, 0, pointCount)
            GLES30.glBindVertexArray(0)
            checkGl("frame render")
        }

        private fun applyPendingCloud() {
            val replacement: NativeCloud.Lease?
            synchronized(cloudLock) {
                if (!pendingUpdate) return
                replacement = pendingLease
                pendingLease = null
                pendingUpdate = false
            }
            deleteGpuBuffers()
            currentLease?.close()
            currentLease = replacement
            fitRequested = true
            replacement?.let(::upload)
        }

        private fun upload(lease: NativeCloud.Lease) {
            if (program == 0) return
            deleteGpuBuffers()
            val arrays = IntArray(1)
            val buffers = IntArray(1)
            GLES30.glGenVertexArrays(1, arrays, 0)
            GLES30.glGenBuffers(1, buffers, 0)
            vertexArray = arrays[0]
            vertexBuffer = buffers[0]
            GLES30.glBindVertexArray(vertexArray)
            GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, vertexBuffer)
            val data = lease.cloud.vertexBuffer.duplicate().order(ByteOrder.nativeOrder())
            data.position(0)
            GLES30.glBufferData(
                GLES30.GL_ARRAY_BUFFER,
                data.capacity(),
                data,
                GLES30.GL_STATIC_DRAW,
            )
            val stride = NativeCloud.VERTEX_STRIDE_BYTES.toInt()
            GLES30.glEnableVertexAttribArray(0)
            GLES30.glVertexAttribPointer(0, 3, GLES30.GL_FLOAT, false, stride, 0)
            GLES30.glEnableVertexAttribArray(1)
            GLES30.glVertexAttribPointer(1, 1, GLES30.GL_FLOAT, false, stride, 12)
            GLES30.glEnableVertexAttribArray(2)
            GLES30.glVertexAttribPointer(2, 4, GLES30.GL_UNSIGNED_BYTE, true, stride, 16)
            GLES30.glBindVertexArray(0)
            GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, 0)
            pointCount = lease.cloud.metadata.pointCount
            checkGl("vertex upload")
        }

        fun releaseOnGlThread() {
            deleteGpuBuffers()
            currentLease?.close()
            currentLease = null
            synchronized(cloudLock) {
                pendingLease?.close()
                pendingLease = null
                pendingUpdate = false
            }
            if (program != 0) GLES30.glDeleteProgram(program)
            program = 0
        }

        private fun deleteGpuBuffers() {
            if (vertexBuffer != 0) GLES30.glDeleteBuffers(1, intArrayOf(vertexBuffer), 0)
            if (vertexArray != 0) GLES30.glDeleteVertexArrays(1, intArrayOf(vertexArray), 0)
            vertexBuffer = 0
            vertexArray = 0
            pointCount = 0
        }

        private fun createProgram(vertexSource: String, fragmentSource: String): Int {
            val vertex = compileShader(GLES30.GL_VERTEX_SHADER, vertexSource)
            val fragment = compileShader(GLES30.GL_FRAGMENT_SHADER, fragmentSource)
            val result = GLES30.glCreateProgram()
            GLES30.glAttachShader(result, vertex)
            GLES30.glAttachShader(result, fragment)
            GLES30.glLinkProgram(result)
            val status = IntArray(1)
            GLES30.glGetProgramiv(result, GLES30.GL_LINK_STATUS, status, 0)
            GLES30.glDeleteShader(vertex)
            GLES30.glDeleteShader(fragment)
            check(status[0] == GLES30.GL_TRUE) {
                "GLES program link failed: ${GLES30.glGetProgramInfoLog(result)}"
            }
            return result
        }

        private fun compileShader(type: Int, source: String): Int {
            val shader = GLES30.glCreateShader(type)
            GLES30.glShaderSource(shader, source)
            GLES30.glCompileShader(shader)
            val status = IntArray(1)
            GLES30.glGetShaderiv(shader, GLES30.GL_COMPILE_STATUS, status, 0)
            check(status[0] == GLES30.GL_TRUE) {
                "GLES shader compile failed: ${GLES30.glGetShaderInfoLog(shader)}"
            }
            return shader
        }

        private fun checkGl(operation: String) {
            val error = GLES30.glGetError()
            check(error == GLES30.GL_NO_ERROR) { "$operation failed with GLES error $error" }
        }

        companion object {
            private const val VERTEX_SHADER = """
                #version 300 es
                precision highp float;
                layout(location = 0) in vec3 aPosition;
                layout(location = 1) in float aIntensity;
                layout(location = 2) in vec4 aColor;
                uniform mat4 uMvp;
                uniform int uColorMode;
                uniform float uPointSize;
                uniform vec2 uIntensityRange;
                uniform vec2 uZRange;
                out vec3 vColor;

                vec3 ramp(float value) {
                    float t = clamp(value, 0.0, 1.0);
                    return clamp(vec3(1.5 * t, 1.5 - abs(2.0 * t - 1.0) * 1.5, 1.5 * (1.0 - t)), 0.0, 1.0);
                }

                void main() {
                    gl_Position = uMvp * vec4(aPosition, 1.0);
                    gl_PointSize = uPointSize;
                    if (uColorMode == 0) {
                        vColor = aColor.rgb;
                    } else if (uColorMode == 1) {
                        float span = max(uIntensityRange.y - uIntensityRange.x, 0.000001);
                        vColor = ramp((aIntensity - uIntensityRange.x) / span);
                    } else if (uColorMode == 2) {
                        float span = max(uZRange.y - uZRange.x, 0.000001);
                        vColor = ramp((aPosition.z - uZRange.x) / span);
                    } else {
                        vColor = vec3(0.35, 0.82, 1.0);
                    }
                }
            """

            private const val FRAGMENT_SHADER = """
                #version 300 es
                precision mediump float;
                in vec3 vColor;
                out vec4 outColor;
                void main() {
                    outColor = vec4(vColor, 1.0);
                }
            """
        }
    }
}
