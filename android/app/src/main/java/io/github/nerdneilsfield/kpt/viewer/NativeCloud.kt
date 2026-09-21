package io.github.nerdneilsfield.kpt.viewer

import java.io.Closeable
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

data class CloudMetadata(
    val pointCount: Int,
    val bounds: FloatArray,
    val intensityMin: Float,
    val intensityMax: Float,
    val intensityP05: Float,
    val intensityP90: Float,
    val hasColor: Boolean,
    val hasIntensity: Boolean,
    val hasNoise: Boolean,
) {
    val center: FloatArray
        get() = floatArrayOf(
            (bounds[0] + bounds[3]) * 0.5f,
            (bounds[1] + bounds[4]) * 0.5f,
            (bounds[2] + bounds[5]) * 0.5f,
        )

    val radius: Float
        get() {
            val x = bounds[3] - bounds[0]
            val y = bounds[4] - bounds[1]
            val z = bounds[5] - bounds[2]
            return (kotlin.math.sqrt(x * x + y * y + z * z) * 0.5f).coerceAtLeast(0.001f)
        }
}

class NativeCloud internal constructor(
    private val nativeHandle: Long,
    val metadata: CloudMetadata,
    vertexBuffer: ByteBuffer,
    private val releaseNative: (Long) -> Unit = NativeBindings::releaseCloud,
) : Closeable {
    val vertexBuffer: ByteBuffer = vertexBuffer
        .order(ByteOrder.nativeOrder())
        .asReadOnlyBuffer()
        .order(ByteOrder.nativeOrder())

    private val references = AtomicInteger(1)
    private val ownerClosed = AtomicBoolean(false)

    fun retain(): Lease {
        while (true) {
            val current = references.get()
            check(current > 0) { "native cloud is already closed" }
            if (references.compareAndSet(current, current + 1)) {
                return Lease(this)
            }
        }
    }

    override fun close() {
        if (ownerClosed.compareAndSet(false, true)) {
            releaseReference()
        }
    }

    private fun releaseReference() {
        val remaining = references.decrementAndGet()
        check(remaining >= 0) { "native cloud released more than retained" }
        if (remaining == 0) {
            releaseNative(nativeHandle)
        }
    }

    class Lease internal constructor(val cloud: NativeCloud) : Closeable {
        private val closed = AtomicBoolean(false)

        override fun close() {
            if (closed.compareAndSet(false, true)) {
                cloud.releaseReference()
            }
        }
    }

    internal companion object {
        fun fromHandle(handle: Long): NativeCloud {
            if (handle == 0L) {
                throw NativeDecodeException(
                    DecodeErrorCode.OUT_OF_MEMORY,
                    "native result allocation failed",
                )
            }
            val error = NativeBindings.cloudErrorCode(handle)
            if (error != 0) {
                val message = NativeBindings.cloudErrorMessage(handle)
                NativeBindings.releaseCloud(handle)
                throw NativeDecodeException(DecodeErrorCode.fromNative(error), message)
            }
            val count = NativeBindings.cloudPointCount(handle)
            val values = NativeBindings.cloudMetadata(handle)
            val buffer = NativeBindings.cloudVertexBuffer(handle)
            if (count <= 0L || count > 2_000_000L || values == null || values.size != 10 || buffer == null) {
                NativeBindings.releaseCloud(handle)
                throw NativeDecodeException(DecodeErrorCode.INTERNAL, "native cloud ABI is invalid")
            }
            val expectedBytes = count * VERTEX_STRIDE_BYTES
            if (buffer.capacity().toLong() != expectedBytes) {
                NativeBindings.releaseCloud(handle)
                throw NativeDecodeException(DecodeErrorCode.INTERNAL, "native vertex buffer size is invalid")
            }
            val flags = NativeBindings.cloudFlags(handle)
            return NativeCloud(
                nativeHandle = handle,
                metadata = CloudMetadata(
                    pointCount = count.toInt(),
                    bounds = values.copyOfRange(0, 6),
                    intensityMin = values[6],
                    intensityMax = values[7],
                    intensityP05 = values[8],
                    intensityP90 = values[9],
                    hasColor = flags and 1 != 0,
                    hasIntensity = flags and 2 != 0,
                    hasNoise = flags and 4 != 0,
                ),
                vertexBuffer = buffer,
            )
        }

        const val VERTEX_STRIDE_BYTES = 20L
    }
}
