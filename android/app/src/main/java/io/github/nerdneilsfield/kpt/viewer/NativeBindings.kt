package io.github.nerdneilsfield.kpt.viewer

import java.nio.ByteBuffer

internal object NativeBindings {
    init {
        System.loadLibrary("kpt_android")
    }

    external fun createDecodeRequest(): Long
    external fun decodeFd(requestHandle: Long, descriptor: Int, sourceName: String): Long
    external fun cancelDecodeRequest(requestHandle: Long)
    external fun releaseDecodeRequest(requestHandle: Long)
    external fun cloudErrorCode(cloudHandle: Long): Int
    external fun cloudErrorMessage(cloudHandle: Long): String
    external fun cloudPointCount(cloudHandle: Long): Long
    external fun cloudFlags(cloudHandle: Long): Int
    external fun cloudMetadata(cloudHandle: Long): FloatArray?
    external fun cloudVertexBuffer(cloudHandle: Long): ByteBuffer?
    external fun releaseCloud(cloudHandle: Long)
}

enum class DecodeErrorCode(val nativeValue: Int) {
    CANCELLED(1),
    UNSUPPORTED_FORMAT(2),
    INVALID_DATA(3),
    RESOURCE_LIMIT(4),
    IO(5),
    OUT_OF_MEMORY(6),
    INTERNAL(7);

    companion object {
        fun fromNative(value: Int): DecodeErrorCode =
            entries.firstOrNull { it.nativeValue == value } ?: INTERNAL
    }
}

class NativeDecodeException(
    val code: DecodeErrorCode,
    message: String,
) : Exception(message)
