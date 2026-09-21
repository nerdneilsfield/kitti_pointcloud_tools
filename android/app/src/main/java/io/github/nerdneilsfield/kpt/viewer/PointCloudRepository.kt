package io.github.nerdneilsfield.kpt.viewer

import android.content.ContentResolver
import android.content.Context
import android.net.Uri
import android.provider.OpenableColumns
import java.io.Closeable
import java.util.concurrent.Executors
import kotlin.coroutines.resume
import kotlin.coroutines.resumeWithException
import kotlinx.coroutines.suspendCancellableCoroutine

class PointCloudRepository(context: Context) : Closeable {
    private val resolver: ContentResolver = context.applicationContext.contentResolver
    private val executor = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "kpt-native-decode").apply { isDaemon = true }
    }

    suspend fun decode(uri: Uri): NativeCloud = suspendCancellableCoroutine { continuation ->
        val requestHandle = NativeBindings.createDecodeRequest()
        if (requestHandle == 0L) {
            continuation.resumeWithException(
                NativeDecodeException(DecodeErrorCode.OUT_OF_MEMORY, "cannot create decode request"),
            )
            return@suspendCancellableCoroutine
        }

        continuation.invokeOnCancellation {
            NativeBindings.cancelDecodeRequest(requestHandle)
        }

        executor.execute {
            try {
                val sourceName = displayName(uri)
                val cloud = resolver.openFileDescriptor(uri, "r")?.use { descriptor ->
                    val resultHandle = NativeBindings.decodeFd(
                        requestHandle,
                        descriptor.fd,
                        sourceName,
                    )
                    NativeCloud.fromHandle(resultHandle)
                } ?: throw NativeDecodeException(DecodeErrorCode.IO, "cannot open selected document")

                continuation.resume(cloud) { _, cancelledCloud, _ ->
                    cancelledCloud.close()
                }
            } catch (error: Throwable) {
                continuation.resumeWith(Result.failure(error))
            } finally {
                NativeBindings.releaseDecodeRequest(requestHandle)
            }
        }
    }

    private fun displayName(uri: Uri): String {
        resolver.query(uri, arrayOf(OpenableColumns.DISPLAY_NAME), null, null, null)?.use { cursor ->
            if (cursor.moveToFirst()) {
                val index = cursor.getColumnIndex(OpenableColumns.DISPLAY_NAME)
                if (index >= 0) {
                    cursor.getString(index)?.takeIf { it.isNotBlank() }?.let { return it }
                }
            }
        }
        return uri.lastPathSegment?.substringAfterLast('/')?.takeIf { it.isNotBlank() }
            ?: "selected.cloud"
    }

    override fun close() {
        executor.shutdownNow()
    }
}
