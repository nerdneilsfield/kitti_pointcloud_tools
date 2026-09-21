package io.github.nerdneilsfield.kpt.viewer

import android.os.ParcelFileDescriptor
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import java.io.File
import java.nio.ByteBuffer
import java.nio.ByteOrder
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class NativeBridgeInstrumentedTest {
    @Test
    fun decodesBinPcdAndPlyThroughFileDescriptorAbi() {
        val bin = ByteBuffer.allocate(16).order(ByteOrder.LITTLE_ENDIAN)
            .putFloat(1f).putFloat(2f).putFloat(3f).putFloat(4f).array()
        decode(bin, "sample.bin").use { cloud ->
            assertEquals(1, cloud.metadata.pointCount)
            assertTrue(cloud.metadata.hasIntensity)
            assertEquals(20, cloud.vertexBuffer.capacity())
            assertTrue(cloud.vertexBuffer.isDirect)
        }

        val pcd = (
            "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n" +
                "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n" +
                "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2 3 9\n"
            ).encodeToByteArray()
        decode(pcd, "sample.pcd").use { cloud ->
            assertEquals(1, cloud.metadata.pointCount)
            assertEquals(9f, cloud.metadata.intensityMax)
        }

        val ply = (
            "ply\nformat ascii 1.0\nelement vertex 1\n" +
                "property float x\nproperty float y\nproperty float z\n" +
                "property uchar red\nproperty uchar green\nproperty uchar blue\n" +
                "end_header\n1 2 3 10 20 30\n"
            ).encodeToByteArray()
        decode(ply, "sample.ply").use { cloud ->
            assertEquals(1, cloud.metadata.pointCount)
            assertTrue(cloud.metadata.hasColor)
        }
    }

    @Test
    fun cancellationMapsToStableErrorCode() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val file = File(context.cacheDir, "cancel.bin").apply {
            writeBytes(ByteArray(16))
        }
        val request = NativeBindings.createDecodeRequest()
        NativeBindings.cancelDecodeRequest(request)
        val result = ParcelFileDescriptor.open(file, ParcelFileDescriptor.MODE_READ_ONLY).use {
            NativeBindings.decodeFd(request, it.fd, file.name)
        }
        try {
            assertEquals(
                DecodeErrorCode.CANCELLED.nativeValue,
                NativeBindings.cloudErrorCode(result),
            )
            assertNotNull(NativeBindings.cloudErrorMessage(result))
        } finally {
            NativeBindings.releaseCloud(result)
            NativeBindings.releaseDecodeRequest(request)
        }
    }

    private fun decode(bytes: ByteArray, name: String): NativeCloud {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val file = File(context.cacheDir, name).apply { writeBytes(bytes) }
        val request = NativeBindings.createDecodeRequest()
        try {
            val result = ParcelFileDescriptor.open(file, ParcelFileDescriptor.MODE_READ_ONLY).use {
                NativeBindings.decodeFd(request, it.fd, name)
            }
            return NativeCloud.fromHandle(result)
        } finally {
            NativeBindings.releaseDecodeRequest(request)
        }
    }
}
