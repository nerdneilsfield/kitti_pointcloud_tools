package io.github.nerdneilsfield.kpt.viewer

import java.nio.ByteBuffer
import java.util.concurrent.atomic.AtomicInteger
import org.junit.Assert.assertEquals
import org.junit.Test

class NativeCloudTest {
    @Test
    fun nativeHandleLivesUntilOwnerAndRendererLeaseRelease() {
        val releases = AtomicInteger(0)
        val cloud = NativeCloud(
            nativeHandle = 42L,
            metadata = CloudMetadata(
                pointCount = 1,
                bounds = floatArrayOf(0f, 0f, 0f, 1f, 1f, 1f),
                intensityMin = 0f,
                intensityMax = 1f,
                intensityP05 = 0f,
                intensityP90 = 1f,
                hasColor = false,
                hasIntensity = true,
                hasNoise = false,
            ),
            vertexBuffer = ByteBuffer.allocateDirect(20),
            releaseNative = { releases.incrementAndGet() },
        )

        val rendererLease = cloud.retain()
        cloud.close()
        cloud.close()
        assertEquals(0, releases.get())

        rendererLease.close()
        rendererLease.close()
        assertEquals(1, releases.get())
    }
}
