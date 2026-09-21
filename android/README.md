# KPT Android Viewer Spike

Experimental single-frame Android viewer for KITTI BIN, PCD, and PLY files.
The Compose UI owns document selection and lifecycle; `kpt_android` decodes on a
dedicated coroutine-backed executor; OpenGL ES 3 uploads one packed native
direct buffer.

## Toolchain

- JDK 21
- Android SDK 36
- Android NDK r29 (`29.0.14206865`)
- CMake 3.22.1
- arm64-v8a device running Android 10 or newer with OpenGL ES 3

From this directory:

```bash
./gradlew :app:assembleDebug
./gradlew :app:testDebugUnitTest
./gradlew :app:connectedDebugAndroidTest
```

The APK is written to `app/build/outputs/apk/debug/app-debug.apk`.

## Spike limits

- 64 MiB selected input
- 2,000,000 decoded points
- 64 MiB codec body
- BIN, PCD, and PLY only

The limits are applied inside the Android native target. Desktop products keep
their existing limits.

## Device evidence

Record device model, SoC, RAM, Android version, and GPU before measuring:

```bash
adb shell getprop ro.product.model
adb shell getprop ro.hardware
adb shell getprop ro.build.version.release
adb shell dumpsys meminfo io.github.nerdneilsfield.kpt.viewer
adb shell dumpsys gfxinfo io.github.nerdneilsfield.kpt.viewer framestats
```

Acceptance run:

1. Load representative 100k and 1M point clouds.
2. Orbit continuously for 10 seconds; compute p95 frame time from `gfxinfo` and
   require at most 33 ms for the 1M fixture.
3. Cancel a large decode and require native return within 500 ms.
4. Open and close a cloud 20 times; post-warm PSS growth must remain within
   20 MiB.
5. Rotate, background/foreground, and force GL context recreation; the current
   native buffer must upload again without another decode.

Go if all correctness checks pass and the reference device meets the frame,
cancellation, and memory gates. Otherwise retain the app as a diagnostic spike
and address the failed gate before expanding formats or adding Player support.
