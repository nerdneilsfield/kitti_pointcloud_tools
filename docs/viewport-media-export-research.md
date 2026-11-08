# Viewport Media Export — Research Report

Date: 2026-11-08

Status: Research complete, decisions locked, design proposed in
`viewport-media-export-design.md`

## 1. Requirements

1. **Screenshot hotkey**: a key press captures the currently displayed GUI
   point-cloud viewport to an image file, without filling in a path first.
2. **GIF export**: export the current viewpoint as an animated GIF with
   configurable frame rate and duration.
3. **Video export**: export the current viewpoint as a video file with
   configurable frame rate and duration.

Requirements 2 and 3 share one frame-capture pipeline and differ only in the
encoder. This report surveys what already exists, evaluates encoder options,
and identifies the constraints that shape the design.

## 2. What the code already provides

The single-frame screenshot path is fully built and shipped. Video/GIF export
reuses it rather than inventing a parallel mechanism.

### 2.1 Renderer capture contract

`ViewportRenderer::captureRgba()` (`src/gui/viewport/renderer.hpp:62`) returns
CPU-owned top-left RGBA8 pixels of the most recently rendered offscreen
viewport texture. All three concrete backends implement it:

- OpenGL (`src/gui/backend/opengl/point_renderer.cpp:1618`) — `glReadPixels`
  from the viewport FBO, guarded by an expected-context check.
- Metal (`src/gui/backend/metal/point_renderer.mm:901`) — reads the committed
  color texture; must not be called while the runtime-owned command buffer is
  in flight.
- WebGL — implemented for the web PNG download flow.

`ViewportSession::captureRgba()` (`src/gui/viewport/session.hpp:84`) forwards
the call and documents the threading rule: capture on the UI/render thread,
encode anywhere.

### 2.2 Deferred capture (Metal constraint)

`App::InspectionScreenshotRequest` (`src/gui/app.hpp:337`) arms a request with
`capture_after_viewport_frame = inspection_viewport_render_count_ + 1`. At the
top of the next `drawViewport()` (`src/gui/app.cpp:1989`),
`capturePendingInspectionScreenshot()` (`src/gui/app.cpp:3635`) captures. This
guarantees the captured frame belongs to a pass that `renderAndPresent()`
already committed — required by Metal, harmless for OpenGL/WebGL.

### 2.3 CPU-side encode and file I/O

`kpt_viewport_capture` (`src/gui/viewport/capture.cpp`) converts RGBA8 to the
RGB layout of `ImageRGB8` and writes PNG atomically through
`kpt::writeImageAtomic` (stb_image_write, vendored in `third_party/stb`).
Encoding runs on a `JobSystem` worker with progress reporting, `stop_token`
cancellation, and a UI-thread completion post (`ui_.post`). Atomic write means
a cancelled or crashed encode never leaves a truncated file.

### 2.4 UI and tests

- Inspection panel → "Screenshot" collapsing header
  (`src/gui/app.cpp:1291`, `drawInspectionScreenshotControls` at
  `src/gui/app.cpp:1905`): output path picker, overwrite checkbox, save
  button; web build downloads `kpt-viewport.png` through the browser.
- `tests/viewport_capture_test.cpp` covers conversion and write behavior
  without a GPU.
- i18n keys live in `i18n/en.json` / `i18n/zh-CN.json` and are compiled into
  the binary by `kpt_generate_i18n` (`cmake/i18n.cmake`).

### 2.5 Frame cadence

The native runtime loop (`src/gui/runner.cpp:148`) is `glfwPollEvents()` +
`glfwSwapInterval(1)`: the viewport re-renders continuously at display vsync
(~60 Hz), whether or not input arrives. A recorder can therefore sample frames
on a wall-clock schedule. The web build throttles when quiet; recording must
force `App::needsContinuousRedraw()` to return true.

### 2.6 Hotkey handling pattern

`App::handleInspectionUndoRedo()` (`src/gui/app.cpp:3320`) shows the
established guard: check `io.WantTextInput` (and modifiers) before
`ImGui::IsKeyPressed`. New hotkeys follow the same pattern.

## 3. Gap analysis

| Capability | Status |
|---|---|
| Capture current viewport frame (all backends) | Built |
| Deferred, Metal-safe capture timing | Built |
| PNG encode + atomic write off-thread | Built |
| Screenshot UI with path/overwrite | Built |
| Screenshot hotkey (no path entry) | Missing — needs default output naming |
| Multi-frame capture with fps/duration pacing | Missing |
| GIF encoder | Missing — no GIF writer in vendored stb |
| Video encoder | Missing |
| Bounded frame queue capture→encoder | Missing |
| Recording state machine + UI | Missing |

## 4. Encoder options

### 4.1 GIF

| Option | Size | Offline build | Notes |
|---|---|---|---|
| Vendor [gif-h](https://github.com/charlietangora/gif-h) | 1 header, ~1.1 kLOC, public domain | Yes | Incremental `GifBegin`/`GifWriteFrame`/`GifEnd`, RGBA8 input, per-frame delay in centiseconds, delta encoding, optional Floyd–Steinberg dithering |
| ffmpeg subprocess (`palettegen`/`paletteuse`) | 0 in-tree | Yes, but needs runtime ffmpeg | Two-pass palette flow is awkward over one pipe; adds a runtime dependency for a format we can write natively |
| Hand-written GIF89a+LZW | ~400–600 LOC new code | Yes | Reinvents what gif-h already is; no benefit |

gif-h is the choice: single header, public domain, incremental streaming API
(bounded memory), and its input format (8-bit RGBA) is exactly what
`captureRgba()` returns. GIF quality for point clouds on dark backgrounds is
good at 8-bit palette depth.

### 4.2 Video

| Option | Build cost | Runtime dependency | Playback reach | Verdict |
|---|---|---|---|---|
| Vendor FFmpeg libs (libavcodec/libavutil/libswscale) | ~1.5 MLOC, own build system, assembly paths | None | MP4/H.264 everywhere | Rejected: incompatible with the vendoring policy's "keep only what is needed" and with configure/build times |
| ffmpeg subprocess pipe (`-f rawvideo` → libx264 → .mp4) | Small: spawn + pipe helper in `src/platform` | ffmpeg binary on PATH | MP4/H.264 everywhere | Rejected for this iteration: the user chose a fully self-contained build; remains the documented future path to MP4 |
| **In-tree MJPEG AVI** (RIFF container + stb JPEG) | ~250 LOC new code, zero new deps (stb already vendored) | None | VLC/mpv/ffplay/Windows Media Player yes, browsers no | **Selected**: fully self-contained, matches the offline/vendoring posture |
| WebM/VP8 in-tree | Infeasible to write | None | Browsers | Rejected |

MJPEG AVI trades file size and browser playback for zero dependencies and a
tiny implementation. The RIFF AVI container is a well-documented, stable
format; JPEG frames come from the already-vendored stb_image_write. For
viewport recordings of a few seconds at ~10 fps this is entirely adequate,
and every supported platform builds it with no additional acquisition.

GIF and video converge on one pipeline — a frame source plus two encoder
backends — which matches the requirement that 2 and 3 are the same feature.

## 5. Constraints discovered

1. **Capture reads the previous committed frame.** Reuse the existing
   `capture_after_viewport_frame` deferral for every recorded frame; this is
   mandatory on Metal and already correct on the other backends.
2. **Resolution must stay fixed during a recording.** Encoders require a
   constant frame size. Two hazards exist:
   - Window/panel resize changes the viewport extent mid-recording.
     Policy: abort the recording with a clear log message.
   - `drawViewport` drops to `interaction_render_scale = 0.75`
     (`src/gui/app.cpp:1974`) while interacting. Recording must pin the render
     scale to 1.0, or orbit-during-record would silently change frame size.
3. **Backpressure.** A 1920×1080 RGBA frame is ~8.3 MB; at 60 fps that is
   ~500 MB/s of raw pixels. The capture thread must not block on a slow
   encoder. Policy: bounded queue (≈8 frames); when full, drop the new frame
   and extend the previous frame's duration (constant-frame-rate output stays
   correct). Report dropped frames on completion.
4. **Constant frame rate output.** Display cadence (vsync) and target fps
   rarely divide evenly. The pacer samples on wall-clock time and duplicates
   the previous frame when the display is slower than the target rate, so the
   encoder always receives exactly `fps × duration` frames.
5. **Hotkey availability.** The PrintScreen key is grabbed globally by most
   desktop environments (GNOME/KDE screenshot tools, Windows Snipping Tool)
   and never reaches the application. F12 is free on all native targets. The
   hotkey also needs a default output location, since the whole point is to
   skip the path entry.
6. **Web build scope.** Browser rAF throttling, absence of native file paths,
   and the native-only encoder flow make the pipeline inapplicable. Web keeps
   the existing PNG download; web video export (MediaRecorder → WebM) is a
   separate future feature, out of scope here.
7. **AVI+MJPEG playback reach.** VLC, mpv, ffplay, and Windows Media Player
   play MJPEG AVI; browsers do not. Files are larger than H.264 (roughly
   90–240 MB worst case for the 60 s cap at 1080p/10 fps, typically far less
   on dark point-cloud scenes). Accepted per the self-contained decision.

## 6. Dependency policy check

- gif-h: vendored into `third_party/gif-h-<ref>/`, pinned to an identifiable
  commit, license text preserved. Complies with the vendoring policy.
- JPEG for MJPEG frames: reuses the already-vendored stb_image_write; no new
  dependency at all.
- Result: the whole feature builds and runs offline with zero new external
  acquisition. No changes to existing vendored dependencies.

## 7. Conclusions

1. Feature 1 is mostly wiring: reuse the screenshot pipeline, add a hotkey
   (F12) and a default timestamped output path.
2. Features 2 and 3 are one recorder core with two in-tree encoder backends:
   vendored gif-h for GIF, stb-JPEG-in-RIFF for AVI video. Both are
   self-contained; the offline-build invariant holds with no new external
   dependency of any kind.
3. The Metal-safe deferred-capture pattern, JobSystem, atomic write helper,
   and i18n/UI conventions already exist and are directly reused; the new
   code is the frame pacer, the bounded queue, two encoder adapters, and
   panel UI.
4. If browser-playable MP4/H.264 is ever required, the documented path is an
   optional ffmpeg subprocess encoder behind the same `RecordingEncoder`
   interface — an addition, not a redesign.
