# Viewport Media Export — Research Report

Date: 2026-11-08

Status: Research complete, decisions locked, independent code review
incorporated; design proposed in `viewport-media-export-design.md`

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
viewport texture; the base-class default returns "renderer does not support
RGBA capture". There are two native implementations, not three:

- OpenGL (`src/gui/backend/opengl/point_renderer.cpp:1618`) — `glReadPixels`
  from the viewport FBO flipped to top-left order, guarded by an
  expected-context check. The WebGL runtime constructs this same
  `OpenGLPointRenderer` (`src/gui/runtime/glfw_webgl_runtime.cpp:199`), so
  WebGL2 shares the single implementation.
- Metal (`src/gui/backend/metal/point_renderer.mm:901`) — reads the committed
  color texture; fails if the command buffer was never enqueued, waits if it
  is still in flight.

`ViewportSession::captureRgba()` (`src/gui/viewport/session.hpp:84`) forwards
the call and documents the threading rule: capture on the UI/render thread,
encode anywhere.

### 2.2 Deferred capture (Metal constraint)

`App::InspectionScreenshotRequest` (`src/gui/app.hpp:337`) arms a request with
`capture_after_viewport_frame = inspection_viewport_render_count_ + 1`. At the
top of the next `drawViewport()` (`src/gui/app.cpp:1989`),
`capturePendingInspectionScreenshot()` (`src/gui/app.cpp:3635`) captures —
before `ViewportSession::draw()` runs its `resize`. This guarantees the
captured frame belongs to a pass that `renderAndPresent()` already committed
(required by Metal, harmless elsewhere). The counter increments only when a
draw actually produces a texture.

### 2.3 CPU-side encode and file I/O

`kpt_viewport_capture` (`src/gui/viewport/capture.cpp`) converts RGBA8 to the
RGB layout of `ImageRGB8` and writes PNG atomically through
`kpt::writeImageAtomic` (stb_image_write, vendored in `third_party/stb`;
`writeImageAtomic` writes `<path>.tmp` and renames on success,
`src/kpt/render/render.cpp:594`). Encoding runs on a `JobSystem` worker with
progress reporting, `stop_token` cancellation, and a UI-thread completion
posted through `UiEvents::post` (not a JobSystem method). Cancellation
removes the `.tmp`; a process crash can leave the `.tmp` behind but never a
truncated file at the destination.

### 2.4 UI and tests

- Inspection panel → "Screenshot" collapsing header
  (`src/gui/app.cpp:1291`, `drawInspectionScreenshotControls` at
  `src/gui/app.cpp:1905`): output path picker, overwrite checkbox, save
  button. On native builds the button is disabled while the path is empty and
  the output must end in `.png`; the web build instead downloads a fixed
  `kpt-viewport.png` through the browser. A hotkey with default output naming
  is therefore new behavior, not an exposed existing one.
- `tests/viewport_capture_test.cpp` covers conversion and write behavior
  without a GPU; its target uses `kpt_enable_windows_utf8_manifest`.
- i18n keys live in `i18n/en.json` **and** `i18n/zh-CN.json` — both are read
  by `kpt_generate_i18n` (`cmake/i18n.cmake`), and a key missing from either
  fails configure.
- File-dialog targets are registered in three places: the `DialogTarget`
  enum (`src/gui/app.hpp:67`), per-target extension filters in `openDialog`
  (`src/gui/app.cpp:2382`), and `applyDialogResult`
  (`src/gui/app.cpp:2523`). `dialog_paths.*` only normalizes paths.

### 2.5 Frame cadence

The native runtime loop (`src/gui/runner.cpp:148`) is `pollEvents` →
`beginFrame` → `App::draw` → `renderAndPresent` on every iteration, and the
OpenGL runtime sets `glfwSwapInterval(1)` at initialization
(`src/gui/runtime/glfw_opengl_runtime.cpp:115`): the desktop viewport
re-renders continuously at display vsync (~60 Hz) whether or not input
arrives. The WebGL runtime has no swap interval; the browser drives frames
through rAF and throttles to a 250 ms timer after ~12 quiet frames
(`src/gui/web/pc_web.cc:147`), gated by `App::needsContinuousRedraw()`
(`src/gui/app.cpp:540`).

### 2.6 Hotkey handling pattern

`App::handleInspectionUndoRedo()` (`src/gui/app.cpp:3320`) shows the
established guard in full: `io.KeyCtrl` (the modifier for that binding),
`io.WantTextInput`, and `inspection_scene_.transactionActive()`. There is no
general modal-dialog guard in the codebase.

### 2.7 Renderer redraw semantics (critical for recording)

The single-cloud OpenGL path early-returns without encoding when the uploaded
revision and frame content are unchanged
(`src/gui/backend/opengl/point_renderer.cpp:1186`); the layered path has no
such early return. Both backends rebuild their FBO and invalidate the last
completed render only on extent change. Consequences:

- A recording must **not** gate captures on the render counter — a static
  scene never advances it. The last committed texture stays valid, so
  wall-clock capture re-reads it correctly.
- Freezing the take's extent keeps the FBO (and thus the capturable texture)
  stable for the whole recording.

## 3. Gap analysis

| Capability | Status |
|---|---|
| Capture current viewport frame (all backends) | Built |
| Deferred, Metal-safe capture timing | Built |
| PNG encode + atomic write off-thread | Built |
| Screenshot UI with path/overwrite | Built |
| Screenshot hotkey with default output naming | Missing — new behavior (native button requires a path today) |
| Multi-frame capture with wall-clock fps/duration pacing | Missing |
| GIF encoder | Missing — no GIF writer in vendored stb; gif-h not yet vendored |
| Video encoder | Missing |
| Bounded (frame, repeat) queue capture→encoder | Missing |
| Recording state machine + UI | Missing |
| JPEG-to-memory wrapper over vendored stb | Missing — `stb_png.cpp` owns `STB_IMAGE_WRITE_IMPLEMENTATION`; a sibling wrapper must not redefine it |

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
good at 8-bit palette depth. Its centisecond delay granularity requires
per-frame delay assignment (Bresenham) to keep total duration exact at
non-divisor fps values.

### 4.2 Video

| Option | Build cost | Runtime dependency | Playback reach | Verdict |
|---|---|---|---|---|
| Vendor FFmpeg libs (libavcodec/libavutil/libswscale) | ~1.5 MLOC, own build system, assembly paths | None | MP4/H.264 everywhere | Rejected: incompatible with the vendoring policy's "keep only what is needed" and with configure/build times |
| ffmpeg subprocess pipe (`-f rawvideo` → libx264 → .mp4) | Small: spawn + pipe helper in `src/platform` | ffmpeg binary on PATH | MP4/H.264 everywhere | Rejected for this iteration: the user chose a fully self-contained build; remains the documented future path to MP4 |
| **In-tree MJPEG AVI** (RIFF container + stb JPEG) | ~250 LOC new code, zero new deps (stb already vendored) | None | VLC/mpv/ffplay/Windows Media Player yes, browsers no | **Selected**: fully self-contained, matches the offline/vendoring posture |
| WebM/VP8 in-tree | Infeasible to write | None | Browsers | Rejected |

MJPEG AVI trades file size and browser playback for zero dependencies and a
tiny implementation. The RIFF AVI container is a well-documented, stable
format (`avih`/`strh` fourCC `MJPG`/`strf`/`movi` `00db` chunks/`idx1`); the
frame counts and index are backfilled at the end of a successful take. JPEG
frames come from the already-vendored stb_image_write — with one correction
found in review: **quality must exceed 90**, because stb enables 4:2:0 chroma
subsampling at `quality <= 90`
(`third_party/stb/stb_image_write.h:1479`) and edge-pads odd dimensions into
16×16 MCUs; at 91+ the encoder writes unsubsampled 4:4:4 and odd viewport
sizes are clean.

GIF and video converge on one pipeline — a frame source plus two encoder
backends — which matches the requirement that 2 and 3 are the same feature.

## 5. Constraints discovered

1. **Capture reads the previous committed frame**, and the capture point must
   stay at the top of `drawViewport`, before `ViewportSession::draw()` runs
   `resize` — mandatory on Metal, already correct on the other backends.
2. **Static scenes never advance the render counter** (single-cloud early
   return). Recording paces on wall clock and aborts only on a genuine
   `captureRgba()` failure.
3. **Resolution must stay fixed during a recording**, and the extent cannot
   be trusted frame to frame: it is computed as
   `available × framebuffer_scale × render_scale` and truncated to int
   (`src/gui/app.cpp:1979`), so scrollbars, DPI, or the 0.75 interaction
   scale can shift it by a pixel. Policy: compute once at take start (render
   scale pinned to 1.0), cache, and reuse for every frame — never recompute
   and compare.
4. **Backpressure without breaking frame count.** A 1920×1080 RGBA frame is
   ~8.3 MB. Dropping a frame when the queue is full permanently skews the
   `due = floor((now-start)*fps) - emitted` accounting, so the queue stores
   `(frame, repeat)` entries and merges due counts into the tail instead of
   dropping; output stays exactly `fps × duration` frames. Bound: 8 distinct
   frames (≈64 MiB at 1080p).
5. **Job priority.** During sequence playback the last JobSystem worker
   accepts only High-priority jobs (`src/gui/jobs/job_system.cpp:174`) and
   the default worker limit is half of `max_workers`; the export job must be
   submitted at `JobPriority::High` or it can starve while the queue fills.
6. **Hotkey availability.** The PrintScreen key is grabbed globally by most
   desktop environments (GNOME/KDE screenshot tools, Windows Snipping Tool)
   and never reaches the application. F12 is free on all native targets. The
   hotkey also needs a default output location, since the whole point is to
   skip the path entry — this default naming is new behavior, not an exposed
   existing path.
7. **Web build scope.** Browser rAF throttling, absence of native file paths,
   and the native-only encoder flow make the pipeline inapplicable. Web keeps
   the existing PNG download; web video export (MediaRecorder → WebM) is a
   separate future feature, out of scope here.
8. **AVI+MJPEG playback reach.** VLC, mpv, ffplay, and Windows Media Player
   play MJPEG AVI; browsers do not. Files are larger than H.264 (roughly
   90–240 MB worst case for the 60 s cap at 1080p/10 fps, typically far less
   on dark point-cloud scenes). Accepted per the self-contained decision.

## 6. Dependency policy check

- gif-h: vendored into `third_party/gif-h-<ref>/`, pinned to an identifiable
  commit, license text preserved. Complies with the vendoring policy.
- JPEG for MJPEG frames: reuses the already-vendored stb_image_write; the new
  wrapper lives beside `stb_png.cpp` and does not redefine
  `STB_IMAGE_WRITE_IMPLEMENTATION`.
- Result: the whole feature builds and runs offline with zero new external
  acquisition. No changes to existing vendored dependencies.

## 7. Conclusions

1. Feature 1 is mostly wiring: reuse the screenshot pipeline, add a hotkey
   (F12) and a default timestamped output path (new behavior on native).
2. Features 2 and 3 are one recorder core with two in-tree encoder backends:
   vendored gif-h for GIF, stb-JPEG-in-RIFF for AVI video. Both are
   self-contained; the offline-build invariant holds with no new external
   dependency of any kind.
3. The Metal-safe deferred-capture pattern, JobSystem, atomic write helper,
   and i18n/UI conventions already exist and are directly reused; the new
   code is the frame pacer, the bounded (frame, repeat) queue, two encoder
   adapters, and panel UI.
4. Review corrected three load-bearing details that the design now encodes:
   wall-clock capture instead of render-counter gating (static scenes),
   repeat-merging instead of frame dropping (exact frame count), and JPEG
   quality ≥91 (no chroma subsampling on odd viewport sizes).
5. If browser-playable MP4/H.264 is ever required, the documented path is an
   optional ffmpeg subprocess encoder behind the same `RecordingEncoder`
   interface — an addition, not a redesign.
