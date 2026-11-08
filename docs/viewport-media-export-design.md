# Viewport Media Export — Design

Date: 2026-11-08

Status: Proposed

Companion research: `viewport-media-export-research.md`

## 1. Goals and non-goals

### Goals

- **G1** F12 captures the current main viewport to a PNG immediately, without
  visiting the Inspection panel or entering a path.
- **G2** The Inspection panel gains a **Recording** section that exports the
  current viewpoint as an animated GIF with configurable fps and duration.
- **G3** The same section exports AVI video (MJPEG) with the same
  fps/duration settings, encoded fully in-tree — no external tools, no new
  heavyweight dependencies.
- **G4** Recording works while the user orbits the camera or the sequence
  player advances — whatever the viewport shows is what gets recorded.
- **G5** Linux (OpenGL), macOS (Metal), and Windows (OpenGL) behave
  identically; the offline-build invariant is preserved and the result is
  fully self-contained.

### Non-goals

- Web/WebGL video or GIF export (web keeps the existing PNG download).
- Auto-orbit / camera-path animation; the recorder captures the live view.
- Trajectory viewport recording.
- MP4/H.264 output and vendored FFmpeg libraries (future option, requires an
  external ffmpeg or a codec dependency; rejected for this iteration in favor
  of a self-contained build).
- Persisting export settings across sessions (session-only defaults).

### Locked decisions (2026-11-08)

1. **Self-contained video**: MJPEG-in-AVI written in-tree via the already
   vendored stb_image_write JPEG encoder. No ffmpeg dependency, neither at
   build time nor at run time.
2. **Hotkey**: F12. PrintScreen is deliberately not the binding: desktop
   environments grab it globally and the application never sees it.
3. **Defaults**: 10 fps and 3 s for both GIF and AVI, matching typical
   point-cloud acquisition rates (~10 Hz).

## 2. User experience

### 2.1 Screenshot hotkey

- **F12** saves the current main viewport as PNG.
- Output path resolution order:
  1. The path in the Inspection → Screenshot field, if set (the hotkey then
     appends `-HHMMSS` before `.png` so repeated presses never silently
     overwrite; the overwrite checkbox is ignored on the hotkey path).
  2. Otherwise `<cwd>/kpt-viewport-YYYYMMDD-HHMMSS.png`.
- The saved path is written to the log panel. Failure (capture error, file
  exists, I/O error) also lands in the log.
- Guard follows `handleInspectionUndoRedo`: ignored while
  `io.WantTextInput` is set (typing in a field) or a modal dialog is open.

### 2.2 Recording UI (Inspection panel → new "Recording" header)

Shown when the scene has layers, next to the existing Screenshot header:

- **Format**: combo `GIF` / `AVI (MJPEG)`.
- **FPS**: slider/input 1–60, default 10.
- **Duration**: seconds, 1–60, default 3.
- **Output**: path input + `...` dialog (existing `pathInput` /
  `DialogTarget` machinery, new target `InspectionRecordingOutput`);
  extension enforced to match format (`.gif` / `.avi`).
- **Overwrite** checkbox, same semantics as screenshot.
- **Record / Cancel** button. During recording it becomes **Cancel** and a
  countdown (`3.2 s left, 96 frames`) is drawn next to it.
- Completion/failure arrives through the existing JobSystem job list and log.

Recording starts on the button press; the recorder captures whatever the
viewport shows — the user can orbit or play a sequence during the take.
Recording ends at the configured duration, on Cancel, or on a fatal capture
error. A viewport **resize aborts** the recording with a clear log message
(encoders need constant frame size).

### 2.3 i18n

New keys under `gui.inspection.*` in `i18n/en.json` and `i18n/zh-CN.json`
(recording, format, fps, duration, record, cancel, progress/completion
messages). No new i18n machinery.

## 3. Architecture

```
UI/render thread                          JobSystem worker
----------------                          ----------------
App::drawViewport
  └─ RecordingSession::tick(frame#)       ExportJob (1 job per recording)
       ├─ due? (wall-clock pacer)              └─ Encoder (one of)
       ├─ captureRgba()  ── bounded queue ──→    ├─ GifEncoder      (gif-h)
       └─ resize? → abort                        └─ AviMjpegEncoder (stb JPEG)
```

### 3.1 RecordingSession (UI thread, lives in `App`)

State machine: `Idle → Recording → Finalizing → Idle`.

- `start(config)`: validates config, snapshots the current viewport extent as
  the locked frame size, submits the ExportJob, records
  `start_frame = inspection_viewport_render_count_`.
- `tick()`: called from `drawViewport` exactly where the screenshot deferral
  fires (top of the draw, previous committed frame — Metal-safe). On each
  call the wall-clock pacer decides how many output frames are due:
  - `due = floor((now - start) * fps) - emitted`
  - capture once per display frame; emit the capture for the first due slot,
    duplicate the previous frame for any remaining slots (constant frame
    rate). If nothing is due, return.
  - if `extent != locked_extent` → abort with "viewport resized".
- `stop(reason)`: closes the frame queue; state becomes `Finalizing` until
  the job's UI completion fires.
- While active, `needsContinuousRedraw()` returns true (web throttling) and
  `drawViewport` pins `render_scale` to 1.0 (no interaction LOD mid-take).

### 3.2 Frame queue

Bounded (8 slots) `std::deque<Rgba8Image>` + mutex + condition variable +
`stop_token`. Capture never blocks the render loop: when full, the new frame
is dropped and the previous frame's emission count grows by one (timing stays
correct). Dropped-frame count is reported in the completion log.

### 3.3 Encoders (worker side, `src/gui/viewport/`)

Common shape:

```cpp
class RecordingEncoder {
public:
  virtual ~RecordingEncoder() = default;
  virtual Result<void> begin(PixelExtent extent, int fps) = 0;
  virtual Result<void> writeFrame(const Rgba8Image &frame,
                                  unsigned repeat) = 0;  // repeat >= 1
  virtual Result<void> finish() = 0;  // flush + close; atomic rename
};
```

Both encoders write to `<path>.tmp` and rename through the existing
atomic-replace helper on success, so a cancelled or crashed export never
leaves a truncated file at the destination.

- **GifEncoder** — wraps vendored gif-h. `writeFrame` calls `GifWriteFrame`
  `repeat` times with delay `100/fps` centiseconds; sub-10 ms rounding error
  is distributed across frames (Bresenham-style) so total duration stays
  exact.
- **AviMjpegEncoder** — writes a RIFF AVI with MJPEG compression:
  - Container: `RIFF 'AVI '` with `hdrl` (`avih` + video `strl` with `strh`
    fourCC `MJPG` and `strf` BITMAPINFOHEADER), `movi` list of `00db` JPEG
    chunks, and a final `idx1` index. No OpenDML — recordings are far below
    the 2 GiB legacy limit.
  - Frames: RGBA8 → RGB, then `stbi_write_jpg_to_func` at quality 90 into a
    memory buffer; `repeat` writes the same JPEG chunk again (cheap
    duplication) and appends an `idx1` entry per written chunk.
  - MJPEG accepts arbitrary width/height — no even-dimension padding.
  - Size estimate: 1080p quality-90 JPEG ≈ 150–400 KB per frame; a 10 fps,
    60 s maximum take lands at roughly 90–240 MB worst case, typically far
    less on dark point-cloud scenes. Bounded by the 60 s duration cap.

### 3.4 Files

| Change | Location |
|---|---|
| Recording session + pacer (pure logic, testable) | `src/gui/viewport/recording.hpp/.cpp` (new target `kpt_viewport_recording`) |
| GIF encoder | `src/gui/viewport/gif_encoder.cpp` (same target) |
| AVI/MJPEG encoder | `src/gui/viewport/avi_mjpeg_encoder.cpp` (same target) |
| Vendored gif-h | `third_party/gif-h-<pinned-commit>/gif.h` + license |
| App wiring: F12 hotkey, Recording UI, `tick` call, `needsContinuousRedraw`, render-scale pin | `src/gui/app.hpp/.cpp` |
| New dialog target | `src/gui/dialog_paths.*`, `App::DialogTarget` |
| i18n keys | `i18n/en.json`, `i18n/zh-CN.json` |
| CMake: `kpt_viewport_recording` target, gif-h include dir | `CMakeLists.txt`, `third_party/CMakeLists.txt` |

No new build-time or run-time external dependencies. JPEG encoding reuses the
vendored stb_image_write already linked through `kpt_render_core`; the JPEG
entry point is exposed next to the existing PNG wrapper
(`src/kpt/render/detail/`).

## 4. Timing and correctness rules

1. **Sampling clock** is `std::chrono::steady_clock`; `start` is taken at the
   first `tick`, not at button press, so the first emitted frame is the one
   the user saw when pressing Record.
2. **Frame count** is exactly `fps × duration`; the last tick pads with
   duplicates of the final capture if the display loop is slow.
3. **fps > display rate** works: every due slot re-emits the latest capture.
   A warning is logged once ("target fps exceeds display rate").
4. **Delay rounding (GIF)**: centisecond delays lose sub-10 ms precision;
   cumulative error is corrected by distributing the remainder
   (Bresenham-style) so total duration stays exact.
5. **Cancellation** (`stop_token` from JobSystem) closes the queue, aborts
   the encoder, and removes the partial `.tmp` file. The UI stays responsive
   throughout.
6. **One recording at a time**; the screenshot hotkey remains usable during a
   recording (independent single-frame path).

## 5. Backend notes

- **OpenGL (Linux/Windows)**: `captureRgba` binds the viewport FBO and does
  `glReadPixels`; per-frame capture at 60 Hz is acceptable for viewport-sized
  targets. No changes to the renderer.
- **Metal (macOS)**: the deferred-by-one-frame capture pattern already proven
  by screenshots is reused unchanged per recorded frame; no new
  synchronization.
- **WebGL**: out of scope (section 1); `RecordingSession` is compiled out
  under `KPT_WEB_BUILD`, the panel header is hidden, and the hotkey is not
  installed.

## 6. Testing and verification

Follows the existing per-component test-executable convention (Catch2,
`CATCH_CONFIG_NO_COUNTER`, no POSIX signals):

1. **Pacer unit tests** (`tests/viewport_recording_test.cpp` against
   `kpt_viewport_recording`): due-count math, duplicate insertion, fps above
   display rate, GIF delay remainder distribution, exact total frame count.
2. **GIF encoder smoke test**: encode two synthetic 64×64 frames, assert
   `GIF89a` magic, trailer byte, and non-trivial size.
3. **AVI encoder unit test**: encode two synthetic frames, then parse the
   output in the test: RIFF/AVI magic, `avih` frame rate and frame count,
   `strh` fourCC `MJPG`, `movi` chunk count, `idx1` entry count and offsets.
   Decode one embedded JPEG frame with vendored `stb_image` to confirm it
   round-trips.
4. **Manual end-to-end** (the checks that matter):
   - Linux/X11: F12 → PNG appears; record 3 s GIF while orbiting → plays in a
     browser; record AVI → `ffplay`/VLC plays, reported fps and duration
     match the configuration.
   - macOS Metal: same three, confirming no capture-stale-frame artifacts.
   - Windows OpenGL: same three, including an output path with non-ASCII
     characters.
5. CI: existing `kpt_gui_smoke` under xvfb is extended only if it stays
   non-flaky; otherwise component tests carry the weight.

## 7. Risks and mitigations

| Risk | Mitigation |
|---|---|
| `glReadPixels`/Metal readback stalls the render loop at high fps | Capture is viewport-sized; the queue decouples encoding; dropped-frame accounting makes stalls visible rather than silent |
| User resizes window mid-recording | Abort with explicit log; documented behavior |
| AVI+MJPEG does not play in browsers and is larger than H.264 | Accepted per the self-contained decision; VLC/mpv/ffplay/Windows Media Player all play it; duration capped at 60 s bounds file size |
| GIF palette banding on gradients | gif-h 8-bit palette; acceptable for point clouds on dark background; dither left off by default for speed, toggle only if complaints appear |
| Partial file after crash/cancel | Encode to `<path>.tmp`, atomic rename on success (reuses `atomic_replace` helpers) |
