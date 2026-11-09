# Viewport Media Export — Design

Date: 2026-11-08

Status: Proposed (revised after independent code review)

Companion research: `viewport-media-export-research.md`

## 1. Goals and non-goals

### Goals

- **G1** F12 captures the current main viewport to a PNG immediately, without
  visiting the Inspection panel or entering a path. Ctrl+Shift+S and an
  "Export snapshot" button in the Screenshot section trigger the same capture:
  F12 is a media/function key by default on macOS, so the hotkey needs an
  alternate trigger that survives desktop key bindings.
- **G2** The Inspection panel gains a **Recording** section that exports the
  current viewpoint as an animated GIF with configurable fps and duration.
- **G3** The same section exports AVI video (MJPEG) with the same
  fps/duration settings, encoded fully in-tree — no external tools, no new
  heavyweight dependencies.
- **G4** Recording works while the user orbits the camera or the sequence
  player advances — whatever the viewport shows is what gets recorded,
  including a completely static scene.
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
2. **Hotkey**: F12, with Ctrl+Shift+S as an alternate trigger. F12 is a media
   key by default on macOS systems and is sometimes registered by third-party
   capture tools on other desktops. PrintScreen is deliberately not the
   binding: desktop environments grab it globally and the application never
   sees it.
3. **Defaults**: 10 fps and 3 s for both GIF and AVI, matching typical
   point-cloud acquisition rates (~10 Hz).

## 2. User experience

### 2.1 Screenshot hotkey

- **F12** (or **Ctrl+Shift+S**) saves the current main viewport as PNG. The
  Screenshot section also provides an **Export snapshot** button with the
  same behavior, so the feature is reachable without any keyboard shortcut.
- Output path resolution order:
  1. The path in the Inspection → Screenshot field, if set (the hotkey then
     appends `-HHMMSS` before `.png` so repeated presses never silently
     overwrite; the overwrite checkbox is ignored on the hotkey path).
  2. Otherwise `<cwd>/kpt-viewport-YYYYMMDD-HHMMSS.png`.
  Note: this default naming is new behavior. Today the native save button is
  disabled while the path is empty and `queueInspectionScreenshot` enforces
  the `.png` extension (`src/gui/app.cpp:1929`, `3620`); only the web build
  downloads a fixed `kpt-viewport.png` without a path.
- The saved path is written to the log panel. Failure (capture error, file
  exists, I/O error) also lands in the log.
- Guard mirrors the existing hotkey pattern at `src/gui/app.cpp:3320`: ignore
  while `io.WantTextInput` is set or an inspection scene transaction is
  active. There is no general modal-dialog guard in the codebase and none is
  added.

### 2.2 Recording UI (Inspection panel → new "Recording" header)

Shown when the scene has layers, next to the existing Screenshot header:

- **Format**: combo `GIF` / `AVI (MJPEG)`.
- **FPS**: slider/input 1–60, default 10.
- **Duration**: seconds, 1–60, default 3.
- **Output**: path input + `...` dialog; extension enforced to match format
  (`.gif` / `.avi`). A new `DialogTarget::InspectionRecordingOutput` must be
  wired in all three places: the enum (`src/gui/app.hpp:67`), the per-target
  extension filters in `openDialog` (`src/gui/app.cpp:2382`), and
  `applyDialogResult` (`src/gui/app.cpp:2523`). `dialog_paths.*` only
  normalizes paths; it is not the target registry.
- **Overwrite** checkbox, same semantics as screenshot.
- **Record / Cancel** button. During recording it becomes **Cancel** and a
  countdown (`3.2 s left, 96 frames`) is drawn next to it.
- Completion/failure arrives through the existing JobSystem job list and log.

Recording starts on the button press; the recorder captures whatever the
viewport shows — the user can orbit or play a sequence during the take.
Recording ends at the configured duration, on Cancel, or on a fatal capture
error. The viewport **render extent is frozen** at the size computed when the
take starts: resizing the window mid-take does not abort the recording, it
simply does not change the recorded frame size (encoders need a constant
frame size).

### 2.3 i18n

New keys under `gui.inspection.*` must be added to **both** `i18n/en.json`
and `i18n/zh-CN.json`; `cmake/i18n.cmake` reads both and configure fails when
a key is missing from one. No new i18n machinery.

## 3. Architecture

```
UI/render thread                          JobSystem worker (High priority)
----------------                          ----------------
App::drawViewport                         ExportJob (1 job per recording)
  └─ RecordingSession::tick(now)               └─ Encoder (one of)
       ├─ due? (wall-clock pacer)                ├─ GifEncoder      (gif-h)
       ├─ captureRgba()  ── bounded queue ──→    └─ AviMjpegEncoder (stb JPEG)
       └─ extent frozen at start size             queue entries: (frame, repeat)
```

### 3.1 RecordingSession (UI thread, lives in `App`)

State machine: `Idle → Recording → Finalizing → Idle`.

- `start(config)`: validates config, computes the viewport extent once (with
  the render scale pinned to 1.0) and caches it as the take's frame size,
  submits the ExportJob at `JobPriority::High`, starts the wall clock at the
  first `tick`.
- `tick(now)`: called from `drawViewport` at exactly the point where the
  screenshot deferral fires today (`src/gui/app.cpp:1989`) — the top of the
  draw, **before** `ViewportSession::draw()` runs its `resize`
  (`src/gui/viewport/session.cpp:125`). What `captureRgba()` returns there is
  the previously committed texture, which is the Metal-safe contract. This
  placement must not move into `draw()`.
  - The pacer computes `due = floor((now - start) * fps) - emitted`. If
    nothing is due, return. Otherwise capture once and hand the frame plus
    the due count to the queue.
  - **No render-counter gating.** The screenshot flow waits for
    `inspection_viewport_render_count_ + 1`, but the single-cloud renderer
    early-returns without encoding when the frame is unchanged
    (`src/gui/backend/opengl/point_renderer.cpp:1186`), so a static scene
    never advances the counter. Recording therefore captures on wall clock
    alone; the last committed texture stays valid and is simply re-read.
    Abort only when `captureRgba()` returns a genuine error.
  - While recording, `drawViewport` uses the cached extent and pins
    `render_scale` to 1.0, overriding the 0.75 interaction scale
    (`src/gui/app.cpp:1974`); otherwise orbiting mid-take would change the
    frame size. The extent is never recomputed during the take.
- `stop(reason)`: closes the frame queue; state becomes `Finalizing` until
  the job's UI completion fires (posted through `UiEvents::post`, not the
  JobSystem itself).
- `JobPriority::High` is required, not cosmetic: while the sequence player is
  active, the last worker accepts only High jobs
  (`src/gui/jobs/job_system.cpp:174`), and the default worker limit is half
  of `max_workers`. A Normal export job could starve while the bounded queue
  fills.
- `needsContinuousRedraw()` needs no change: the native loop already draws
  continuously; the throttling that function controls exists only in the web
  runtime, where recording is compiled out.

### 3.2 Frame queue

Bounded to 8 **distinct frames**; entries are `(Rgba8Image, repeat)` pairs.
The capture side never blocks the render loop and never drops a logical
frame: when the queue is full, the new due count is merged into the tail
entry's `repeat` (`tail.repeat += due`). The worker therefore always accounts
for every due slot and the output always contains exactly `fps × duration`
frames; a slow encoder only costs spatial novelty (the tail frame is
repeated), never timing. Worst-case queue memory at 1920×1080 RGBA is about
64 MiB. Merge events are counted and reported in the completion log.

### 3.3 Encoders (worker side, `src/gui/viewport/`)

```cpp
class RecordingEncoder {
public:
  virtual ~RecordingEncoder() = default;
  virtual Result<void> begin(PixelExtent extent, int fps) = 0;
  // Writes one queue entry: the frame emitted `repeat` times. Per-output-frame
  // timing is derived by the encoder from fps and its own output index, so a
  // merged entry never shifts the total duration.
  virtual Result<void> writeFrame(const Rgba8Image &frame, unsigned repeat) = 0;
  virtual Result<void> finish() = 0;   // flush, backfill, close, rename
  virtual void abort() = 0;            // cancel path: close, delete .tmp
};
```

Both encoders write to `<path>.tmp` and rename through the existing
atomic-replace helper only after a successful `finish()`. Cancellation
(`abort()`) deletes the `.tmp`; a process crash may leave the `.tmp` behind,
but the destination path never contains a truncated file. (The existing PNG
path has exactly this crash behavior — `.tmp` removal happens on the cancel
path, not in a destructor — and the docs do not promise more.)

- **GifEncoder** — wraps vendored gif-h. Delay is assigned **per output
  frame**, not per entry: for output frame index `n`,
  `delay_cs(n) = round((n + 1) * 100 / fps) - round(n * 100 / fps)`.
  This Bresenham form keeps the total duration exact for any fps
  (e.g. 7 fps → delays alternate 14/15 cs instead of a constant 14 cs that
  accumulates error). `repeat` emits that many output frames with their own
  computed delays.
- **AviMjpegEncoder** — writes a RIFF AVI with MJPEG compression:
  - Frames: RGBA8 → RGB, then `stbi_write_jpg_to_func` at **quality 95**
    into a memory buffer. Quality must exceed 90: at `quality <= 90` stb
    enables 4:2:0 chroma subsampling (`third_party/stb/stb_image_write.h:1479`)
    and edge-pads odd dimensions into 16×16 MCUs; at 91+ subsampling is off
    (4:4:4, 8×8 MCUs) and odd viewport sizes encode cleanly. `repeat`
    rewrites the same JPEG chunk with one `idx1` entry per chunk.
  - Container: `RIFF 'AVI '` with `hdrl` (`avih` + video `strl` with `strh`
    fourCC `MJPG` and `strf` BITMAPINFOHEADER), a `movi` list of `00db`
    chunks, and a trailing `idx1`. No OpenDML — takes are far below the
    2 GiB legacy limit.
  - **Write order**: frame counts in `avih`/`strh`, the `RIFF` size, and the
    `idx1` offsets are only known at the end. `finish()` writes `idx1`, seeks
    back to backfill `avih`/`strh`/RIFF sizes, closes, then renames.
    `abort()` skips backfill and deletes the `.tmp` — a cancelled take does
    not produce a playable prefix.
  - Size estimate: 1080p quality-95 JPEG ≈ 150–400 KB per frame; a 10 fps,
    60 s maximum take lands at roughly 90–240 MB worst case, typically far
    less on dark point-cloud scenes. Bounded by the 60 s duration cap.

### 3.4 JPEG entry point

`src/kpt/render/detail/stb_png.cpp` already defines
`STB_IMAGE_WRITE_IMPLEMENTATION`. The new JPEG wrapper
(`src/kpt/render/detail/stb_jpg.cpp`, same `kpt_render_core` target) must
**not** define it again; it includes the header and exposes
`writeJpegToMemory(rgb view, quality) → std::vector<std::uint8_t>` via
`stbi_write_jpg_to_func`.

### 3.5 Files

| Change | Location |
|---|---|
| Recording session + pacer (pure logic, testable) | `src/gui/viewport/recording.hpp/.cpp` (new target `kpt_viewport_recording`) |
| GIF encoder | `src/gui/viewport/gif_encoder.cpp` (same target) |
| AVI/MJPEG encoder | `src/gui/viewport/avi_mjpeg_encoder.cpp` (same target) |
| JPEG-to-memory wrapper | `src/kpt/render/detail/stb_jpg.cpp/.hpp` (in `kpt_render_core`) |
| Vendored gif-h | `third_party/gif-h-<pinned-commit>/gif.h` + license |
| App wiring: F12 hotkey, Recording UI, `tick` call, extent freeze, render-scale pin | `src/gui/app.hpp/.cpp` |
| Dialog target + filters + result handling | `src/gui/app.hpp` (enum), `src/gui/app.cpp` (`openDialog` filters, `applyDialogResult`) |
| i18n keys | `i18n/en.json`, `i18n/zh-CN.json` (both required) |
| CMake: `kpt_viewport_recording` target, gif-h include dir | `CMakeLists.txt`, `third_party/CMakeLists.txt` |

No new build-time or run-time external dependencies.

## 4. Timing and correctness rules

1. **Sampling clock** is `std::chrono::steady_clock`; `start` is taken at the
   first `tick`, not at button press, so the first emitted frame is the one
   the user saw when pressing Record.
2. **Frame count** is exactly `fps × duration`. Every due slot is accounted
   by the queue (captured or merged into the tail repeat); the last tick pads
   with repeats of the final capture if the display loop is slow.
3. **fps > display rate** works: every due slot beyond the first re-emits the
   latest capture. A warning is logged once ("target fps exceeds display
   rate").
4. **Static scenes record correctly.** The single-cloud renderer skips
   re-encoding unchanged frames, so recording never gates on the render
   counter; it re-reads the still-valid committed texture on wall clock.
5. **Delay rounding (GIF)**: per-output-frame Bresenham delays keep total
   duration exact for any fps.
6. **Cancellation** (`stop_token` from JobSystem) closes the queue, calls
   `abort()` on the encoder, and removes the partial `.tmp` file. The UI
   stays responsive throughout.
7. **One recording at a time**; the screenshot hotkey remains usable during a
   recording (independent single-frame path).

## 5. Backend notes

- **OpenGL (Linux/Windows)** and **WebGL** share one renderer implementation:
  the WebGL runtime creates an `OpenGLPointRenderer`
  (`src/gui/runtime/glfw_webgl_runtime.cpp:199`), and `captureRgba` exists
  once (`src/gui/backend/opengl/point_renderer.cpp:1618`). FBOs are rebuilt
  and `capture_ready_` cleared only on extent change, which the frozen take
  extent never triggers.
- **Metal (macOS)**: reading the previously committed texture at the top of
  the draw is the established screenshot contract, reused per recorded frame;
  no new synchronization. Metal likewise invalidates its last render only on
  extent change.
- **WebGL**: out of scope (section 1); `RecordingSession` is compiled out
  under `KPT_WEB_BUILD`, the panel header is hidden, and the hotkey is not
  installed.

## 6. Testing and verification

Follows the existing per-component test-executable convention (Catch2,
`CATCH_CONFIG_NO_COUNTER`, no POSIX signals; new targets that touch platform
paths also get `kpt_enable_windows_utf8_manifest`, as
`viewport_capture_tests` does at `CMakeLists.txt:1133`):

1. **Pacer unit tests** (`tests/viewport_recording_test.cpp` against
   `kpt_viewport_recording`): due-count math, repeat merging when the queue
   is full, exact total frame count under a slow encoder, fps above display
   rate, per-frame GIF delay sequence (e.g. 7 fps → 14/15 cs pattern),
   static-scene capture (no render-counter advance between ticks).
2. **GIF encoder smoke test**: encode two synthetic 64×64 frames, assert
   `GIF89a` magic, trailer byte, and non-trivial size.
3. **AVI encoder unit test**: encode two synthetic frames, then parse the
   output in the test: RIFF/AVI magic, backfilled `avih` frame rate and frame
   count, `strh` fourCC `MJPG`, `movi` chunk count, `idx1` entry count and
   offsets. Decode one embedded JPEG frame with vendored `stb_image` to
   confirm it round-trips. Also assert odd dimensions (e.g. 63×41) encode
   without error at quality 95.
4. **JPEG wrapper test**: `writeJpegToMemory` output decodes via
   `stb_image` to the same dimensions.
5. **Manual end-to-end** (the checks that matter):
   - Linux/X11: F12 → PNG appears; record 3 s GIF while orbiting → plays in a
     browser; record AVI → `ffplay`/VLC plays, reported fps and duration
     match the configuration; record a static scene → output still has the
     full frame count.
   - macOS Metal: same, confirming no capture-stale-frame artifacts.
   - Windows OpenGL: same, including an output path with non-ASCII
     characters.
6. CI: existing `kpt_gui_smoke` under xvfb is extended only if it stays
   non-flaky; otherwise component tests carry the weight.

## 7. Risks and mitigations

| Risk | Mitigation |
|---|---|
| `glReadPixels`/Metal readback stalls the render loop at high fps | Capture is viewport-sized; the queue decouples encoding; merge counting makes encoder stalls visible rather than silent |
| User resizes window mid-recording | Take extent is frozen at start; resize no longer aborts or corrupts the stream |
| AVI+MJPEG does not play in browsers and is larger than H.264 | Accepted per the self-contained decision; VLC/mpv/ffplay/Windows Media Player all play it; duration capped at 60 s bounds file size |
| GIF palette banding on gradients | gif-h 8-bit palette; acceptable for point clouds on dark background; dither left off by default for speed |
| Cancelled/crashed export leaves artifacts | Cancel deletes the `.tmp`; a crash may leave a `.tmp` but never a truncated file at the destination path |
| Export job starves during sequence playback | Export job submitted at `JobPriority::High` (last worker is reserved for High while the player is active) |
