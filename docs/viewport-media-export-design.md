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
- **G2** The Player panel gains a **Recording** section that exports the
  playback pass as an animated GIF with configurable output fps and a seconds
  or point-cloud-frame limit.
- **G3** The same section exports AVI video (MJPEG) with the same fps/limit
  settings, encoded fully in-tree — no external tools, no new heavyweight
  dependencies.
- **G4** Recording follows the sequence playhead: each newly displayed
  point-cloud frame records whatever the viewport shows at that moment, so
  orbiting during playback is recorded. A paused viewer records nothing and
  consumes no budget.
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
  2. Otherwise `<Downloads>/kpt-viewport-YYYYMMDD-HHMMSS.png`, where
     `<Downloads>` is the platform downloads directory; the stamped path
     falls back to the process working directory only when the platform
     cannot provide one.
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

### 2.2 Recording UI (Player panel → new "Recording" section)

Shown in the Player panel next to the existing Snapshot export section, at
the same nesting level. The Viewer keeps screenshots only.

- **Format**: combo `GIF` / `AVI (MJPEG)`.
- **FPS**: input 1–60, default 10. This is the **output** video fps; it is
  independent of the player's playback fps.
- **Limit**: combo `Seconds` / `Point-cloud frames`, with one value field.
  The value is a positive count or `-1`, which records until the current
  playback pass reaches its natural end.
- **Output**: path input + `...` dialog; extension enforced to match format
  (`.gif` / `.avi`). A new `DialogTarget::InspectionRecordingOutput` must be
  wired in all three places: the enum (`src/gui/app.hpp:67`), the per-target
  extension filters in `openDialog` (`src/gui/app.cpp:2382`), and
  `applyDialogResult` (`src/gui/app.cpp:2523`). `dialog_paths.*` only
  normalizes paths; it is not the target registry.
- **Overwrite** checkbox, same semantics as screenshot.
- **Prepare recording / Cancel**. Prepare only **arms** the take: the UI
  shows `Waiting for playback to start` with a Cancel button, and nothing is
  captured, counted, or written until the viewer plays. Play begins on the
  frame visible when Play is pressed; if playback is already running when the
  take is prepared, it starts on the current playback frame or the next one
  without requiring another Play. A prepared take that reaches the pass
  boundary before capturing anything records that boundary frame, so a
  one-frame sequence still produces a file. While waiting or recording, all
  settings in the section are disabled so the prepared take cannot change.
- Completion/failure arrives through the existing JobSystem job list and log.

Recording captures each point-cloud sequence frame the playhead advances to —
the user can orbit or play a sequence during the take. Recording ends when the
selected limit is reached, when the current playback pass ends (`-1` or an
early end), on Cancel, or on a fatal capture error. A paused viewer neither
records static frames nor consumes either budget; resuming continues from the
consumed amount, and a manual seek consumes no played time and no point-cloud
frame. The viewport **render extent is frozen** at the size computed when the
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
App::draw / drawViewport                  ExportJob (1 job per recording)
  ├─ updatePlayback (pass boundary)              └─ Encoder (one of)
  └─ updatePlayerRecording(now)                     ├─ GifEncoder      (gif-h)
       ├─ newly displayed playback frame?           └─ AviMjpegEncoder (stb JPEG)
       ├─ captureRgba() ── bounded queue ──→
       └─ extent frozen at prepare size            queue entries:
                                                   (frame, repeat, source_frames)
```

### 3.1 RecordingSession (UI thread, lives in `App`)

State machine: `Idle → Waiting → Recording → Finalizing → Idle`.

- `prepare(config)`: validates config, computes the viewport extent once (with
  the render scale pinned to 1.0) and caches it as the take's frame size.
  Nothing else happens: no encoder job, no `.tmp`, no capture, no budget.
- `begin(now, playback_active)`: called on the first capturable playback frame;
  submits the ExportJob at `JobPriority::High` and starts the played-time
  clock used by a seconds budget.
- `addFrame(frame, now)`: called once per **newly displayed point-cloud
  sequence frame**, with `drawViewport` at the point where the screenshot
  deferral fires today — the top of the draw, **before**
  `ViewportSession::draw()` runs its `resize`. What `captureRgba()` returns
  there is the previously committed texture, which is the Metal-safe
  contract. This placement must not move into `draw()`. Output duration is
  owned by played time: each new capture writes the output slots that became
  due since the previous one, so a slow capture never stretches the video and
  the total stays within one output frame of the played duration. At a low
  output fps some displayed point-cloud frames may not get their own output
  frame; a take that ends before one slot still publishes one frame. When the
  take ends on the same tick as the final capture, that frame uses the
  previously measured display interval as its effective duration, so it is
  encoded whenever that interval covers an output sample. A manual seek is
  not a playback display, so it is not captured and consumes no point-cloud
  frame budget.
  - **No render-counter gating.** The screenshot flow waits for
    `inspection_viewport_render_count_ + 1`, but the single-cloud renderer
    early-returns without encoding when the frame is unchanged
    (`src/gui/backend/opengl/point_renderer.cpp:1186`), so a static scene
    never advances the counter. Recording follows playback, not wall clock,
    so a paused or static viewer records nothing new. Abort only when
    `captureRgba()` returns a genuine error.
  - `setPlaybackActive(active, now)` freezes and resumes the played-time
    clock, so pauses consume neither seconds nor frames.
  - At a pass boundary the caller waits until the boundary frame is captured,
    then calls `finish()`. If the boundary is reached before the prepared take
    captured a frame, that boundary frame is recorded first. A loop wrap
    therefore never records the first frame of the next pass.
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
- `needsContinuousRedraw()` returns true while a take is prepared or active so
  the app keeps drawing enough to observe playback advances and pass
  boundaries; recording itself is compiled out of the web runtime.

### 3.2 Frame queue

Bounded to 8 **distinct frames**; entries are
`(Rgba8Image, repeat, source_frames)` triples. The capture side never blocks
the render loop and never drops a displayed sequence frame: when the queue is
full, the new repeat count and source count are merged into the tail entry
(`tail.repeat += repeat`, `tail.source_frames += source_frames`). The worker
therefore accounts for every displayed sequence frame and every output frame;
a slow encoder only costs spatial novelty (the tail frame is repeated), never
timing. Worst-case queue memory at 1920×1080 RGBA is about 64 MiB. Merge
events are counted and reported in the completion log.

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
  - Size estimate: 1080p quality-95 JPEG ≈ 150–400 KB per frame; a take is
    bounded by the configured seconds/frames budget or the current playback
    pass, not by a fixed 60 s cap.

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
| Recording session + frame budget (pure logic, testable) | `src/gui/viewport/recording.hpp/.cpp` (new target `kpt_viewport_recording`) |
| GIF encoder | `src/gui/viewport/gif_encoder.cpp` (same target) |
| AVI/MJPEG encoder | `src/gui/viewport/avi_mjpeg_encoder.cpp` (same target) |
| JPEG-to-memory wrapper | `src/kpt/render/detail/stb_jpg.cpp/.hpp` (in `kpt_render_core`) |
| Vendored gif-h | `third_party/gif-h-<pinned-commit>/gif.h` + license |
| App wiring: F12 hotkey, Recording UI, `updatePlayerRecording` call, extent freeze, render-scale pin | `src/gui/app.hpp/.cpp` |
| Dialog target + filters + result handling | `src/gui/app.hpp` (enum), `src/gui/app.cpp` (`openDialog` filters, `applyDialogResult`) |
| i18n keys | `i18n/en.json`, `i18n/zh-CN.json` (both required) |
| CMake: `kpt_viewport_recording` target, gif-h include dir | `CMakeLists.txt`, `third_party/CMakeLists.txt` |

No new build-time or run-time external dependencies.

## 4. Timing and correctness rules

1. **Played time** is `std::chrono::steady_clock`; the clock starts at the
   first captured playback frame, not at button press, and freezes whenever
   playback is not advancing.
2. **Frame budget** counts displayed point-cloud sequence frames, not output
   frames or ticks: `N` covers `N` actual sequence frames, including the
   first captured one. Consecutive renders of the same point cloud do not
   double-count.
3. **Output fps and playback fps are independent.** Output slots are due by
   cumulative played time (`floor(played * fps + 0.5)`) and each capture
   writes the slots due since the previous one, so the output duration matches
   the played time within one output frame. A low output fps may skip displayed
   source frames rather than stretching the video; the point-cloud frame limit
   is still counted on displayed source frames. A seconds budget is measured
   on the same played clock and stops at the budget with the final displayed
   frame included.
4. **Static scenes and pauses record nothing.** Recording follows playback
   advances, so the single-cloud renderer's unchanged-frame skip and a paused
   viewer neither add frames nor consume budget.
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

1. **Session unit tests** (`tests/viewport_recording_test.cpp` against
   `kpt_viewport_recording`): output-repeat math, queue merge accounting,
   preparation writes nothing, `N` source frames under different output and
   display fps, paused seconds budget, cancellation, and capture failure.
   App-level tests in `tests/gui_test.cpp` cover preparation waiting, the
   first playback-advanced frame starting the take, manual seeks not
   consuming the frame budget, and the pass boundary (including the next
   loop pass) not adding a frame.
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
   - Linux/X11: F12 → PNG appears; prepare a GIF, press Play → the take
     starts and plays in a browser; record AVI → `ffplay`/VLC plays with the
     configured output fps; pause and resume → the budget does not advance
     while paused; `-1` → the take ends with the pass's last frame and does
     not record the next loop pass.
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
| AVI+MJPEG does not play in browsers and is larger than H.264 | Accepted per the self-contained decision; VLC/mpv/ffplay/Windows Media Player all play it; the configured seconds/frames budget or the playback pass bounds file size |
| GIF palette banding on gradients | gif-h 8-bit palette; acceptable for point clouds on dark background; dither left off by default for speed |
| Cancelled/crashed export leaves artifacts | Cancel deletes the `.tmp`; a crash may leave a `.tmp` but never a truncated file at the destination path |
| Export job starves during sequence playback | Export job submitted at `JobPriority::High` (last worker is reserved for High while the player is active) |
