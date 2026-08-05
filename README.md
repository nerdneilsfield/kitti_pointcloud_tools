# kitti_pointcloud_tools (kpt)

[简体中文](README.zh-CN.md) | English

A small toolkit for converting, viewing, playing and rendering KITTI-style
point clouds. Its point-cloud types and seven file codecs are implemented
in-tree; GUI and headless renderers consume the same dependency-free cloud.

## Features

- **5 CLI tools**: conversion, rendering, native viewing and sequence playback
- **Optional headless renderer**: `pc_render` (`KPT_BUILD_RENDER=ON`)
- **7 formats**: `bin`, `pcd`, `ply`, `xyz`, `xyzi`, `xyzrgb`, `xyzrgbi`
- **Native codecs** for KITTI BIN, delimited text, PCD and PLY
- **Canonical point type** `kpt::PointXYZRGBI` (x, y, z, rgb, intensity,
  optional U8 noise class)
- **Headless multi-view PNG rendering** (no display required for `pc_render`)
- **Sequence playback** with optional semantic labels, dual pose CSVs and per-frame snapshots
- **Optional Dear ImGui workbench** combining viewing, playback, conversion,
  batch conversion and multi-view rendering
- **Static WebAssembly workbench** with WebGL2 Viewer and Player
- **VS Code + Three.js extension** with seven formats, labels, poses,
  sequence playback, Worker decoding, and octree-derived LOD

## Platform status

| Platform | Architecture | GUI backend | Status |
|---|---|---|---|
| Linux | x86-64 | OpenGL 3.3 / X11 | Implemented and verified |
| Windows 10 1903+/11 | x86-64 | OpenGL 3.3 | Implemented; CI package verification enabled |
| macOS 13+ | arm64, x86-64 | Metal | Implemented; dual-architecture CI verification enabled |
| Browser | WASM | WebGL2 | Implemented; Chrome smoke verified |

The release workflow runs native lifecycle and packaged-app smoke tests on
separate Apple Silicon and Intel runners.

Browser builds use Emscripten pthreads and require cross-origin isolation.
See [WebAssembly / WebGL2 build](docs/web-build.md).
See [VS Code extension build, usage, and performance](docs/vscode-extension.md).

## Dependencies

### Common prerequisites

- **CMake** >= 3.21
- **Ninja** (validated with 1.11.1) and Git
- **C++20** compiler: GCC 11+, Clang 14+, Apple Clang from Xcode 14+, or
  Visual Studio 2022

Eigen is found from the host first and otherwise falls back to the vendored
copy. Conversion-only builds need no PCL, OpenCV, VTK, OpenGL, GLFW,
Fontconfig, or Freetype package.

Vendored under `third_party/` (no separate install needed):

- `spdlog` (logging)
- `popl` (CLI option parsing)
- `catch2` v2 (tests)
- `eigen` (fallback if system Eigen missing)
- `rapidcsv` (legacy CSV helper retained for compatibility)
- `stb_image_write` (native PNG output; `stb_image` is test-only)
- Dear ImGui `v1.92.8-docking` and ImGuiFileDialog `v0.6.8` (GUI or their
  platform-neutral tests); GLFW `3.4` (only when `KPT_BUILD_GUI=ON`)

### Linux dependencies

The verified Linux GUI uses X11 and OpenGL. On Ubuntu 22.04/24.04/26.04:

```bash
sudo apt update
sudo apt install build-essential cmake ninja-build git pkg-config \
  libfreetype-dev libfontconfig1-dev \
  libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev \
  libxcursor-dev libxi-dev xvfb
cmake --version  # must be 3.21 or newer
```

For a conversion-only build, the smaller package set is sufficient:

```bash
sudo apt install build-essential cmake ninja-build git
```

The `linux-vcpkg-*` presets obtain Freetype and Fontconfig from vcpkg, but
vendored GLFW still requires the host X11/OpenGL development packages above.

### Windows dependencies

Install:

1. Windows 10 1903+ or Windows 11.
2. Visual Studio 2022 with **Desktop development with C++**, MSVC, Windows SDK,
   CMake tools, and Ninja.
3. Git and a bootstrapped vcpkg checkout.

From **Developer PowerShell for VS 2022** with the x64 environment active:

```powershell
git clone https://github.com/microsoft/vcpkg C:\src\vcpkg
C:\src\vcpkg\bootstrap-vcpkg.bat
$env:VCPKG_ROOT = "C:\src\vcpkg"
```

The Windows executable manifest enables UTF-8 and long paths. Native codecs
use `std::filesystem::path` and wide Win32 boundaries.

### macOS dependencies

Install Xcode command-line tools, CMake, Ninja, Git, and vcpkg:

```bash
xcode-select --install
brew install cmake ninja git
git clone https://github.com/microsoft/vcpkg "$HOME/src/vcpkg"
"$HOME/src/vcpkg/bootstrap-vcpkg.sh"
export VCPKG_ROOT="$HOME/src/vcpkg"
```

Homebrew is only one way to install CMake/Ninja; any CMake >= 3.21 and
C++20-capable Xcode toolchain is valid.

## Build by platform

Named presets are the supported build entry point. The Linux system-package
build is:

```bash
cmake --preset linux-system-debug
cmake --build --preset linux-system-debug
xvfb-run -a ctest --preset linux-system-debug
./build/linux-system-debug/pc_gui
```

Use `linux-system-release` for Release. A Linux vcpkg build uses:

```bash
export VCPKG_ROOT=/absolute/path/to/vcpkg
cmake --preset linux-vcpkg-debug
cmake --build --preset linux-vcpkg-debug
xvfb-run -a ctest --preset linux-vcpkg-debug
```

Windows builds must run inside an activated MSVC x64 developer environment:

```powershell
$env:VCPKG_ROOT = "C:\src\vcpkg"
cmake --preset windows-x64-vcpkg-debug
cmake --build --preset windows-x64-vcpkg-debug
ctest --preset windows-x64-vcpkg-debug
.\build\windows-x64-vcpkg-debug\pc_gui.exe
```

Use `windows-x64-vcpkg-release` for Release. The package workflow builds and
tests this preset, then smoke-tests all six commands from the extracted ZIP.

macOS Apple Silicon:

```bash
export VCPKG_ROOT="$HOME/src/vcpkg"
cmake --preset macos-arm64-vcpkg-debug
cmake --build --preset macos-arm64-vcpkg-debug
ctest --preset macos-arm64-vcpkg-debug
```

Intel Mac uses `macos-x64-vcpkg-debug`; each also has a `-release` preset.
The arm64 preset builds the complete Metal GUI and automated backend tests.

### Conversion-only build

Disable the renderer, GUI, and tests:

```bash
cmake -S . -B build/convert-only -G Ninja \
  -DKPT_BUILD_RENDER=OFF -DKPT_BUILD_GUI=OFF -DKPT_BUILD_TESTS=OFF
cmake --build build/convert-only --target pc_convert pc_batch_convert
```

This target graph needs only the compiler, CMake, Ninja, Git, and standard
system runtime libraries.

### Headless rendering without GUI

Build the CPU renderer, `pc_render`, and display-free `pc_player --snapshot`
without GLFW/OpenGL:

```bash
cmake -S . -B build/headless -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DKPT_BUILD_RENDER=ON -DKPT_BUILD_GUI=OFF -DKPT_BUILD_TESTS=OFF
cmake --build build/headless --target pc_render pc_player
```

The pinned vcpkg manifest keeps font dependencies in its opt-in
`platform-fonts` feature; presets that need external font libraries request
it. Headless PNG rendering uses vendored stb and adds no package-manager
dependency.

### Preset summary

| Platform | Debug preset | Release preset | GUI |
|---|---|---|---|
| Linux system packages | `linux-system-debug` | `linux-system-release` | OpenGL/X11, verified |
| Linux vcpkg | `linux-vcpkg-debug` | `linux-vcpkg-release` | OpenGL/X11 |
| Windows x64 vcpkg | `windows-x64-vcpkg-debug` | `windows-x64-vcpkg-release` | OpenGL, packaged CI smoke |
| macOS arm64 vcpkg | `macos-arm64-vcpkg-debug` | `macos-arm64-vcpkg-release` | Metal, verified |
| macOS x64 vcpkg | `macos-x64-vcpkg-debug` | `macos-x64-vcpkg-release` | Metal, Intel CI smoke |

See [Cross-platform build guide](docs/cross-platform-build.md) for exact
prerequisites, all presets, vcpkg setup, Unicode/CJK paths, settings
locations, backend status, and recorded acceptance evidence.

Examples below use `./build/<tool>` as shorthand. For a preset build, replace
it with `./build/<preset-name>/<tool>`.

### GUI workbench (Linux/X11)

The GUI is opt-in and builds entirely from vendored sources. System X11 and
OpenGL development libraries are still required.

```bash
cmake --preset linux-system-debug
cmake --build --preset linux-system-debug
./build/linux-system-debug/pc_gui
```

`pc_gui` provides five dockable tools:

- Viewer: load a point cloud into the OpenGL viewport
- Player: open a sequence, apply optional labels/poses, seek, reset, play
  forward or in reverse, loop and export sequence snapshots
- Convert: convert one file
- Batch Convert: glob a directory and queue parallel conversions
- Render: queue the existing headless multi-view renderer per selected view

The central viewport follows CloudCompare's object-centered mouse controls:
left drag uses trackball rotation around the selected pivot, Shift+left drag
rolls, right drag pans in the screen plane, middle click picks a new rotation
center, and the wheel zooms.
Dragging remains captured after the pointer leaves the viewport. Fit, display
modes and CloudCompare's eight standard view presets remain available.
Background work is shown in the Jobs panel; worker count defaults to half the
detected hardware threads and remains adjustable.

For a non-interactive OpenGL/ImGui startup check:

```bash
xvfb-run -a ./build/linux-system-debug/pc_gui --smoke-test
```

## Packages and releases

`VERSION` is the release version source for CMake, DEB, DMG, Windows ZIP, and VSIX
artifacts. Update every checked-in manifest together with:

```bash
./tools/set-version.sh 0.1.2
```

Reproducible Ubuntu packages require Docker:

```bash
./tools/package-deb.sh 22.04
./tools/package-deb.sh 24.04
./tools/package-deb.sh 26.04
```

On macOS 13 or newer with `VCPKG_ROOT` configured, build the ad-hoc-signed
universal2 DMG with `./tools/package-macos.sh`. On Windows with Visual Studio
2022 and `VCPKG_ROOT`, run `./tools/package-windows.ps1` from PowerShell to
build the portable x64 ZIP. Activate Emscripten 6.0.5, then run
`./tools/package-vsix.sh` to build the VS Code extension. Outputs are written
to `artifacts/`.

Release assets comprise three native Ubuntu amd64 DEBs (22.04, 24.04, and
26.04), one universal2 DMG, one Windows x64 portable ZIP, one VSIX, and a
`SHA256SUMS` file covering all six packages.

Pushing a matching `vX.Y.Z` tag runs all package jobs and publishes a GitHub
Release only after every artifact passes verification. A manual workflow run
uploads CI artifacts without creating a Release. The macOS DMG is ad-hoc
signed but not notarized; use Finder's **Open** context-menu action when
Gatekeeper reports an unidentified developer.

## Tools

Each tool supports `-h,--help` and `-l,--log-level` (`0=err 1=warn 2=info 3=debug`, default `2`).
GUI startup, OpenGL adapter details, framebuffer metrics, file loads and job
results are written to the terminal as well as the in-app log where applicable.
Levels `0` and `1` are intentionally quiet during successful operation.

### pc_convert — single-file converter

Converts one file to another format. Text output schema is selected by the
output extension. `--ascii-flavor`, when supplied for compatibility, must
match that extension.

```bash
./build/pc_convert input.bin output.pcd
./build/pc_convert input.pcd output.xyz
./build/pc_convert input.pcd output.xyzi --ascii-flavor xyzi
```

Options:

| Flag | Description |
|------|-------------|
| `--ascii-flavor F` | `xyz\|xyzi\|xyzrgb\|xyzrgbi`; must match output extension |

Positional: `<input> <output>`.

### pc_batch_convert — batch directory converter

Walks a directory, filters by glob, converts each matching file to the target format.

```bash
./build/pc_batch_convert -i data/velodyne -o out_pcd -t pcd
./build/pc_batch_convert -i data/ -o out_bin -t bin -g '*.pcd'
./build/pc_batch_convert -i data/ -o out/ -t xyzrgbi
```

Options:

| Flag | Description |
|------|-------------|
| `-i,--input-dir DIR` | input directory (required) |
| `-o,--output-dir DIR` | output directory (created if missing, required) |
| `-t,--to FMT` | `bin\|pcd\|ply\|xyz\|xyzi\|xyzrgb\|xyzrgbi` (required) |
| `-g,--glob PAT` | fnmatch pattern, default `*` |
| `--ascii-flavor F` | compatibility option; must match `--to` |

### pc_viewer — native interactive viewer

Uses the same GPU renderer and workbench loop as `pc_gui`, without PCL:

```bash
./build/pc_viewer data/frame.pcd --colorby intensity --point-size 3
```

### pc_player — native sequence player

Uses the same workbench for interactive playback. `--snapshot PREFIX` switches
to display-free per-frame PNG export and exits; this mode builds with
`KPT_BUILD_RENDER=ON` even when `KPT_BUILD_GUI=OFF`:

```bash
./build/pc_player -i data/velodyne -g '*.bin' --fps 10
./build/pc_player -i data/velodyne --snapshot out/frame \
  --snapshot-views front,top
```

Snapshots default to orthographic robust framing. Use
`--snapshot-trim-percent 0` to retain all finite points, or
`--snapshot-projection perspective --snapshot-fov 120` for perspective
framing.

PNG dimensions are limited to 32 Mi pixels because vendored
`stb_image_write` buffers filtered and compressed output in memory.

### pc_render — multi-view PNG snapshot (headless)

Renders a single point cloud to one PNG per requested view. No display needed.

```bash
./build/pc_render data/000123.pcd -o frame
# writes frame_front.png frame_right.png ... (10 views by default)
./build/pc_render frame.bin -o shot --views front,top --color-by z \
  --width 1920 --height 1080
```

Output filenames are `<prefix>_<view>.png`. `--color-by auto` selects RGB when
visible RGB values exist, otherwise normalized intensity grayscale, then
neutral gray. Explicit modes select RGB, intensity grayscale, a blue-green-red
Z-height gradient, or neutral solid gray. Explicit `rgb` fails clearly when
the cloud has no visible RGB values. By default, each axis discards its lowest
and highest 1% order-statistic tails, points outside that XYZ box are excluded,
and each view is orthographically fitted with a 5% margin. The CLI reports
input/framed length-width-height, retained-point ratio, and each image's
visible-pixel count.

Options:

| Flag | Default | Description |
|------|---------|-------------|
| `-o,--output-prefix P` | — | output filename prefix (required) |
| `--width N` | `640` | image width |
| `--height N` | `480` | image height |
| `--projection MODE` | `orthographic` | `orthographic` or `perspective` |
| `--trim-percent P` | `1` | percentage removed from each axis tail; `0` disables |
| `--fov DEG` | `120` | perspective field of view; requires perspective projection |
| `--views LIST` | `all` | `all` or comma-separated view names |
| `--color-by MODE` | `auto` | `auto`, `rgb`, `intensity`, `z`, or `solid` |

To reproduce the former framing, use
`--projection perspective --trim-percent 0 --fov 120`.

Positional: `<file>`.

## Format Support

The canonical in-memory type is `kpt::PointXYZRGBI` (x, y, z, rgb, intensity,
noise). PCD fields named `noise`, `is_noise`, or `noise_class` are accepted as
`SIZE 1 TYPE U COUNT 1`; zero means valid and any non-zero value means noise.
All formats convert through it. Only fields represented by both source and
destination round-trip; missing fields are zero and subset formats
intentionally drop unrepresented attributes.
See the [native I/O design](docs/superpowers/specs/2026-07-28-native-pointcloud-io-design.md)
for schema aliases, loss rules and parser limits.

### Read (`kpt::load`)

Format is detected by file extension. Each text extension selects an exact
column schema; malformed rows are skipped with bounded warnings.

| Format | Ext | Fields read | Notes |
|--------|-----|-------------|-------|
| Bin    | `.bin` | x, y, z, intensity | 16 bytes/point, IEEE-754 little-endian float32; rgb=0 |
| PCD    | `.pcd` | x, y, z, rgb, intensity, optional U8 noise | PCD 0.7 ASCII, binary and LZF `binary_compressed` |
| PLY    | `.ply` | x, y, z, rgb, intensity | PLY 1.0 ASCII, binary little-endian and binary big-endian |
| XYZ    | `.xyz` | x, y, z | rgb=0, intensity=0 |
| XYZI   | `.xyzi` | x, y, z, intensity | rgb=0 |
| XYZRGB | `.xyzrgb` | x, y, z, r, g, b | intensity=0 |
| XYZRGBI| `.xyzrgbi` | x, y, z, r, g, b, intensity | — |

Text schemas:

| Cols | Interpreted as |
|------|-----------------|
| 3 | x, y, z |
| 4 | x, y, z, intensity |
| 6 | x, y, z, r, g, b |
| 7 | x, y, z, r, g, b, intensity |

Blank and `#` comment lines are ignored. Rows with the wrong column count or
RGB values outside integral `[0,255]` are skipped; individual warnings stop
after 50 rows.

### Write (`kpt::save`)

Format is detected by output extension. CLI `--ascii-flavor` is accepted only
when it matches that extension/target format.

| Format | Ext | Fields written | Notes |
|--------|-----|-----------------|-------|
| Bin    | `.bin` | x, y, z, intensity | IEEE-754 little-endian float32 |
| PCD    | `.pcd` | x, y, z, rgb, intensity, optional U8 noise | binary mode |
| PLY    | `.ply` | x, y, z, rgb, intensity | binary little-endian |
| XYZ    | `.xyz` | x, y, z | locale-independent, float round-trip precision |
| XYZI   | `.xyzi` | x, y, z, intensity | — |
| XYZRGB | `.xyzrgb` | x, y, z, r, g, b | r/g/b as int 0-255 |
| XYZRGBI| `.xyzrgbi` | x, y, z, r, g, b, intensity | — |

## Project Structure

```
kitti_pointcloud_tools/
├── CMakeLists.txt
├── cmake/                 # compiler warnings, sanitizers, static analyzers
├── third_party/           # vendored: spdlog, popl, catch2, eigen, stb, ...
├── data/                  # sample point clouds
├── src/
│   ├── common/            # platform-neutral Result utility
│   ├── kpt/               # native point-cloud core and headless renderer
│   │   ├── types.hpp      # PointXYZRGBI and data format types
│   │   ├── io/            # native codecs for all 7 formats + detection
│   │   ├── label/         # semantic label load + applyLabel coloring
│   │   ├── workflow/      # shared conversion and batch operations
│   │   └── render/        # native CPU multi-view renderer + stb PNG output
│   ├── gui/               # portable app/model plus selected GPU runtime
│   ├── platform/          # Linux, Windows, and macOS paths/fonts/settings
│   └── cli/               # conversion, render, viewer and player entry points
└── tests/                 # core, platform, dialog, viewport, and GUI tests
```

`kpt_core` owns plain point types and native codecs; `kpt_render` adds a native
CPU renderer and vendored stb PNG output. GUI composition selects exactly one
backend-specific target.

## License

BSD 3-Clause — see [LICENSE](LICENSE). Copyright (c) 2020, DengQi.
