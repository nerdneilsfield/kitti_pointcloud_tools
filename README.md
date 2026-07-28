# kitti_pointcloud_tools (kpt)

A small toolkit for converting, viewing, playing and rendering KITTI-style
point clouds. Its point-cloud types and seven file codecs are implemented
in-tree; GUI and headless renderers consume the same dependency-free cloud.

## Features

- **5 CLI tools**: conversion, rendering, native viewing and sequence playback
- **Optional headless renderer**: `pc_render` (`KPT_BUILD_RENDER=ON`)
- **7 formats**: `bin`, `pcd`, `ply`, `xyz`, `xyzi`, `xyzrgb`, `xyzrgbi`
- **Native codecs** for KITTI BIN, delimited text, PCD and PLY
- **Canonical point type** `kpt::PointXYZRGBI` (x, y, z, rgb, intensity)
- **Headless multi-view PNG rendering** (no display required for `pc_render`)
- **Sequence playback** with optional semantic labels, dual pose CSVs and per-frame snapshots
- **Optional Dear ImGui workbench** combining viewing, playback, conversion,
  batch conversion and multi-view rendering

## Dependencies

Provided by system packages or the pinned vcpkg manifest:

- **Eigen** 3
- **CMake** >= 3.21
- **Ninja** (validated with 1.11.1)
- **C++20** compiler (GCC 11+ / Clang 14+)

Vendored under `third_party/` (no separate install needed):

- `spdlog` (logging)
- `popl` (CLI option parsing)
- `catch2` v2 (tests)
- `eigen` (fallback if system Eigen missing)
- `rapidcsv` (legacy CSV helper retained for compatibility)
- `stb_image_write` (native PNG output; `stb_image` is test-only)
- Dear ImGui `v1.92.8-docking`, GLFW `3.4` and ImGuiFileDialog `v0.6.8`
  (only built when `KPT_BUILD_GUI=ON`)

## Build

Named presets are the supported build entry point. The Linux system-package
build is:

```bash
cmake --preset linux-system-debug
cmake --build --preset linux-system-debug
ctest --preset linux-system-debug
```

For conversion-only builds, disable the renderer, GUI, and tests:

```bash
cmake -S . -B build/convert-only -G Ninja \
  -DKPT_BUILD_RENDER=OFF -DKPT_BUILD_GUI=OFF -DKPT_BUILD_TESTS=OFF
cmake --build build/convert-only --target pc_convert pc_batch_convert
```

The pinned vcpkg manifest contains only platform font dependencies. Headless
PNG rendering uses vendored stb and adds no package-manager dependency.

Windows and macOS use the pinned vcpkg manifest:

```powershell
# x64 Native Tools Command Prompt for VS
$env:VCPKG_ROOT = "C:\src\vcpkg"
cmake --preset windows-x64-vcpkg-debug
cmake --build --preset windows-x64-vcpkg-debug
ctest --preset windows-x64-vcpkg-debug
```

```bash
# macOS arm64; run from a shell with Xcode command-line tools
export VCPKG_ROOT="$HOME/src/vcpkg"
cmake --preset macos-arm64-vcpkg-debug
cmake --build --preset macos-arm64-vcpkg-debug
ctest --preset macos-arm64-vcpkg-debug
```

The macOS presets currently build core/headless targets only. Native Metal GUI
work remains gated on implementation and target-hardware verification.
Windows OpenGL source support exists but has not yet been verified on a
Windows host.

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
- Player: open a sequence, apply optional labels/poses, seek/play/loop and
  export sequence snapshots
- Convert: convert one file
- Batch Convert: glob a directory and queue parallel conversions
- Render: queue the existing headless multi-view renderer per selected view

The central viewport supports orbit (left drag), pan (middle/right drag),
zoom (wheel), fit, display modes and the ten existing view presets. Background
work is shown in the Jobs panel; worker count defaults to half the detected
hardware threads and remains adjustable.

For a non-interactive OpenGL/ImGui startup check:

```bash
xvfb-run -a ./build/linux-system-debug/pc_gui --smoke-test
```

## Tools

Each tool supports `-h,--help` and `-l,--log-level` (`0=err 1=warn 2=info 3=debug`, default `2`).

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
to headless per-frame PNG export and exits:

```bash
./build/pc_player -i data/velodyne -g '*.bin' --fps 10
./build/pc_player -i data/velodyne --snapshot out/frame \
  --snapshot-views front,top
```

### pc_render — multi-view PNG snapshot (headless)

Renders a single point cloud to one PNG per requested view. No display needed.

```bash
./build/pc_render data/000123.pcd -o frame
# writes frame_front.png frame_right.png ... (10 views by default)
./build/pc_render frame.bin -o shot --views front,top --width 1920 --height 1080
```

Output filenames are `<prefix>_<view>.png`.

Options:

| Flag | Default | Description |
|------|---------|-------------|
| `-o,--output-prefix P` | — | output filename prefix (required) |
| `--width N` | `640` | image width |
| `--height N` | `480` | image height |
| `--fov DEG` | `120` | field of view |
| `--views LIST` | `all` | `all` or comma-separated view names |

Positional: `<file>`.

## Format Support

The canonical in-memory type is `kpt::PointXYZRGBI` (x, y, z, rgb, intensity).
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
| PCD    | `.pcd` | x, y, z, rgb, intensity | PCD 0.7 ASCII, binary and LZF `binary_compressed` |
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
| PCD    | `.pcd` | x, y, z, rgb, intensity | binary mode |
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
