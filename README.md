# kitti_pointcloud_tools (kpt)

A small self-use toolkit for converting, viewing, playing and rendering
KITTI-style point clouds. Built from target-scoped core, render, optional
PCLVisualizer, and GUI components.

## Features

- **3 core CLI tools**: `pc_convert`, `pc_batch_convert`, `pc_render`
- **2 optional PCLVisualizer tools**: `pc_viewer`, `pc_player`
- **7 formats**: `bin`, `pcd`, `ply`, `xyz`, `xyzi`, `xyzrgb`, `xyzrgbi`
- **Auto-detect** of ASCII subformat by per-line column count on read (3/4/6/7 cols)
- **Canonical point type** `kpt::PointXYZRGBI` (x, y, z, rgb, intensity) — custom-registered with PCL
- **Headless multi-view PNG rendering** (no display required for `pc_render`)
- **Sequence playback** with optional semantic labels, dual pose CSVs and per-frame snapshots
- **Optional Dear ImGui workbench** combining viewing, playback, conversion,
  batch conversion and multi-view rendering

## Dependencies

Provided by system packages or the pinned vcpkg manifest:

- **PCL** >= 1.14 (tested with 1.14)
- **OpenCV** >= 4.6 (tested with 4.6)
- **Eigen** 3
- **CMake** >= 3.21
- **Ninja** (validated with 1.11.1)
- **C++20** compiler (GCC 11+ / Clang 14+)
- OpenGL toolkit for `pc_viewer` / `pc_player` (VTK via PCL visualization)

Vendored under `third_party/` (no separate install needed):

- `spdlog` (logging)
- `popl` (CLI option parsing)
- `catch2` v2 (tests)
- `eigen` (fallback if system Eigen missing)
- `rapidcsv` (legacy CSV helper retained for compatibility)
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
prerequisites, all presets, vcpkg setup, optional PCL viewers, Unicode/CJK
paths, settings locations, backend status, and recorded acceptance evidence.

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

Converts one file to another format. ASCII output subformat is chosen by the
output extension by default; override with `--ascii-flavor`.

```bash
./build/pc_convert input.bin output.pcd
./build/pc_convert input.pcd output.xyz
./build/pc_convert input.pcd output.txt --ascii-flavor xyzi
```

Options:

| Flag | Description |
|------|-------------|
| `--ascii-flavor F` | `xyz\|xyzi\|xyzrgb\|xyzrgbi` (only affects ASCII output) |

Positional: `<input> <output>`.

### pc_batch_convert — batch directory converter

Walks a directory, filters by glob, converts each matching file to the target format.

```bash
./build/pc_batch_convert -i data/velodyne -o out_pcd -t pcd
./build/pc_batch_convert -i data/ -o out_bin -t bin -g '*.pcd'
./build/pc_batch_convert -i data/ -o out/ -t xyzrgbi --ascii-flavor xyz
```

Options:

| Flag | Description |
|------|-------------|
| `-i,--input-dir DIR` | input directory (required) |
| `-o,--output-dir DIR` | output directory (created if missing, required) |
| `-t,--to FMT` | `bin\|pcd\|ply\|xyz\|xyzi\|xyzrgb\|xyzrgbi` (required) |
| `-g,--glob PAT` | fnmatch pattern, default `*` |
| `--ascii-flavor F` | override ASCII subformat |

### pc_viewer — single-frame interactive viewer

Opens one point cloud in an interactive PCL visualizer window. Use mouse to
rotate/zoom/pan.

```bash
./build/pc_viewer data/000123.pcd
./build/pc_viewer frame.bin -c rgb -s 5 -b 0.1,0.1,0.1
```

Options:

| Flag | Default | Description |
|------|---------|-------------|
| `-c,--colorby MODE` | `intensity` | `intensity\|rgb\|z\|none` |
| `-s,--point-size N` | `3` | point size |
| `-b,--bg R,G,B` | `0,0,0` | background color (0-1) |

Positional: `<file>`.

### pc_player — sequence player

Iterates a directory of point cloud frames in order, with optional semantic
labels, pose compensation and per-frame PNG snapshots.

```bash
./build/pc_player -i sequences/00/velodyne
./build/pc_player -i seq/velodyne --label-dir seq/labels --poses seq/poses.txt -c label
./build/pc_player -i seq/ -f 20 --snapshot snap_ --snapshot-w 1280 --snapshot-h 720
./build/pc_player -i seq/ --snapshot out_ --snapshot-views front,top,topleftfront
```

Options:

| Flag | Default | Description |
|------|---------|-------------|
| `-i,--input-dir DIR` | — | input directory (required) |
| `-g,--glob PAT` | `*` | fnmatch pattern |
| `--label-dir DIR` | — | semantic label directory (one `.label` per frame) |
| `--poses FILE` | — | pose CSV (first trajectory) |
| `--poses2 FILE` | — | second pose CSV |
| `-c,--colorby MODE` | `intensity` | `intensity\|rgb\|z\|label\|none` |
| `-s,--point-size N` | `3` | point size |
| `--snapshot PREFIX` | — | enable snapshots, prefix for PNG filenames |
| `--snapshot-w N` | `640` | snapshot width |
| `--snapshot-h N` | `480` | snapshot height |
| `--snapshot-fov DEG` | `120` | snapshot field of view |
| `--snapshot-views LIST` | `all` | `all` or comma-separated view names |
| `-f,--fps N` | `10` | playback frames per second |

View names: `front right back left top bottom toprightfront topleftfront
botrightfront botleftfront`.

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
All formats round-trip through it; missing fields are filled with zeros on read.

### Read (`kpt::load`)

Format is detected by file extension. For ASCII formats the actual subformat
is auto-detected by column count per line, so the extension only routes to the
ASCII reader — a `.xyz` file with 7 columns parses as XYZRGBI.

| Format | Ext | Fields read | Notes |
|--------|-----|-------------|-------|
| Bin    | `.bin` | x, y, z, intensity | 16 bytes/point (4×float32). rgb=0 |
| PCD    | `.pcd` | x, y, z, rgb, intensity | via PCL |
| PLY    | `.ply` | x, y, z, rgb, intensity | via PCL |
| XYZ    | `.xyz` | x, y, z | rgb=0, intensity=0 |
| XYZI   | `.xyzi` | x, y, z, intensity | rgb=0 |
| XYZRGB | `.xyzrgb` | x, y, z, r, g, b | intensity=0 |
| XYZRGBI| `.xyzrgbi` | x, y, z, r, g, b, intensity | — |

ASCII auto-detect by column count (applies to all of XYZ/XYZI/XYZRGB/XYZRGBI):

| Cols | Interpreted as |
|------|-----------------|
| 3 | x, y, z |
| 4 | x, y, z, intensity |
| 6 | x, y, z, r, g, b |
| 7 | x, y, z, r, g, b, intensity |

Lines with other column counts are skipped (warned, up to 50).

### Write (`kpt::save`)

Format is detected by output extension; ASCII subformat follows the extension
unless overridden by `--ascii-flavor`.

| Format | Ext | Fields written | Notes |
|--------|-----|-----------------|-------|
| Bin    | `.bin` | x, y, z, intensity | 16 bytes/point |
| PCD    | `.pcd` | x, y, z, rgb, intensity | binary mode |
| PLY    | `.ply` | x, y, z, rgb, intensity | binary mode |
| XYZ    | `.xyz` | x, y, z | 6-decimal fixed |
| XYZI   | `.xyzi` | x, y, z, intensity | — |
| XYZRGB | `.xyzrgb` | x, y, z, r, g, b | r/g/b as int 0-255 |
| XYZRGBI| `.xyzrgbi` | x, y, z, r, g, b, intensity | — |

## Project Structure

```
kitti_pointcloud_tools/
├── CMakeLists.txt
├── cmake/                 # compiler warnings, sanitizers, static analyzers
├── third_party/           # vendored: spdlog, popl, catch2, eigen, rapidcsv
├── data/                  # sample point clouds
├── src/
│   ├── common/            # platform-neutral Result utility
│   ├── kpt/               # core, render, and optional PCL viewer libraries
│   │   ├── types.hpp      # PointXYZRGBI and data format types
│   │   ├── io/            # load/save for all 7 formats + format detect
│   │   ├── label/         # semantic label load + applyLabel coloring
│   │   ├── workflow/      # shared conversion and batch operations
│   │   ├── viewer/        # InteractiveViewer (PCL visualizer wrapper)
│   │   ├── player/        # SequencePlayer (frame loop, poses, snapshots)
│   │   └── render/        # renderMultiView (headless PNG via OpenCV)
│   ├── gui/               # portable app/model plus selected GPU runtime
│   ├── platform/          # Linux, Windows, and macOS paths/fonts/settings
│   └── cli/               # five CLI entry points (two are optional)
└── tests/                 # core, platform, dialog, viewport, and GUI tests
```

`kpt_core`, `kpt_render`, and optional `kpt_pcl_viewer` keep headless consumers
from directly inheriting PCLVisualizer. GUI composition selects exactly one
backend-specific target.

## License

BSD 3-Clause — see [LICENSE](LICENSE). Copyright (c) 2020, DengQi.
