# kitti_pointcloud_tools (kpt)

A small self-use toolkit for converting, viewing, playing and rendering KITTI-style
point clouds. Built around a thin `kpt` C++ library and five CLI tools.

## Features

- **5 CLI tools**: `pc_convert`, `pc_batch_convert`, `pc_viewer`, `pc_player`, `pc_render`
- **7 formats**: `bin`, `pcd`, `ply`, `xyz`, `xyzi`, `xyzrgb`, `xyzrgbi`
- **Auto-detect** of ASCII subformat by per-line column count on read (3/4/6/7 cols)
- **Canonical point type** `kpt::PointXYZRGBI` (x, y, z, rgb, intensity) — custom-registered with PCL
- **Headless multi-view PNG rendering** (no display required for `pc_render`)
- **Sequence playback** with optional semantic labels, dual pose CSVs and per-frame snapshots

## Dependencies

All system-installed:

- **PCL** >= 1.14 (tested with 1.14)
- **OpenCV** >= 4.6 (tested with 4.6)
- **Eigen** 3
- **CMake** >= 3.10
- **C++20** compiler (GCC 11+ / Clang 14+)
- OpenGL toolkit for `pc_viewer` / `pc_player` (VTK via PCL visualization)

Vendored under `third_party/` (no separate install needed):

- `spdlog` (logging)
- `popl` (CLI option parsing)
- `catch2` v2 (tests)
- `eigen` (fallback if system Eigen missing)
- `rapidcsv` (pose CSV parsing)

## Build

```bash
cmake -B build && cmake --build build -j
```

Binaries land in `build/`. Tests (optional, on by default):

```bash
cmake --build build --target kpt_tests && ctest --test-dir build
```

Disable tests with `cmake -B build -DENABLE_TESTING=OFF`.

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
│   ├── kpt/               # the library
│   │   ├── types.hpp      # PointXYZRGBI, Format, ColorBy, View enums
│   │   ├── io/            # load/save for all 7 formats + format detect
│   │   ├── label/         # semantic label load + applyLabel coloring
│   │   ├── viewer/        # InteractiveViewer (PCL visualizer wrapper)
│   │   ├── player/        # SequencePlayer (frame loop, poses, snapshots)
│   │   └── render/        # renderMultiView (headless PNG via OpenCV)
│   ├── cli/               # five CLI entry points (one .cc per tool)
│   └── rapidcsv.h
└── tests/                 # Catch2 tests: io_test, label_test, render_test
```

The `kpt` static library exposes a small surface: `load`, `save`,
`InteractiveViewer`, `SequencePlayer`, `renderMultiView`, and `applyLabel`.

## License

BSD 3-Clause — see [LICENSE](LICENSE). Copyright (c) 2020, DengQi.
