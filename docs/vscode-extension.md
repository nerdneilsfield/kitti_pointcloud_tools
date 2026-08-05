# VS Code point-cloud extension

The extension is under `vscode/`. Point Cloud Tools is a read-only custom
editor backed by a single-threaded KPT WebAssembly codec and a Three.js
webview. It ships desktop and browser extension-host entries under the
`dengqi.pointcloud-tools` identifier.

Codec ABI v4 additionally exports in-memory conversion and optional per-point
U8 noise classes.
Three.js preserves the selected base color mode and can override non-zero
noise points with a configurable color.

## Sequence player

Run `KPT: Open Point Cloud Sequence`, then select multiple supported clouds
and, optionally, matching `.label` files plus up to two `.csv`/`.txt` pose
files. Clouds are naturally sorted by filename. If any label is selected,
every cloud must have a label with the same stem.

## Conversion

Run `KPT: Convert Point Cloud` or use the Explorer context menu. The command
reads through `workspace.fs`, converts with the same C++ codec compiled to
WebAssembly, and writes the explicitly selected destination through
`workspace.fs`. BIN, PCD, PLY, XYZ, XYZI, XYZRGB, and XYZRGBI are accepted as
both sources and destinations. It never overwrites the source path.

`KPT: Open Point Cloud` explicitly selects the KPT custom editor. Supported
files remain available through **Open With**, while generic `.bin` files are
not claimed as the default editor.

The host sends a catalog first and reads frames only when requested. The
webview keeps current, previous, and next decoded frames, prefetches neighbors,
and provides seek, play/pause, and 2/5/10/20 fps controls. Labels are applied
by the same C++ RangeNet mapping used by the native application. Pose rows may
be three translations or 12-value 3x4 transforms; two trajectories can be
drawn together.

## Build

Build the decoder first with Emscripten 6.0.5:

```bash
source /path/to/emsdk/emsdk_env.sh
cmake --preset wasm-decoder-release
cmake --build --preset wasm-decoder-release
```

Then build or package the extension:

```bash
npm --prefix vscode install
npm --prefix vscode run check
cd vscode && npx @vscode/vsce package
```

The build bundles Three.js and the decoder worker. The VSIX contains desktop
and browser extension-host JavaScript, webview JavaScript, worker JavaScript,
and codec WASM.

## Verification

```bash
# Full webview, WebGL shader, reload, controls, and all seven formats
KPT_WEB_HEADED=1 xvfb-run -a npm --prefix vscode run test:browser

# Real VS Code Electron and a non-file FileSystemProvider (remote-like URI)
xvfb-run -a npm --prefix vscode run test:vscode

# 100K, 1M, and 10M KITTI BIN benchmark
KPT_WEB_HEADED=1 xvfb-run -a npm --prefix vscode run benchmark
```

Measured in the development container:

| Points | BIN input | SoA output | Decoder only | Worker round trip incl. index |
|---:|---:|---:|---:|---:|
| 100,000 | 1.53 MiB | 1.53 MiB | 2 ms | 14 ms |
| 1,000,000 | 15.26 MiB | 15.26 MiB | 13 ms | 95 ms |
| 10,000,000 | 152.59 MiB | 152.59 MiB | 136 ms | 906 ms |

These figures are regression baselines, not cross-machine performance claims.

## Memory model

The extension calls the memory ABI directly; it no longer writes an input copy
to MEMFS. KITTI BIN additionally decodes straight into render SoA buffers,
without constructing the canonical AoS cloud:

```text
worker input       16 bytes/point
WASM input         16 bytes/point
WASM SoA           16 bytes/point
transferable SoA   16 bytes/point
                   ─────────────
BIN worker peak    64 bytes/point, excluding allocator overhead
```

PCD, PLY, and text formats use the same memory stream API but still materialize
the canonical cloud during their schema-rich decode. Exact `hasColor` and
`hasIntensity` values now come from format/header schema, not sampled values.
RGB storage is emitted only when the source schema actually contains color.

The decoder Worker builds an octree-derived ordering with leaves capped at
100,000 points, including deterministic index splitting for degenerate input.
The ordering and ranges are spatial-index metadata; they do not currently
provide per-leaf frustum culling. Clouds above 100,000 points also receive a
per-leaf sampled LOD; the renderer switches to it beyond 2.5 cloud radii,
reducing distant draw work while preserving full data for close inspection.

Sequence cache is also byte-bounded at 384 MiB. Frames above one third of that
budget are kept as current-frame-only and are not neighbor-prefetched.

## Runtime limits

`kpt.maxFileSizeMiB` defaults to 256 MiB, is capped at 512 MiB, and is checked
both with `workspace.fs.stat()` and against the returned byte length. Reloads
are coalesced so each editor has at most one extension-host read in flight.
Starting a new decode terminates the old worker, which cancels synchronous WASM
work and releases its heap. A provider read already in progress cannot be
forcibly cancelled by `workspace.fs`; its stale result is discarded before it
can enter the webview. Request IDs also prevent late errors or decode results
from replacing the current document.

Conversion applies the same input limit. Its encoded output exists once in
WASM memory and once as a `Uint8Array` before `workspace.fs.writeFile()`
completes.
